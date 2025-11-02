#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import math, os

# ====== 在这里写明 SDF 文件地址 ======
# 默认取脚本同目录下的 cube.sdf，你也可以改成绝对路径，比如："/home/xxx/models/cube.sdf"
USER_SET = "/home/hong/piper_ros/src/cube_description/cube.sdf"  # 改成你的绝对路径；或 "~/.gazebo/models/cube.sdf"
SDF_PATH = os.path.abspath(os.path.expanduser(USER_SET))


def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r*0.5), math.sin(r*0.5)
    cp, sp = math.cos(p*0.5), math.sin(p*0.5)
    cy, sy = math.cos(y*0.5), math.sin(y*0.5)
    return Quaternion(
        x=sr*cp*cy - cr*sp*sy,
        y=cr*sp*cy + sr*cp*sy,
        z=cr*cp*sy - sr*sp*sy,
        w=cr*cp*cy + sr*sp*sy
    )


class Spawner(Node):
    def __init__(self):
        super().__init__('sdf_spawner')
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        self.del_cli   = self.create_client(DeleteEntity, '/delete_entity')

    def wait(self):
        if not self.spawn_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('找不到 /spawn_entity 服务，请先启动 gazebo_ros。')
        self.del_cli.wait_for_service(timeout_sec=1.0)

    def delete(self, name):
        req = DeleteEntity.Request(); req.name = name
        fut = self.del_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)

    def try_spawn(self, name, xml, pose, ref='world'):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.robot_namespace = ''
        req.initial_pose = pose
        req.reference_frame = ref
        fut = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        return fut.result()  # 可能为 None


def main():
    ap = argparse.ArgumentParser(description='Spawn an SDF model into Gazebo (auto cubeN names).')
    ap.add_argument('--delete-name', type=str, default='', help='删除指定模型名并退出（例：cube1）')
    ap.add_argument('--name', type=str, default='', help='固定模型名；留空则自动使用 cube1, cube2, ...')
    ap.add_argument('--start-idx', type=int, default=1, help='自动编号起始（默认 1）')
    ap.add_argument('--x', type=float, default=0.5)
    ap.add_argument('--y', type=float, default=0.0)
    ap.add_argument('--z', type=float, default=0.02, help='默认放地面上方避免穿插')
    ap.add_argument('--roll',  type=float, default=0.0)
    ap.add_argument('--pitch', type=float, default=0.0)
    ap.add_argument('--yaw',   type=float, default=0.0)
    ap.add_argument('--replace', action='store_true', help='若指定 --name 且重名则先删除再生成')
    args = ap.parse_args()

    if not os.path.isfile(SDF_PATH):
        raise FileNotFoundError(f'SDF 文件不存在：{SDF_PATH}')

    with open(SDF_PATH, 'r', encoding='utf-8') as f:
        xml = f.read()

    rclpy.init()
    node = Spawner()
    try:
        node.wait()

        # 位姿
        pose = Pose()
        pose.position = Point()
        pose.position.x = args.x
        pose.position.y = args.y
        pose.position.z = args.z
        pose.orientation = rpy_to_quat(args.roll, args.pitch, args.yaw)

        # 1) 用户给了固定名字
        if args.name:
            if args.replace:
                node.delete(args.name)
            resp = node.try_spawn(args.name, xml, pose)
            if not resp or not resp.success:
                msg = None if not resp else resp.status_message
                raise RuntimeError(f'生成失败（name={args.name}）：{msg}')
            node.get_logger().info(f'生成成功：{resp.status_message}  name={args.name}')
            return

        # 2) 自动递增 cubeN
        base = 'cube'
        idx = max(1, args.start_idx)
        while True:
            name = f'{base}{idx}'
            resp = node.try_spawn(name, xml, pose)
            if resp and resp.success:
                node.get_logger().info(f'生成成功：{resp.status_message}  name={name}')
                break
            # 如果失败且是重名或其他原因，换下一个编号继续尝试
            idx += 1
            if idx - args.start_idx > 9999:
                raise RuntimeError('自动命名尝试过多（超过 1 万次），请检查仿真或命名策略。')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

