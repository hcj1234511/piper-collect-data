#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

int main(int argc, char *argv[])
{
  // 初始化 ROS2
  rclcpp::init(argc, argv);
      auto node = std::make_shared<rclcpp::Node>("get_current_pose_node",
                    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&]() { executor.spin(); });
  
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  
  // 创建 MoveGroupInterface，用于控制和查询机械臂
  static const std::string PLANNING_GROUP = "arm";  // ← 这里改成你的 MoveIt 规划组名称
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 获取当前末端执行器的位姿
  geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();

  // 打印结果
  RCLCPP_INFO(node->get_logger(), "=== 当前末端执行器位姿 ===");
  RCLCPP_INFO(node->get_logger(), "Frame ID: %s", current_pose.header.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "Position: [x: %.3f, y: %.3f, z: %.3f]",
              current_pose.pose.position.x,
              current_pose.pose.position.y,
              current_pose.pose.position.z);
  RCLCPP_INFO(node->get_logger(), "Orientation: [x: %.3f, y: %.3f, z: %.3f, w: %.3f]",
              current_pose.pose.orientation.x,
              current_pose.pose.orientation.y,
              current_pose.pose.orientation.z,
              current_pose.pose.orientation.w);

  rclcpp::shutdown();
  return 0;
}
