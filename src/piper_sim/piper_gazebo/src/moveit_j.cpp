#include <memory>
#include <thread>
#include <fstream>
#include <vector>
#include <iostream>
#include <random>
#include <cmath>  // for M_PI
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <sensor_msgs/msg/joint_state.hpp>

int main(int argc, char *argv[])
{
    // 初始化 ROS2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "joint_space_motion_demo",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner([&]() { executor.spin(); });

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));
    auto logger = rclcpp::get_logger("joint_space_motion_demo");

    // 创建 MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");

    // 创建 PlanningSceneMonitor（用于碰撞检测）
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
        node, "robot_description", tf_buffer);

    if (!planning_scene_monitor->getPlanningScene())
    {
        RCLCPP_ERROR(logger, "无法获取 PlanningScene，无法进行碰撞检测！");
        rclcpp::shutdown();
        return 1;
    }

    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->startStateMonitor();

    // 获取当前关节状态
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();
    RCLCPP_INFO(logger, "当前关节角度：");
    for (size_t i = 0; i < current_joint_values.size(); ++i)
        RCLCPP_INFO(logger, "Joint %ld: %.3f", i + 1, current_joint_values[i]);

    // 定义角度转弧度
    auto deg2rad = [](double deg) { return deg * M_PI / 180.0; };
    std::random_device rd;
    std::mt19937 gen(rd());

    // 每个关节的范围
    std::uniform_real_distribution<> joint1(deg2rad(-150), deg2rad(150));
    std::uniform_real_distribution<> joint2(deg2rad(0),    deg2rad(180));
    std::uniform_real_distribution<> joint3(deg2rad(-170), deg2rad(0));
    std::uniform_real_distribution<> joint4(deg2rad(-100), deg2rad(100));
    std::uniform_real_distribution<> joint5(deg2rad(-70),  deg2rad(70));
    std::uniform_real_distribution<> joint6(deg2rad(-120), deg2rad(120));

    // 生成随机目标关节角度
    std::vector<double> target_joint_values(6);
    target_joint_values[0] = joint1(gen);
    target_joint_values[1] = joint2(gen);
    target_joint_values[2] = joint3(gen);
    target_joint_values[3] = joint4(gen);
    target_joint_values[4] = joint5(gen);
    target_joint_values[5] = joint6(gen);

    RCLCPP_INFO(logger, "随机生成的目标关节角度:");
    for (size_t i = 0; i < target_joint_values.size(); ++i)
        RCLCPP_INFO(logger, "Joint %ld: %.3f rad (%.2f deg)",
                    i + 1, target_joint_values[i], target_joint_values[i] * 180.0 / M_PI);

    move_group.setJointValueTarget(target_joint_values);

    // 规划路径
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!success)
    {
        RCLCPP_ERROR(logger, "路径规划失败！");
        rclcpp::shutdown();
        spinner.join();
        return 0;
    }

    // === ✅ 添加：碰撞检测 ===
    planning_scene_monitor->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor);

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_request.group_name = move_group.getName();  // 检查"arm"组
    scene->checkCollision(collision_request, collision_result);

    if (collision_result.collision)
    {
        RCLCPP_WARN(logger, "⚠️ 检测到碰撞（可能与地面或其他障碍物），取消执行！");
        rclcpp::shutdown();
        spinner.join();
        return 0;
    }

    // === ✅ 如果没有碰撞，执行并记录 joint_states ===
    RCLCPP_INFO(logger, "路径安全，无碰撞，开始执行...");

    bool record_flag = false;
    std::vector<sensor_msgs::msg::JointState> joint_state_log;

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (record_flag)
                joint_state_log.push_back(*msg);
        });

    record_flag = true;
    move_group.execute(plan);
    record_flag = false;

    // 保存 joint_states
    if (!joint_state_log.empty())
    {
        std::ofstream file("joint_states_log.csv");
        file << "time";
        for (const auto &name : joint_state_log.front().name)
            file << "," << name;
        file << std::endl;

        for (const auto &js : joint_state_log)
        {
            double t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9;
            file << t;
            for (const auto &pos : js.position)
                file << "," << pos;
            file << std::endl;
        }
        file.close();
        RCLCPP_INFO(logger, "joint_states 已保存到 joint_states_log.csv");
    }

    rclcpp::shutdown();
    spinner.join();
    return 0;
}
