#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <fstream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cartesian_path_demo",
                    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&]() { executor.spin(); });

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Logger
    auto logger = rclcpp::get_logger("cartesian_path_demo");

    // MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    // 获取当前位姿
    auto start_pose = move_group.getCurrentPose();
    RCLCPP_INFO(logger, "当前末端位姿: x=%.3f, y=%.3f, z=%.3f",
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                start_pose.pose.position.z);

    // 笛卡尔路径
    std::vector<geometry_msgs::msg::Pose> waypoints;
    geometry_msgs::msg::Pose target_pose = start_pose.pose;
    target_pose.position.x = 0.056;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.203;
    waypoints.push_back(target_pose);

    // 记录/joint_states
    bool record_flag = false;
    std::vector<sensor_msgs::msg::JointState> joint_state_log;

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (record_flag)
                joint_state_log.push_back(*msg);
        });

    // 规划笛卡尔轨迹
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    std::cout << fraction << std::endl;
    if (fraction > 0.8)
    {
        RCLCPP_INFO(logger, "笛卡尔路径规划成功 (%.1f%%)，开始执行并记录 /joint_states", fraction * 100.0);

        // 开始记录
        record_flag = true;

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        move_group.execute(plan);

        // 执行完成，停止记录
        record_flag = false;

        // 保存 joint_states 到文件
        std::ofstream file("joint_states_log.csv");
        file << "time";
        for (const auto &name : joint_state_log.front().name)
            file << "," << name;
        file << std::endl;

        for (const auto &js : joint_state_log)
        {
            file << js.header.stamp.sec + js.header.stamp.nanosec*1e-9;
            for (const auto &pos : js.position)
                file << "," << pos;
            file << std::endl;
        }
        file.close();

        RCLCPP_INFO(logger, "joint_states 已保存到 joint_states_log.csv");
    }
    else
    {
        RCLCPP_WARN(logger, "笛卡尔路径规划成功率低，未执行");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
