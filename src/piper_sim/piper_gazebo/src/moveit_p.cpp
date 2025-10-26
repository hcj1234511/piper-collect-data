#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <fstream>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ompl_path_demo",
                    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&]() { executor.spin(); });

    node->set_parameter(rclcpp::Parameter("use_sim_time", true));

    // Logger
    auto logger = rclcpp::get_logger("ompl_path_demo");

    // MoveGroup
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    move_group.setGoalPositionTolerance(0.01);
    move_group.setPlannerId("RRTConnect");

    // 获取规划组关节名称
    std::vector<std::string> joint_names = move_group.getJointNames();
    RCLCPP_INFO(node->get_logger(), "规划组关节名称：");
    for (const auto& name : joint_names)
        RCLCPP_INFO(node->get_logger(), "%s", name.c_str());

    // 获取当前末端位姿
    auto start_pose = move_group.getCurrentPose();
    RCLCPP_INFO(logger, "当前末端位姿: x=%.6f, y=%.6f, z=%.6f, ox=%.6f, oy=%.6f, oz=%.6f, ow=%.6f",
                start_pose.pose.position.x,
                start_pose.pose.position.y,
                start_pose.pose.position.z,
                start_pose.pose.orientation.x,
                start_pose.pose.orientation.y,
                start_pose.pose.orientation.z,
                start_pose.pose.orientation.w);

    // 设置目标位姿
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.056141;
    target_pose.position.y = 0.000005;
    target_pose.position.z = 0.213190;
    target_pose.orientation.x = -0.000005;
    target_pose.orientation.y = 0.675653;
    target_pose.orientation.z = -0.000022;
    target_pose.orientation.w = 0.737219;

    move_group.setPoseTarget(target_pose);

    // 记录 /joint_states
    bool record_flag = false;
    std::vector<sensor_msgs::msg::JointState> joint_state_log;

    auto joint_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            if (record_flag)
                joint_state_log.push_back(*msg);
        });

    // 使用 OMPL 规划路径
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = move_group.plan(plan);

    if (success)
    {
        RCLCPP_INFO(logger, "OMPL 规划成功，开始执行并记录 /joint_states");

        // 开始记录
        record_flag = true;

        move_group.execute(plan);

        // 执行完成，停止记录
        record_flag = false;

        // 保存 joint_states 到 CSV 文件
        if (!joint_state_log.empty())
        {
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
            RCLCPP_WARN(logger, "joint_state_log 为空，未保存数据");
        }
    }
    else
    {
        RCLCPP_WARN(logger, "OMPL 规划失败");
    }

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
