#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>
#include <fstream>
#include <typeinfo>
#include <random>
#include <cxxabi.h>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
    moveit::planning_interface::MoveGroupInterface gripper_group(node,"gripper");
    move_group.setGoalPositionTolerance(0.02);
    move_group.setGoalOrientationTolerance(0.05);
    move_group.setMaxVelocityScalingFactor(0.5);
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setPlannerId("RRTConnect");
    gripper_group.setMaxVelocityScalingFactor(0.05);
    gripper_group.setMaxAccelerationScalingFactor(0.05);

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

    geometry_msgs::msg::Pose target_pose;
    move_group.setRandomTarget();
    // move_group.setPoseTarget(target_pose);

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
        gripper_group.setNamedTarget("open");
        gripper_group.move();
        move_group.execute(plan);
        target_pose = move_group.getCurrentPose().pose;
        RCLCPP_INFO(logger, "目标点末端位姿: x=%.6f, y=%.6f, z=%.6f, ox=%.6f, oy=%.6f, oz=%.6f, ow=%.6f",
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z,
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w);
        gripper_group.setNamedTarget("close");
        gripper_group.move();
        // 执行完成，停止记录
        record_flag = false;

        // 保存 joint_states 到 ~/piper_ros/data/ 带时间戳的 CSV
        if (!joint_state_log.empty())
        {
            // 1) 目录：~/piper_ros/data
            const char* home = std::getenv("HOME");
            std::filesystem::path out_dir = (home ? std::filesystem::path(home) : std::filesystem::path(".")) / "piper_ros" / "data";
            std::error_code ec;
            std::filesystem::create_directories(out_dir, ec);

            // 2) 文件名：joint_states_YYYY-MM-DD_HH-MM-SS.csv
            auto now = std::chrono::system_clock::now();
            std::time_t tt = std::chrono::system_clock::to_time_t(now);
            std::tm tm {};
            localtime_r(&tt, &tm);

            std::ostringstream ts;
            ts << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
            std::string fname = "joint_states_" + ts.str() + ".csv";
            std::filesystem::path fullpath = out_dir / fname;

            // 3) 写入 CSV 文件
            std::ofstream file(fullpath);
            file << "time";
            for (const auto &name : joint_state_log.front().name)
                file << "," << name << "_pos";
            for (const auto &name : joint_state_log.front().name)
                file << "," << name << "_vel";
            file << "\n";

            for (const auto &js : joint_state_log)
            {
                const double t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9;
                file << std::fixed << std::setprecision(9) << t;

                // position
                for (const auto &pos : js.position)
                    file << "," << std::setprecision(9) << pos;
                // velocity
                if (js.velocity.size() == js.name.size()) {
                    for (const auto &vel : js.velocity)
                        file << "," << std::setprecision(9) << vel;
                } else {
                    for (size_t i = 0; i < js.name.size(); ++i)
                        file << ",0.0";  // 若无velocity则补0
                }
                file << "\n";
            }

            //  文件末尾追加 target_pose
            file << "# target_pose,"
                << std::fixed << std::setprecision(6)
                << target_pose.position.x << ","
                << target_pose.position.y << ","
                << target_pose.position.z << ","
                << target_pose.orientation.x << ","
                << target_pose.orientation.y << ","
                << target_pose.orientation.z << ","
                << target_pose.orientation.w << "\n";

            file.close();
            RCLCPP_INFO(logger, "joint_states(含pos/vel) 已保存到: %s", fullpath.string().c_str());
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
    move_group.setNamedTarget("zero");
    move_group.move();

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
