#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/Geometry>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <map>
#include <fstream>
#include <typeinfo>
#include <string>
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
    auto node = std::make_shared<rclcpp::Node>("ompl_path_demo");

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
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);
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
    
    // 声明参数（可被命令行 --ros-args 覆盖）
    node->declare_parameter<double>("x", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>("y", std::numeric_limits<double>::quiet_NaN());
    node->declare_parameter<double>("z", 0.15);
    node->declare_parameter<double>("ox", 0.0);
    node->declare_parameter<double>("oy", 1.0);
    node->declare_parameter<double>("oz", 0.0);
    node->declare_parameter<double>("ow", 0.0);
    node->declare_parameter<std::string>("home", "nan");
    // // 设置目标位姿
    // double r = 0;
    // double p = 3.14;
    // double y = 0;
    // tf2::Quaternion q;
    // q.setRPY(r,p,y);
    // geometry_msgs::msg::Quaternion q_msg = tf2::toMsg(q);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_x(0.1,0.4);
    std::uniform_real_distribution<double> dist_y(-0.3,0.3);
    

    // 读取参数（如果没有传则随机）
    double x = node->get_parameter("x").as_double();
    double y = node->get_parameter("y").as_double();
    double z = node->get_parameter("z").as_double();
    double ox = node->get_parameter("ox").as_double();
    double oy = node->get_parameter("oy").as_double();
    double oz = node->get_parameter("oz").as_double();
    double ow = node->get_parameter("ow").as_double();
    std::string home = node->get_parameter("home").as_string();
    if (home == "home")
    {
        move_group.setNamedTarget("zero");
        gripper_group.setNamedTarget("close");
        move_group.move();
        gripper_group.move();
        return 0;
    }

    if (std::isnan(x)) x = dist_x(gen);
    if (std::isnan(y)) y = dist_y(gen);
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.x = ox;
    target_pose.orientation.y = oy;
    target_pose.orientation.z = oz;
    target_pose.orientation.w = ow;

    RCLCPP_INFO(logger, "目标点末端位姿: x=%.6f, y=%.6f, z=%.6f, ox=%.6f, oy=%.6f, oz=%.6f, ow=%.6f",
                    target_pose.position.x,
                    target_pose.position.y,
                    target_pose.position.z,
                    target_pose.orientation.x,
                    target_pose.orientation.y,
                    target_pose.orientation.z,
                    target_pose.orientation.w);


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
        gripper_group.setNamedTarget("open");
        moveit::planning_interface::MoveGroupInterface::Plan plan_open;
        moveit::core::MoveItErrorCode success1 = gripper_group.plan(plan_open);
        if (success1)
            gripper_group.execute(plan_open);
        move_group.execute(plan);
        std::map<std::string, double> gripper_joint_values;
        // 物体尺寸为4cm，夹爪开口设置为3.2cm以产生足够夹持力且避免穿透
        // joint7=0.016时，joint8=-0.016，开口=3.2cm（比物体小0.8cm，避免穿透同时保持夹持力）
        gripper_joint_values["joint7"] = 0.017;
        gripper_group.setJointValueTarget(gripper_joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan2;
        moveit::core::MoveItErrorCode success2 = gripper_group.plan(plan2);
        if (success2)
        {
            RCLCPP_INFO(logger, "gripper plan2 成功");
            gripper_group.execute(plan2);
        }
        geometry_msgs::msg::Pose place_pose;
        place_pose.position.x = 0;
        place_pose.position.y = 0.3;
        place_pose.position.z = 0.2;
        place_pose.orientation.x = -0.7071;
        place_pose.orientation.y = 0.7071;
        place_pose.orientation.z = 0;
        place_pose.orientation.w = 0;
        move_group.setPoseTarget(place_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan_place;
        moveit::core::MoveItErrorCode success3 = move_group.plan(plan_place);
        if (success3)
            move_group.execute(plan_place);
        // 执行完成，停止记录
        gripper_group.setNamedTarget("open");
        moveit::planning_interface::MoveGroupInterface::Plan plan_end;
        moveit::core::MoveItErrorCode success4 = gripper_group.plan(plan_end);
        if (success4)
            gripper_group.execute(plan_end);
        record_flag = false;

        // 固定的关节顺序
        std::vector<std::string> joint_order = {
            "joint1", "joint2", "joint3", "joint4",
            "joint5", "joint6", "joint7", "joint8"
        };

        // 保存 joint_states 到 ~/piper_ros/data/
        if (!joint_state_log.empty())
        {
            const char* home = std::getenv("HOME");
            std::filesystem::path out_dir = (home ? std::filesystem::path(home) : std::filesystem::path(".")) / "piper_ros" / "data";
            std::error_code ec;
            std::filesystem::create_directories(out_dir, ec);

            // 时间戳文件名
            auto now = std::chrono::system_clock::now();
            std::time_t tt = std::chrono::system_clock::to_time_t(now);
            std::tm tm {};
            localtime_r(&tt, &tm);

            std::ostringstream ts;
            ts << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
            std::string fname = "joint_states_" + ts.str() + ".csv";
            std::filesystem::path fullpath = out_dir / fname;

            std::ofstream file(fullpath);
            file << "time";
            for (const auto &n : joint_order) file << "," << n << "_pos";
            for (const auto &n : joint_order) file << "," << n << "_vel";
            file << ",ee_x,ee_y,ee_z,ee_qx,ee_qy,ee_qz,ee_qw\n";

            for (const auto &js : joint_state_log)
            {
                const double t = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9;
                file << std::fixed << std::setprecision(9) << t;

                // 映射 joint_name → 值
                std::unordered_map<std::string, double> pos_map, vel_map;
                for (size_t i = 0; i < js.name.size(); ++i) {
                    pos_map[js.name[i]] = js.position[i];
                    if (i < js.velocity.size()) vel_map[js.name[i]] = js.velocity[i];
                }

                // 按固定顺序写入
                for (const auto &n : joint_order) {
                    double v = pos_map.count(n) ? pos_map[n] : 0.0;
                    file << "," << std::setprecision(9) << v;
                }
                for (const auto &n : joint_order) {
                    double v = vel_map.count(n) ? vel_map[n] : 0.0;
                    file << "," << std::setprecision(9) << v;
                }
                // 计算当前末端位姿（正向运动学）
                moveit::core::RobotState kinematic_state(move_group.getRobotModel());
                kinematic_state.setVariablePositions(js.name, js.position);
                const Eigen::Isometry3d& ee_pose = kinematic_state.getGlobalLinkTransform(move_group.getEndEffectorLink());

                Eigen::Vector3d pos = ee_pose.translation();
                Eigen::Quaterniond q(ee_pose.rotation());

                file << "," << pos.x() << "," << pos.y() << "," << pos.z()
                    << "," << q.x() << "," << q.y() << "," << q.z() << "," << q.w();
                file << "\n";
            }

            // 追加目标位姿
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


    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
