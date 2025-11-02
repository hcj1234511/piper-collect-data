#include <cstdio>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {
constexpr const char* GROUP_NAME = "arm";     // ← 你的 MoveIt 规划组名
constexpr double STEP_POS = 0.02;             // 位置步长 5 cm
constexpr double STEP_DEG = 5.0;              // 姿态步长 5°
constexpr double STEP_RAD = STEP_DEG * M_PI / 180.0;

struct TermiosGuard {
  termios oldt{};
  bool ok{false};
  TermiosGuard() {
    if (tcgetattr(STDIN_FILENO, &oldt) == 0) {
      termios newt = oldt;
      newt.c_lflag &= ~(ICANON | ECHO);
      newt.c_cc[VMIN] = 1;
      newt.c_cc[VTIME] = 0;
      ok = (tcsetattr(STDIN_FILENO, TCSANOW, &newt) == 0);
    }
  }
  ~TermiosGuard() {
    if (ok) tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  }
};

inline void print_help() {
  printf(
    "\n=== MoveIt 末端键控（基座系）===\n"
    "W/S: +X / -X   (每次 5 cm)\n"
    "A/D: +Y / -Y   (每次 5 cm)\n"
    "U/u: +Roll 5°   | U(大写)= -Roll 5°\n"
    "I/i: +Pitch 5°  | I(大写)= -Pitch 5°\n"
    "O/o: +Yaw 5°    | O(大写)= -Yaw 5°\n"
    "Q: 退出\n"
    "-----------------------------\n");
}

inline void quatToRPY(const geometry_msgs::msg::Quaternion& qmsg,
                      double& r, double& p, double& y) {
  tf2::Quaternion q;
  tf2::fromMsg(qmsg, q);
  tf2::Matrix3x3(q).getRPY(r, p, y);
}

inline geometry_msgs::msg::Quaternion rpyToQuat(double r, double p, double y) {
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  return tf2::toMsg(q);
}
} // namespace

int main(int argc, char** argv) {
  // 初始化 ROS2
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop_moveit_ee");
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  // 使用多线程 executor，在独立线程中 spin
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {
    RCLCPP_INFO(rclcpp::get_logger("teleop_moveit_ee"), "Executor spinning...");
    executor.spin();
  });

  // MoveGroupInterface
  moveit::planning_interface::MoveGroupInterface move_group(node, GROUP_NAME);
  move_group.setGoalPositionTolerance(0.01);       // 1cm
  move_group.setGoalOrientationTolerance(0.05);    // ≈3°
  move_group.setNumPlanningAttempts(10);
  move_group.setPlanningTime(5.0);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setStartStateToCurrentState();

  TermiosGuard tg;
  if (!tg.ok) {
    RCLCPP_ERROR(node->get_logger(), "无法切换终端到非规范模式，键控不可用。");
    executor.cancel();
    if (spinner.joinable()) spinner.join();
    return 1;
  }

  print_help();

  // 当前末端姿态
  geometry_msgs::msg::Pose target = move_group.getCurrentPose().pose;
  auto print_pose = [&](){
    double rr, pp, yy; quatToRPY(target.orientation, rr, pp, yy);
    printf("Target Pose: pos(%.3f, %.3f, %.3f), rpy(%.3f, %.3f, %.3f) rad\n",
           target.position.x, target.position.y, target.position.z, rr, pp, yy);
    fflush(stdout);
  };
  print_pose();

  char c = 0;
  while (rclcpp::ok()) {
    if (read(STDIN_FILENO, &c, 1) != 1) continue;
    bool changed = false;

    switch (c) {
      // 平移（基座系）
      case 'w': case 'W': target.position.x += STEP_POS; changed = true; break;
      case 's': case 'S': target.position.x -= STEP_POS; changed = true; break;
      case 'a': case 'A': target.position.y += STEP_POS; changed = true; break;
      case 'd': case 'D': target.position.y -= STEP_POS; changed = true; break;
      case 'q': case 'Q': target.position.z += STEP_POS; changed = true; break;
      case 'e': case 'E': target.position.z -= STEP_POS; changed = true; break;
      // 姿态：小写为 +5°；大写为 -5°
      case 'u': case 'U': case 'j': {
        double r, p, y; quatToRPY(target.orientation, r, p, y);
        const double delta = (c=='u') ? STEP_RAD : -STEP_RAD; // 'U' 与 'j' 走负向
        target.orientation = rpyToQuat(r + delta, p, y);
        changed = true; break;
      }
      case 'i': case 'I': case 'k': {
        double r, p, y; quatToRPY(target.orientation, r, p, y);
        const double delta = (c=='i') ? STEP_RAD : -STEP_RAD; // 'I' 与 'k' 负向
        target.orientation = rpyToQuat(r, p + delta, y);
        changed = true; break;
      }
      case 'o': case 'O': case 'l': {
        double r, p, y; quatToRPY(target.orientation, r, p, y);
        const double delta = (c=='o') ? STEP_RAD : -STEP_RAD; // 'O' 与 'l' 负向
        target.orientation = rpyToQuat(r, p, y + delta);
        changed = true; break;
      }

      case 'm': case 'M':
        printf("Bye.\n");
        rclcpp::shutdown();
        executor.cancel();
        if (spinner.joinable()) spinner.join();
        return 0;

      default:
        print_help();
        break;
    }

    if (!changed) continue;

    // 计划并执行
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool ok = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok) {
      RCLCPP_WARN(node->get_logger(), "规划失败，尝试放宽容差或调整方向。");
      continue;
    }
    auto exe = move_group.execute(plan);
    if (!exe) {
      RCLCPP_WARN(node->get_logger(), "执行失败。");
      continue;
    }

    // 更新当前 target
    target = move_group.getCurrentPose().pose;
    print_pose();
  }

  executor.cancel();
  if (spinner.joinable()) spinner.join();
  rclcpp::shutdown();
  return 0;
}
