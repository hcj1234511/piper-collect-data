#include "/home/hong/piper_ros/src/piper_sim/piper_gazebo/utils/printplan.hpp"
#include <iomanip>

namespace piper_utils {

static inline void printVec(const std::vector<double>& v,
                            const std::string& prefix = "", int precision = 4) {
  std::cout << prefix << "[";
  for (size_t i = 0; i < v.size(); ++i) {
    std::cout << std::fixed << std::setprecision(precision) << v[i];
    if (i + 1 < v.size()) std::cout << ", ";
  }
  std::cout << "]\n";
}

void printPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
               const moveit::core::MoveItErrorCode& success) {
  using moveit_msgs::msg::RobotTrajectory;

  std::cout << "=== MoveIt Plan ===\n";
  std::cout << "success: " << (success ? "true" : "false")
            << "  (code=" << success.val << ")\n";
  std::cout << "planning_time: " << std::fixed << std::setprecision(3)
            << plan.planning_time_ << " s\n";

  const RobotTrajectory& traj = plan.trajectory_;
  const auto& jt = traj.joint_trajectory;

  std::cout << "joint_names: [";
  for (size_t i = 0; i < jt.joint_names.size(); ++i) {
    std::cout << jt.joint_names[i] << (i + 1 < jt.joint_names.size() ? ", " : "");
  }
  std::cout << "]\n";

  std::cout << "#joint points: " << jt.points.size() << "\n";
  std::cout << "#multi_dof points: " << traj.multi_dof_joint_trajectory.points.size() << "\n";

  if (!jt.points.empty()) {
    const auto& p0 = jt.points.front();
    const auto& pN = jt.points.back();

    printVec(p0.positions, "start positions: ");
    if (!p0.velocities.empty()) printVec(p0.velocities, "start velocities: ");
    const double t0 = p0.time_from_start.sec + p0.time_from_start.nanosec * 1e-9;
    const double tN = pN.time_from_start.sec + pN.time_from_start.nanosec * 1e-9;
    std::cout << "start t: " << t0 << " s\n";
    printVec(pN.positions, "final positions: ");
    if (!pN.velocities.empty()) printVec(pN.velocities, "final velocities: ");
    std::cout << "total duration: " << tN << " s\n";
  }
  std::cout << "===================\n";
}

} // namespace piper_utils
