// utils/printplan.hpp
#pragma once
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>  // ROS2
#include <moveit_msgs/msg/robot_state.hpp>       // ROS2
#include <moveit/robot_state/robot_state.h>
#include <iostream>

namespace piper_utils {

// 声明（实现仍在 .cpp）
void printPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
               const moveit::core::MoveItErrorCode& success);

} // namespace piper_utils
