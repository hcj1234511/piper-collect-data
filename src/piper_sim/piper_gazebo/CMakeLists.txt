cmake_minimum_required(VERSION 3.5)
project(piper_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(moveit_p src/moveit_p.cpp)
add_executable(moveit_j src/moveit_j.cpp)
add_executable(get_current_pose src/get_current_pose.cpp)
add_executable(moveit_key src/moveit_key.cpp)
add_executable(moveit_p_random src/moveit_p_random.cpp)


ament_target_dependencies(moveit_p
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(moveit_j
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
)
ament_target_dependencies(get_current_pose
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
)
ament_target_dependencies(moveit_key
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(moveit_p_random
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/joint8_ctrl.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS moveit_p 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)  
install(
  TARGETS get_current_pose 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS moveit_j 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS moveit_key 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS moveit_p_random 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
