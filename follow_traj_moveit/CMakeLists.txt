cmake_minimum_required(VERSION 3.8)
project(follow_traj_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(ur_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Build the action server executable
add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  ur_interfaces
  geometry_msgs
  action_msgs
)

add_executable(static_tf2_broadcaster src/static_transform_publisher.cpp)
ament_target_dependencies(static_tf2_broadcaster
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(traj_preprocess src/trajectory_preprocessor.cpp)
ament_target_dependencies(traj_preprocess
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Install targets
install(TARGETS 
  action_server
  static_tf2_broadcaster
  traj_preprocess
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_export_dependencies(rclcpp)
ament_export_dependencies(ur_interfaces)

ament_package()
