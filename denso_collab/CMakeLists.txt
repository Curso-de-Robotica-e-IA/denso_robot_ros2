cmake_minimum_required(VERSION 3.8)
project(denso_collab)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(move_to_screen_standoff src/move_to_screen_standoff.cpp)
ament_target_dependencies(move_to_screen_standoff
  geometry_msgs
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_executable(align_tool src/align_tool.cpp)
ament_target_dependencies(align_tool
  geometry_msgs
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros)

add_executable(move_to_collab_start src/move_to_collab_start.cpp)
ament_target_dependencies(move_to_collab_start
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  sensor_msgs)

install(TARGETS align_tool move_to_screen_standoff move_to_collab_start DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
