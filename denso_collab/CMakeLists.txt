cmake_minimum_required(VERSION 3.8)
project(denso_collab)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

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
target_include_directories(align_tool PRIVATE include)

add_executable(random_collab_pose src/random_collab_pose.cpp)
ament_target_dependencies(random_collab_pose
  ament_index_cpp
  geometry_msgs
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  yaml_cpp_vendor)
target_include_directories(random_collab_pose PRIVATE include)
target_link_libraries(random_collab_pose yaml-cpp)

add_executable(move_to_collab_start src/move_to_collab_start.cpp)
ament_target_dependencies(move_to_collab_start
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  sensor_msgs)

add_executable(record_right_pose src/record_right_pose.cpp)
ament_target_dependencies(record_right_pose rclcpp sensor_msgs)

add_executable(move_right_to_holder_pose src/move_right_to_holder_pose.cpp)
ament_target_dependencies(move_right_to_holder_pose
  moveit_ros_planning_interface
  rcl_interfaces
  rclcpp
  sensor_msgs)

install(TARGETS align_tool random_collab_pose move_to_screen_standoff move_to_collab_start record_right_pose move_right_to_holder_pose DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
