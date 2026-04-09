cmake_minimum_required(VERSION 3.8)
project(denso_robot_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/save_calib_tool_tf.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
