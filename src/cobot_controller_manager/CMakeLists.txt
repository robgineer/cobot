cmake_minimum_required(VERSION 3.10)
project(cobot_controller_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

add_library(cobot_controller_manager SHARED
  src/cobot_controller_manager.cpp
)

target_include_directories(cobot_controller_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cobot_controller_manager
  rclcpp
  pluginlib
  moveit_core
  trajectory_msgs
  moveit_core
  rclcpp_action
  control_msgs
)

pluginlib_export_plugin_description_file(moveit_core cobot_controller_plugins.xml)


install(TARGETS cobot_controller_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES cobot_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
