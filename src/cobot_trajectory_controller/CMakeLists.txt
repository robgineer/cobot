cmake_minimum_required(VERSION 3.8)
project(cobot_trajectory_controller)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cobot_hardware REQUIRED)
find_package(cobot_msgs REQUIRED)

include_directories(
  include
)
add_library(${PROJECT_NAME} SHARED
  src/cobot_trajectory_controller.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  pluginlib
  realtime_tools
  trajectory_msgs
  cobot_hardware
  cobot_msgs
)
pluginlib_export_plugin_description_file(controller_interface cobot_trajectory_controller_plugins.xml)


install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES cobot_trajectory_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
