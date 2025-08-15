cmake_minimum_required(VERSION 3.8)
project(cobot_rviz_plugins)

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(cobot_msgs REQUIRED)

# enable Qt meta-object compiler for cobot control panel
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/cobot_rviz_plugins/cobot_control_panel.hpp
)
# add cobot_control_panel lib
add_library(cobot_control_panel src/cobot_control_panel.cpp ${MOC_FILES})
target_include_directories(cobot_control_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(cobot_control_panel
  rclcpp
  std_srvs
  pluginlib
  rviz_common
  cobot_msgs
)
install(TARGETS cobot_control_panel
        EXPORT export_cobot_control_panel
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES ${PROJECT_NAME}.xml
        DESTINATION share/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_targets(export_cobot_control_panel)
# define plugin base type
pluginlib_export_plugin_description_file(rviz_common ${PROJECT_NAME}.xml)

ament_package()
