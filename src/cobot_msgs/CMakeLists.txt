cmake_minimum_required(VERSION 3.8)
project(cobot_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# generate service for cobot_control_panel communication
rosidl_generate_interfaces(${PROJECT_NAME} 
  "srv/CobotApiSrv.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
