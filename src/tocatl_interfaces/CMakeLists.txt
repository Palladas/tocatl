cmake_minimum_required(VERSION 3.8)
project(tocatl_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorTarget.msg"
  "msg/MotorCommand.msg"
  "msg/MotorState.msg"
  "msg/FootState.msg"
  "msg/RobotState.msg"
  "msg/LegState.msg"
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs nav_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
