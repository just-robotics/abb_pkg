cmake_minimum_required(VERSION 3.8)
project(box_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanDeviceState.msg"
  "srv/BoxRecoverMotor.srv"
  "srv/SetHomingMode.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
