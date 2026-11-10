cmake_minimum_required(VERSION 3.8)
project(manipulator_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# JointModeCommand (jog commands, manual section 7) is missing: its fields are not
# described in the manual, see future_works.md.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointState.msg"
  "msg/RobotState.msg"
  "msg/RobotMode.msg"
  "msg/RPY.msg"
  "msg/PoseRPY.msg"
  "srv/TwistModeCK.srv"
  "srv/ComputeIkPreview.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
