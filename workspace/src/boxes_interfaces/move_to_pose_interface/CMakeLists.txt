cmake_minimum_required(VERSION 3.8)
project(move_to_pose_interface)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MoveToPosePlanner.msg"
  "action/MoveByJoints.action"
  "action/MoveToNamePose.action"
  "action/MoveByTF.action"
  "action/MoveToPoint.action"
  "action/MoveRelative.action"
  "action/Execute.action"
  DEPENDENCIES action_msgs geometry_msgs moveit_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
