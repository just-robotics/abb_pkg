cmake_minimum_required(VERSION 3.8)
project(vel_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(abb_libegm REQUIRED)

include_directories(include)

add_executable(vel_control_node
  src/vel_control_node.cpp
  src/controller.cpp
)

ament_target_dependencies(vel_control_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2
  abb_libegm
)

install(TARGETS vel_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
