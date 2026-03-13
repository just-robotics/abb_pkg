cmake_minimum_required(VERSION 3.8)
project(rrt_3d_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_library(voxel_map_3d
  src/voxel_map_3d.cpp
)

target_include_directories(voxel_map_3d PUBLIC include)

add_library(rrt_3d_planner_core
  src/rrt_3d_planner.cpp
)

target_include_directories(rrt_3d_planner_core PUBLIC include)
target_link_libraries(rrt_3d_planner_core voxel_map_3d)

add_executable(rrt_3d_planner_node
  src/rrt_3d_planner_node.cpp
)

ament_target_dependencies(rrt_3d_planner_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
)

target_link_libraries(rrt_3d_planner_node
  voxel_map_3d
  rrt_3d_planner_core
  ${PCL_LIBRARIES}
)

install(TARGETS
  voxel_map_3d
  rrt_3d_planner_core
  rrt_3d_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
