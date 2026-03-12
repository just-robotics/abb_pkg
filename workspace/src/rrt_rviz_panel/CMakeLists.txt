cmake_minimum_required(VERSION 3.8)
project(rrt_rviz_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(rrt_start_goal_panel SHARED
  src/start_goal_panel.cpp
  include/rrt_rviz_panel/start_goal_panel.hpp
)

target_include_directories(rrt_start_goal_panel PUBLIC
  include
)

ament_target_dependencies(rrt_start_goal_panel
  rclcpp
  geometry_msgs
  rviz_common
  pluginlib
)

target_link_libraries(rrt_start_goal_panel
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS rrt_start_goal_panel
  DESTINATION lib
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
