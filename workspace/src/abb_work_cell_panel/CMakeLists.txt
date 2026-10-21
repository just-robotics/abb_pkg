cmake_minimum_required(VERSION 3.8)
project(abb_work_cell_panel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_library(abb_work_cell_panel SHARED
  src/work_cell_panel.cpp
  include/abb_work_cell_panel/work_cell_panel.hpp
)

target_include_directories(abb_work_cell_panel PUBLIC
  include
)

ament_target_dependencies(abb_work_cell_panel
  rclcpp
  std_msgs
  std_srvs
  rviz_common
  pluginlib
)

target_link_libraries(abb_work_cell_panel
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS abb_work_cell_panel
  DESTINATION lib
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
