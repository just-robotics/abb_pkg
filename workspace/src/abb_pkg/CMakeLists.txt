cmake_minimum_required(VERSION 3.8)
project(abb_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# ================= Dependencies =================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(abb_libegm REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(kdl_parser REQUIRED)

# ================= Include =================
include_directories(
  include
)

# ================= Executable =================
add_executable(egm_node
  src/egm_node.cpp
  src/controller.cpp
)

ament_target_dependencies(egm_node
  rclcpp
  visualization_msgs
  geometry_msgs
  abb_libegm
  trac_ik_lib
  kdl_parser
)

# ================= Install =================
install(TARGETS
  egm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

# ================= Export =================
ament_package()
