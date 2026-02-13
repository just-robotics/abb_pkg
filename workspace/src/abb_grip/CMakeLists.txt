cmake_minimum_required(VERSION 3.8)
project(abb_grip)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost 1.66 REQUIRED COMPONENTS system)

add_executable(grip_node
  src/grip_node.cpp
  src/udp_sender.cpp
)
target_include_directories(grip_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(grip_node PUBLIC cxx_std_17)
ament_target_dependencies(grip_node rclcpp std_msgs)
target_link_libraries(grip_node Boost::system)

install(TARGETS grip_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.hpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
