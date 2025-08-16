cmake_minimum_required(VERSION 3.8)
project(gui_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(udp_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gui_bridge_node src/gui_bridge_node.cpp)
ament_target_dependencies(gui_bridge_node rclcpp std_msgs udp_msgs)

install(TARGETS gui_bridge_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
