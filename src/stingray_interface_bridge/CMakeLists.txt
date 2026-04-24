cmake_minimum_required(VERSION 3.8)
project(stingray_interface_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stingray_interfaces REQUIRED)
find_package(vectornav_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME}
  src/bridge_node.cpp
  src/main.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  std_msgs
  std_srvs
  stingray_interfaces
  vectornav_msgs
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
