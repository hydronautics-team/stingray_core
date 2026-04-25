cmake_minimum_required(VERSION 3.8)
project(lights_device)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(lights_control SHARED
  src/lights_device.cpp
)
target_include_directories(lights_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lights_control
  rclcpp
  std_msgs
)

add_executable(lights_device_node
  src/main.cpp
)
target_link_libraries(lights_device_node lights_control)
ament_target_dependencies(lights_device_node
  rclcpp
  std_msgs
)

install(
  TARGETS lights_control lights_device_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()