cmake_minimum_required(VERSION 3.8)
project(parser)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_library(SERIAL_LIB serial REQUIRED)


include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
)

add_executable(imu_node 
  src/main.cpp 
  src/AH127Cprotocol.cpp
  include/AH127Cprotocol.h
)

ament_target_dependencies(imu_node
  rclcpp
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")


target_link_libraries(imu_node
  "${SERIAL_LIB}"
  "${cpp_typesupport_target}"
)

add_dependencies(imu_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(PROGRAMS
  scripts/emulator.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
