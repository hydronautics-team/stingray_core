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
find_package(Qt5 REQUIRED COMPONENTS SerialPort)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_AUTOMOC ON)

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
  Qt5::SerialPort
  "${cpp_typesupport_target}"
)

install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Установка эмулятора
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
