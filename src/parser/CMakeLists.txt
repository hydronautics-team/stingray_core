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
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Find serial library
find_library(SERIAL_LIB serial REQUIRED)

# Include directories
include_directories(include)

# Generate ROS interfaces - this creates a target named "parser"
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Data.msg"
)

# Create executable with a DIFFERENT name (not "parser")
add_executable(imu_node 
  src/main.cpp 
  src/AH127Cprotocol.cpp
  include/AH127Cprotocol.h
)

# Specify dependencies for the executable
ament_target_dependencies(imu_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Get typesupport target from the generated messages
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link libraries to the executable
target_link_libraries(imu_node
  ${SERIAL_LIB}
  ${cpp_typesupport_target}
)

# Add dependencies on generated ROS messages
add_dependencies(imu_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Install executable
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files with proper directory structure
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(PROGRAMS
  scripts/emulator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Package the project
ament_package()