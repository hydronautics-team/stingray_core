cmake_minimum_required(VERSION 3.8)

project(stingray_core_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(tf2 REQUIRED)

# ============================================================
# DVL adapter
# ============================================================

add_executable(dvl_adapter
  src/dvl_adapter.cpp
)

target_include_directories(dvl_adapter PUBLIC
  include
)

ament_target_dependencies(dvl_adapter
  rclcpp
  dvl_msgs
  nav_msgs
)

# ============================================================
# Pressure adapter
# ============================================================

add_executable(pressure_adapter
  src/pressure_adapter.cpp
)

target_include_directories(pressure_adapter PUBLIC
  include
)

ament_target_dependencies(pressure_adapter
  rclcpp
  geometry_msgs
)

# ============================================================
# VectorNav adapter
# ============================================================

add_executable(vectornav_adapter
  src/vectornav_adapter.cpp
)

target_include_directories(vectornav_adapter PUBLIC
  include
)

ament_target_dependencies(vectornav_adapter
  rclcpp
  sensor_msgs
  vectornav_msgs
  tf2
)

# ============================================================
# Install
# ============================================================

install(
  TARGETS
    dvl_adapter
    pressure_adapter
    vectornav_adapter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
