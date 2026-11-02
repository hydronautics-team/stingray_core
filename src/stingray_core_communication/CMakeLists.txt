cmake_minimum_required(VERSION 3.8)
project(stingray_core_communication)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---- СБОРКА НОДЫ ----
add_executable(thrusters_driver_node src/thrusters_driver_node.cpp)
ament_target_dependencies(thrusters_driver_node rclcpp std_msgs)

# ---- УСТАНОВКА НОДЫ ----
install(
  TARGETS thrusters_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- УСТАНОВКА launch ----
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---- УСТАНОВКА params ----
install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()