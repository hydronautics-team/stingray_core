cmake_minimum_required(VERSION 3.8)
project(lights_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Brightness.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

# Здесь не было ручных команд install(), и это было проблемой.
# В чистой среде ROS 2 это иногда работало "по умолчанию".

ament_package()
