cmake_minimum_required(VERSION 3.8)
project(stingray_core_communication)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Добавляем исполняемый файл для узла драйвера двигателей
add_executable(thrusters_driver_node src/thrusters_driver_node.cpp)
# Указываем зависимости для узла
ament_target_dependencies(thrusters_driver_node rclcpp std_msgs)

# Устанавливаем исполняемый файл в целевую директорию
install(TARGETS
  thrusters_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()