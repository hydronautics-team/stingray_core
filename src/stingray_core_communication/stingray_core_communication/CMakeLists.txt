cmake_minimum_required(VERSION 3.8)
project(stingray_core_communication)

set(CMAKE_CXX_STANDARD 20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---- СБОРКА НОДЫ ----
add_executable(thrusters_driver_node src/thrusters_driver_node.cpp)
add_executable(pressure_link_node src/pressure_link_node.cpp)
add_executable(panel_link_node src/pressure_panel_link_node.cpp)


# Указываем зависимости для узла
ament_target_dependencies(thrusters_driver_node rclcpp std_msgs)
ament_target_dependencies(pressure_link_node rclcpp std_msgs)
ament_target_dependencies(panel_link_node rclcpp std_msgs)


# add_subdirectory(../../Hydrolib-soft Hydrolib-soft)
add_subdirectory(../Hydrolib-soft/hydrolib_bus/hydrolib_bus_application hydrolib_bus_application)
add_subdirectory(../Hydrolib-soft/hydrolib_bus/hydrolib_bus_datalink hydrolib_bus_datalink)
add_subdirectory(../Hydrolib-soft/hydrolib_logger hydrolib_logger)
add_subdirectory(../Hydrolib-soft/hydrolib_return_codes hydrolib_return_codes)
add_subdirectory(../Hydrolib-soft/hydrolib_strings hydrolib_strings)
add_subdirectory(../Hydrolib-soft/hydrolib_math hydrolib_math)
add_subdirectory(../Hydrolib-soft/hydrolib_concepts hydrolib_concepts)
add_subdirectory(../Hydrolib-soft/hydrolib_crc hydrolib_crc)
add_subdirectory(../Hydrolib-soft/deprecated/hydrolib_ring_queue HydrolibRingQueue)
add_subdirectory(../Hydrolib-soft/deprecated/hydrolib_common hydrolib_common)

# Используем plain стиль для совместимости с ament_target_dependencies
target_link_libraries(thrusters_driver_node
  HydrolibBusApplication
  HydrolibBusDatalink
  HydrolibRingQueue
  HydrolibCommon
)
target_link_libraries(pressure_link_node
  HydrolibBusApplication
  HydrolibBusDatalink
  HydrolibRingQueue
  HydrolibCommon
)
target_link_libraries(panel_link_node
  HydrolibBusApplication
  HydrolibBusDatalink
  HydrolibRingQueue
  HydrolibCommon
)

# Устанавливаем исполняемый файл в целевую директорию
install(TARGETS
  thrusters_driver_node
  pressure_link_node
  panel_link_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---- УСТАНОВКА launch ----
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ---- УСТАНОВКА params ----
install(
  DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_link_node_base
    test/test_link_node_base.cpp
  )
  target_include_directories(test_link_node_base PRIVATE
    src
  )
  ament_target_dependencies(test_link_node_base
    rclcpp
    std_msgs
  )
  target_link_libraries(test_link_node_base
    HydrolibBusApplication
    HydrolibBusDatalink
    HydrolibRingQueue
    HydrolibCommon
  )
endif()

ament_package()
