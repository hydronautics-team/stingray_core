cmake_minimum_required(VERSION 3.8)
project(stingray_core_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)
find_package(stingray_core_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)


include_directories(
        include
)

add_executable(uart_driver
        src/uart_driver.cpp
        include/uart_driver.h
        src/messages/common.cpp
        src/messages/config.cpp
        src/messages/direct.cpp
        src/messages/normal.cpp
        src/messages/welt.cpp
        include/messages/welt.h
        include/messages/common.h
        include/messages/config.h
        include/messages/direct.h
        include/messages/normal.h
        )
ament_target_dependencies(uart_driver rclcpp std_msgs std_srvs serial stingray_core_interfaces  nlohmann_json ament_index_cpp)

add_executable(hardware_bridge
        src/hardware_bridge.cpp
        include/hardware_bridge.h
        src/messages/common.cpp
        src/messages/config.cpp
        src/messages/direct.cpp
        src/messages/normal.cpp
        include/messages/common.h
        include/messages/config.h
        include/messages/direct.h
        include/messages/normal.h
        src/messages/welt.cpp
        include/messages/welt.h
        )
ament_target_dependencies(hardware_bridge rclcpp std_msgs std_srvs serial stingray_core_interfaces  nlohmann_json ament_index_cpp)

add_executable(udp_driver
        src/udp_driver.cpp
        include/udp_driver.h
        src/messages/common.cpp
        src/messages/config.cpp
        src/messages/direct.cpp
        src/messages/normal.cpp
        include/messages/common.h
        include/messages/config.h
        include/messages/direct.h
        include/messages/normal.h
        src/messages/welt.cpp
        include/messages/welt.h
        )
ament_target_dependencies(udp_driver Boost rclcpp std_msgs std_srvs serial stingray_core_interfaces  nlohmann_json ament_index_cpp)

add_library(
        messages SHARED
        src/messages/common.cpp
        src/messages/config.cpp
        src/messages/direct.cpp
        src/messages/normal.cpp
        include/messages/common.h
        include/messages/config.h
        include/messages/direct.h
        include/messages/normal.h
        src/messages/welt.cpp
        include/messages/welt.h
)

# this line to exports the library
ament_target_dependencies(messages rclcpp)

target_include_directories(messages
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(TARGETS
messages
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


install(TARGETS
        hardware_bridge
        uart_driver
        udp_driver
        DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(Boost)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
ament_package()