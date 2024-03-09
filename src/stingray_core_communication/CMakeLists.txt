cmake_minimum_required(VERSION 3.8)
project(stingray_core_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 14)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)
find_package(stingray_core_interfaces REQUIRED)

set(stingray_core_communication_SRCS
        src/uart_driver.cpp
        include/stingray_core_communication/uart_driver.h
        src/hardware_bridge.cpp
        include/stingray_core_communication/hardware_bridge.h
        src/udp_driver.cpp
        include/stingray_core_communication/udp_driver.h
        src/messages/common.cpp
        src/messages/config.cpp
        src/messages/direct.cpp
        src/messages/normal.cpp
        src/messages/welt.cpp
        include/stingray_core_communication/messages/welt.h
        include/stingray_core_communication/messages/common.h
        include/stingray_core_communication/messages/config.h
        include/stingray_core_communication/messages/direct.h
        include/stingray_core_communication/messages/normal.h
)
add_library(${PROJECT_NAME} ${stingray_core_communication_SRCS})
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs std_srvs serial stingray_core_interfaces)

include_directories(
        include
)

#add_executable(uart_driver
#        src/uart_driver.cpp
#        include/stingray_core_communication/uart_driver.h
#        src/messages/common.cpp
#        src/messages/config.cpp
#        src/messages/direct.cpp
#        src/messages/normal.cpp
#        src/messages/welt.cpp
#        include/stingray_core_communication/messages/welt.h
#        include/stingray_core_communication/messages/common.h
#        include/stingray_core_communication/messages/config.h
#        include/stingray_core_communication/messages/direct.h
#        include/stingray_core_communication/messages/normal.h
#        )
#ament_target_dependencies(uart_driver rclcpp std_msgs std_srvs serial stingray_core_interfaces)
#
#add_executable(hardware_bridge
#        src/hardware_bridge.cpp
#        include/stingray_core_communication/hardware_bridge.h
#        src/messages/common.cpp
#        src/messages/config.cpp
#        src/messages/direct.cpp
#        src/messages/normal.cpp
#        include/stingray_core_communication/messages/common.h
#        include/stingray_core_communication/messages/config.h
#        include/stingray_core_communication/messages/direct.h
#        include/stingray_core_communication/messages/normal.h
#        src/messages/welt.cpp
#        include/stingray_core_communication/messages/welt.h
#        )
#ament_target_dependencies(hardware_bridge rclcpp std_msgs std_srvs serial stingray_core_interfaces)
#
#add_executable(udp_driver
#        src/udp_driver.cpp
#        include/stingray_core_communication/udp_driver.h
#        src/messages/common.cpp
#        src/messages/config.cpp
#        src/messages/direct.cpp
#        src/messages/normal.cpp
#        include/stingray_core_communication/messages/common.h
#        include/stingray_core_communication/messages/config.h
#        include/stingray_core_communication/messages/direct.h
#        include/stingray_core_communication/messages/normal.h
#        src/messages/welt.cpp
#        include/stingray_core_communication/messages/welt.h
#        )
#ament_target_dependencies(udp_driver Boost rclcpp std_msgs std_srvs serial stingray_core_interfaces)

## Install executable
install(TARGETS
#        hardware_bridge
#        uart_driver
#        udp_driver
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
)

ament_export_include_directories(include)

ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

## Install headers
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include)

ament_export_dependencies(rosidl_default_runtime Boost)

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

ament_package()