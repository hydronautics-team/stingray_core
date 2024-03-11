cmake_minimum_required(VERSION 3.8)
project(stingray_core_communication)


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(serial REQUIRED)
find_package(stingray_core_interfaces REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}
        include/stingray_core_communication/uart_driver.h
        include/stingray_core_communication/hardware_bridge.h
        include/stingray_core_communication/udp_driver.h
        src/messages/common.cpp
        src/messages/config.cpp
        src/messages/direct.cpp
        src/messages/normal.cpp
        include/stingray_core_communication/messages/common.h
        include/stingray_core_communication/messages/config.h
        include/stingray_core_communication/messages/direct.h
        include/stingray_core_communication/messages/normal.h
)
target_link_libraries(${PROJECT_NAME} rclcpp Boost std_msgs std_srvs serial stingray_core_interfaces)
# this line to exports the library
## Install headers
# ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
install(
        DIRECTORY include/${PROJECT_NAME}
        DESTINATION include
)

# ament_target_dependencies(${PROJECT_NAME} rclcpp Boost std_msgs std_srvs serial stingray_core_interfaces)


add_executable(uart_driver_node
       src/uart_driver_node.cpp
       include/stingray_core_communication/uart_driver.h
       src/messages/common.cpp
       src/messages/config.cpp
       src/messages/direct.cpp
       src/messages/normal.cpp
       include/stingray_core_communication/messages/common.h
       include/stingray_core_communication/messages/config.h
       include/stingray_core_communication/messages/direct.h
       include/stingray_core_communication/messages/normal.h
       )
ament_target_dependencies(uart_driver_node rclcpp std_msgs std_srvs serial stingray_core_interfaces)

add_executable(hardware_bridge_node
       src/hardware_bridge_node.cpp
       include/stingray_core_communication/hardware_bridge.h
       src/messages/common.cpp
       src/messages/config.cpp
       src/messages/direct.cpp
       src/messages/normal.cpp
       include/stingray_core_communication/messages/common.h
       include/stingray_core_communication/messages/config.h
       include/stingray_core_communication/messages/direct.h
       include/stingray_core_communication/messages/normal.h
       )
ament_target_dependencies(hardware_bridge_node rclcpp std_msgs std_srvs serial stingray_core_interfaces)

add_executable(udp_driver_node
       src/udp_driver_node.cpp
       include/stingray_core_communication/udp_driver.h
       src/messages/common.cpp
       src/messages/config.cpp
       src/messages/direct.cpp
       src/messages/normal.cpp
       include/stingray_core_communication/messages/common.h
       include/stingray_core_communication/messages/config.h
       include/stingray_core_communication/messages/direct.h
       include/stingray_core_communication/messages/normal.h
       )
ament_target_dependencies(udp_driver_node Boost rclcpp std_msgs std_srvs serial stingray_core_interfaces)

## Install executable
install(TARGETS
        hardware_bridge_node
        uart_driver_node
        udp_driver_node
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}
        # LIBRARY DESTINATION lib
        # ARCHIVE DESTINATION lib
        # RUNTIME DESTINATION bin
        # INCLUDES DESTINATION include
)

ament_export_dependencies(rosidl_default_runtime Boost)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)


ament_package()