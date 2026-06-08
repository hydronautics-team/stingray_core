cmake_minimum_required(VERSION 3.8)
project(detect_marker)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_executable(detect_marker src/detect_marker.cpp)
target_link_libraries(detect_marker ${OpenCV_LIBS} pthread)
ament_target_dependencies(detect_marker rclcpp std_msgs)

install(TARGETS detect_marker DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()