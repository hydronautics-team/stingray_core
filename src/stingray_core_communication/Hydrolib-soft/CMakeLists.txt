cmake_minimum_required(VERSION 3.10)

project(Hydrolib VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_subdirectory(hydrolib_acs)
add_subdirectory(hydrolib_bus/hydrolib_bus_application)
add_subdirectory(hydrolib_bus/hydrolib_bus_datalink)
add_subdirectory(hydrolib_math)
add_subdirectory(hydrolib_filter)
add_subdirectory(hydrolib_concepts)
add_subdirectory(hydrolib_crc)
add_subdirectory(hydrolib_imu)
add_subdirectory(hydrolib_logger)
add_subdirectory(hydrolib_pid)
add_subdirectory(hydrolib_pressure_sensor)
add_subdirectory(hydrolib_return_codes)
add_subdirectory(hydrolib_strings)
add_subdirectory(hydrolib_thrust_generator)
add_subdirectory(hydrolib_vectornav)


enable_testing()
