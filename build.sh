#!/bin/bash

colcon build --packages-select \
    dvl_msgs \
    stingray_core_control \
    stingray_core_communication \
    serial_driver \
    io_context \
    asio_cmake_module \
    vectornav_msgs \
    vectornav \
    stingray_core_launch \
    pressure_sensor