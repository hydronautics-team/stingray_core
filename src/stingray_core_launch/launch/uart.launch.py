import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    # hardware bridge
    # topic names
    driver_request_topic_arg = DeclareLaunchArgument(
        "driver_request_topic", default_value='/stingray/topics/driver_request'
    )
    uv_state_topic_arg = DeclareLaunchArgument(
        "uv_state_topic", default_value='/stingray/topics/uv_state'
    )
    device_state_array_topic_arg = DeclareLaunchArgument(
        "device_state_array_topic", default_value='/stingray/topics/device_state_array'
    )
    driver_response_topic_arg = DeclareLaunchArgument(
        "driver_response_topic", default_value='/stingray/topics/driver_response'
    )

    # service names
    set_twist_srv_arg = DeclareLaunchArgument(
        "set_twist_srv", default_value='/stingray/services/set_twist'
    )
    set_stabilization_srv_arg = DeclareLaunchArgument(
        "set_stabilization_srv", default_value='/stingray/services/set_stabilization'
    )
    reset_imu_srv_arg = DeclareLaunchArgument(
        "reset_imu_srv", default_value='/stingray/services/reset_imu'
    )
    enable_thrusters_srv_arg = DeclareLaunchArgument(
        "enable_thrusters_srv", default_value='/stingray/services/enable_thrusters'
    )
    set_device_srv_arg = DeclareLaunchArgument(
        "set_device_srv", default_value='/stingray/services/set_device'
    )

    # uart driver
    device_arg = DeclareLaunchArgument(
        "device", default_value="/dev/ttyS0"
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value="115200"
    )

    return LaunchDescription([
        driver_request_topic_arg,
        uv_state_topic_arg,
        device_state_array_topic_arg,
        driver_response_topic_arg,
        set_twist_srv_arg,
        set_stabilization_srv_arg,
        reset_imu_srv_arg,
        enable_thrusters_srv_arg,
        set_device_srv_arg,
        device_arg,
        baudrate_arg,
        Node(
            package='stingray_core_communication',
            executable='hardware_bridge_node',
            name='hardware_bridge_node',
            parameters=[
                {'driver_request_topic': LaunchConfiguration("driver_request_topic")},
                {'uv_state_topic': LaunchConfiguration("uv_state_topic")},
                {'device_state_array_topic': LaunchConfiguration("device_state_array_topic")},
                {'driver_response_topic': LaunchConfiguration("driver_response_topic")},
                {'set_twist_srv': LaunchConfiguration("set_twist_srv")},
                {'set_stabilization_srv': LaunchConfiguration("set_stabilization_srv")},
                {'reset_imu_srv': LaunchConfiguration("reset_imu_srv")},
                {'enable_thrusters_srv': LaunchConfiguration("enable_thrusters_srv")},
                {'set_device_srv': LaunchConfiguration("set_device_srv")},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
        Node(
            package='stingray_core_communication',
            executable='uart_driver_node',
            name='uart_driver_node',
            parameters=[
                {'driver_request_topic': LaunchConfiguration("driver_request_topic")},
                {'driver_response_topic': LaunchConfiguration("driver_response_topic")},
                {'device': LaunchConfiguration("device")},
                {'baudrate': LaunchConfiguration("baudrate")},
                {'data_bits': 8},
                {'stop_bits': 1},
                {'parity': "none"},
                {'serial_timeout': 1000},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
    ])
