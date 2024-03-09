from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # load ros config

    return LaunchDescription([
        Node(
            package='stingray_core_communication',
            executable='hardware_bridge',
            name='hardware_bridge',
            respawn=True,
            respawn_delay=0.5,
        ),
        Node(
            package='stingray_core_communication',
            executable='udp_driver',
            name='udp_driver',
            parameters=[
                {'send_to_ip': '192.168.1.11'},
                {'send_to_port': 13053},
                {'receive_from_ip': "192.168.1.173"},
                {'receive_from_port': 13050},
            ],
            respawn=True,
            respawn_delay=0.5,
        ),
    ])
