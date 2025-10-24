import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():  
    share_dir = get_package_share_directory('serial_driver')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'example.params.yaml'),
        description='File path to the ROS2 parameters file to use'
    )

    return LaunchDescription([
        params_file_arg,
        Node(
            package='serial_driver',
            executable='serial_bridge',
            name='serial_bridge',
            namespace=TextSubstitution(text=''),
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
            respawn=True,
            respawn_delay=0.5,
        ),
    ])
