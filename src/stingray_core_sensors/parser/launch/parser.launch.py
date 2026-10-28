import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='parser',
            executable='imu_node',
            name='imu_node',
            output='screen',
            emulate_tty=True,
        )
    ])
