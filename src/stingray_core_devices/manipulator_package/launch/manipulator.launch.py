from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_package',
            executable='manipulator_sub_node',
            name='manipulator_sub_node',
            output='screen',
        ),
    ])
