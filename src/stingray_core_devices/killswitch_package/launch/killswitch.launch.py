from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='killswitch_package',
            executable='killswitch_sub_node',
            name='killswitch_sub_node',
            output='screen',
        ),
    ])
