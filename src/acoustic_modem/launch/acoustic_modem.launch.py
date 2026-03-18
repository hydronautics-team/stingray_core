from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
                package='acoustic_modem',
                executable='acoustic_modem',
                name='acoustic_modem_node',
                output="screen",
                namespace='stingray_core/acoustic_modem',
        )
    ])