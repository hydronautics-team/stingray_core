from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "ms5837_pressure_sensor"

    ping1d_node = Node(
        package='ms5837_pressure_sensor',
        executable='ms5837_node',
        output="screen",
    )

    nodes = [
        ping1d_node,
    ]

    return LaunchDescription(nodes)