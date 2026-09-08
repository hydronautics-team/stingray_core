import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory("stingray_core_localization"),
        "config",
        "localization.yaml",
    )

    localization_node = Node(
        package="stingray_core_localization",
        executable="localization",
        name="localization",
        parameters=[config_file],
    )

    return LaunchDescription([
        localization_node,
    ])
