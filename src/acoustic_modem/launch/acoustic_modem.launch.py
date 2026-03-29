import os

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

ACOUSTIC_MODEM_PACKAGE = "acoustic_modem"
ACOUSTIC_MODEM_NODE_NAME = "acoustic_modem"
ACOUSTIC_MODEM_NAMESPACE = "stingray_core/acoustic_modem"

def generate_launch_description():
    package_dir = get_package_share_directory(ACOUSTIC_MODEM_PACKAGE)

    return LaunchDescription(
        [
            Node(
                package=ACOUSTIC_MODEM_PACKAGE,
                executable=ACOUSTIC_MODEM_NODE_NAME,
                name=ACOUSTIC_MODEM_NODE_NAME,
                output="screen",
                namespace=ACOUSTIC_MODEM_NAMESPACE,

            )
        ]
    )
