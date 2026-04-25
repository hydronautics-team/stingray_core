import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

DISPLAY_DEVICE_PACKAGE = "display_device"
DISPLAY_DEVICE_NODE_NAME = "display_device"
DISPLAY_DEVICE_NAMESPACE = "stingray_core/display_device"


def generate_launch_description():
    package_dir = get_package_share_directory(DISPLAY_DEVICE_PACKAGE)
    config_file = os.path.join(package_dir, "config", "display_device.param.yaml")

    return LaunchDescription(
        [
            Node(
                package=DISPLAY_DEVICE_PACKAGE,
                executable=DISPLAY_DEVICE_NODE_NAME,
                name=DISPLAY_DEVICE_NODE_NAME,
                output="screen",
                namespace=DISPLAY_DEVICE_NAMESPACE,
                parameters=[config_file],
                emulate_tty=True,
            )
        ]
    )
