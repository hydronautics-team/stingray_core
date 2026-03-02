import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

POWER_CONTROL_PACKAGE = "power_control"
POWER_CONTROL_NODE_NAME = "power_gpio_node"
POWER_CONTROL_NAMESPACE = "stingray_core/power_control"


def generate_launch_description():
    package_dir = get_package_share_directory(POWER_CONTROL_PACKAGE)
    config_file = os.path.join(package_dir, "config", "power_gpio.param.yaml")

    return LaunchDescription(
        [
            Node(
                package=POWER_CONTROL_PACKAGE,
                executable=POWER_CONTROL_NODE_NAME,
                name=POWER_CONTROL_NODE_NAME,
                output="screen",
                namespace=POWER_CONTROL_NAMESPACE,
                parameters=[config_file],
                emulate_tty=True,
            )
        ]
    )
