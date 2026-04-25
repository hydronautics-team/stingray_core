import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

BATTERY_SENSOR_PACKAGE = "battery_sensor"
BATTERY_NODE_NAME = "battery_sensor"
BATTERY_NAMESPACE = "stingray_core/battery"


def generate_launch_description():
    package_dir = get_package_share_directory(BATTERY_SENSOR_PACKAGE)
    config_file = os.path.join(package_dir, "config", "battery.param.yaml")

    return LaunchDescription(
        [
            Node(
                package=BATTERY_SENSOR_PACKAGE,
                executable=BATTERY_NODE_NAME,
                name=BATTERY_NODE_NAME,
                output="screen",
                namespace=BATTERY_NAMESPACE,
                parameters=[config_file],
                emulate_tty=True,
            )
        ]
    )
