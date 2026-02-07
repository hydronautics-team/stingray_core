import os

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

BUTTONS_SENSOR_PACKAGE = "buttons_sensor"
BUTTONS_SENSOR_NODE_NAME = "buttons_sensor"
BUTTONS_SENSOR_NAMESPACE = "stingray_core/buttons_sensor"


def generate_launch_description():
    package_dir = get_package_share_directory(BUTTONS_SENSOR_PACKAGE)
    config_file = os.path.join(package_dir, "config", "buttons_sensor.param.yaml")

    return LaunchDescription(
        [
            Node(
                package=BUTTONS_SENSOR_PACKAGE,
                executable=BUTTONS_SENSOR_NODE_NAME,
                name=BUTTONS_SENSOR_NODE_NAME,
                output="screen",
                namespace=BUTTONS_SENSOR_NAMESPACE,
                parameters=[config_file],
                emulate_tty=True,
            )
        ]
    )
