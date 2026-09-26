import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

MANIPULATOR_PACKAGE = "manipulator_device"
MANIPULATOR_NODE_NAME = "manipulator_device_node"
MANIPULATOR_NAMESPACE = "stingray_core/devices/manipulator"

    
def generate_launch_description():
    package_dir = get_package_share_directory(MANIPULATOR_PACKAGE)
    config_file = os.path.join(
        package_dir,
        "config",
        "manipulator.param.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package=MANIPULATOR_PACKAGE,
                executable=MANIPULATOR_NODE_NAME,
                name=MANIPULATOR_NODE_NAME,
                output="screen",
                namespace=MANIPULATOR_NAMESPACE,
                parameters=[config_file],
                emulate_tty=True,
            )
        ]
    )
