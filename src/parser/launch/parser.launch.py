import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

PARSER_PACKAGE = "parser"
PARSER_NODE_NAME = "imu_node"
PARSER_NAMESPACE = "stingray_core/parser"


def generate_launch_description():
    package_dir = get_package_share_directory(PARSER_PACKAGE)

    return LaunchDescription(
        [
            Node(
                package=PARSER_PACKAGE,
                executable=PARSER_NODE_NAME,
                name=PARSER_NODE_NAME,
                output="screen",
                namespace=PARSER_NAMESPACE,
                emulate_tty=True,
            )
        ]
    )
