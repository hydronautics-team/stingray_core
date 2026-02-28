from launch import LaunchDescription
from launch_ros.actions import Node

PARSER_PACKAGE = "parser"
PARSER_NODE_EXECUTABLE = "imu_node"  # Changed from "parser" to "imu_node"
PARSER_NODE_NAME = "imu_node"
PARSER_NAMESPACE = "stingray_core/parser"


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package=PARSER_PACKAGE,
                executable=PARSER_NODE_EXECUTABLE,  # This should be "imu_node"
                name=PARSER_NODE_NAME,
                output="screen",
                namespace=PARSER_NAMESPACE,
                emulate_tty=True,
            )
        ]
    )
