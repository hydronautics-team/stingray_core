from launch import LaunchDescription
from launch_ros.actions import Node

LIGHTS_DEVICE_PACKAGE = "lights_device"
LIGHTS_DEVICE_NODE_NAME = "lights_device"
LIGHTS_DEVICE_NAMESPACE = "stingray_core/lights_device"


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package=LIGHTS_DEVICE_PACKAGE,
                executable=LIGHTS_DEVICE_NODE_NAME,
                name=LIGHTS_DEVICE_NODE_NAME,
                output="screen",
                namespace=LIGHTS_DEVICE_NAMESPACE,
                emulate_tty=True,
            )
        ]
    )
