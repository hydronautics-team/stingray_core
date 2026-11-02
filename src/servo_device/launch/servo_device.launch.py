from launch import LaunchDescription
from launch_ros.actions import Node

SERVO_DEVICE_PACKAGE = "servo_device"
SERVO_DEVICE_NODE_NAME = "servo_device"
SERVO_DEVICE_NAMESPACE = "stingray_core/servo_device"


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package=SERVO_DEVICE_PACKAGE,
                executable=SERVO_DEVICE_NODE_NAME,
                name=SERVO_DEVICE_NODE_NAME,
                output="screen",
                namespace=SERVO_DEVICE_NAMESPACE,
                emulate_tty=True,
            )
        ]
    )
