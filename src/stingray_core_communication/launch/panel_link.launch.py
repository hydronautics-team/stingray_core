import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Путь к пакету serial_driver
    serial_driver_dir = get_package_share_directory("serial_driver")

    # Путь к вашему файлу параметров
    thruster_params_file = PathJoinSubstitution(
        [
            get_package_share_directory("stingray_core_communication"),
            "params",
            "display.params.yaml",
        ]
    )

    # Включение launch-файла serial_driver с передачей аргумента params_file
    serial_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                serial_driver_dir, "launch", "serial_driver_bridge_node.launch.py"
            )
        ),
        launch_arguments={
            "params_file": thruster_params_file,
            "namespace": "/display",
        }.items(),
    )

    return LaunchDescription(
        [
            serial_bridge_launch,
            Node(
                package="stingray_core_communication",
                executable="display_driver_node",
                name="display_driver_node",
                remappings=[
                    ("serial_write", "/display/serial_write"),
                    ("serial_read", "/display/serial_read"),
                ],
                output="screen",
            ),
        ]
    )
