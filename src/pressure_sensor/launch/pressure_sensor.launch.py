from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

PRESSURE_SENSOR_PACKAGE = 'pressure_sensor'
PRESSURE_SENSOR_NODE_NAME = 'pressure_sensor_impl'
PRESSURE_SENSOR_NAMESPACE = 'stingray_core/pressure_sensor'


def generate_launch_description():
    package_dir = get_package_share_directory(PRESSURE_SENSOR_PACKAGE)
    config_file = os.path.join(
        package_dir, 'config', 'pressure_sensor_params.yaml')

    return LaunchDescription([
        Node(
            package=PRESSURE_SENSOR_PACKAGE,
            executable=PRESSURE_SENSOR_NODE_NAME,
            name=PRESSURE_SENSOR_NODE_NAME,
            output='screen',
            namespace=PRESSURE_SENSOR_NAMESPACE,
            parameters=[config_file],
            emulate_tty=True,
        )
    ])
