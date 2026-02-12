from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os


def generate_launch_description():
    # Получаем путь к пакету serial_driver
    serial_driver_dir = get_package_share_directory('serial_driver')

    # Создаем действие для включения лаунч файла serial_driver
    serial_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_driver_dir, 'launch', 'serial_driver_bridge_node.launch.py')
        )
    )

    return LaunchDescription([
        # Запускаем serial_bridge_node через его лаунч файл
        serial_bridge_launch,

        # Запускаем ваш узел thrusters_driver_node
        Node(
            package='stingray_core_communication',
            executable='thrusters_driver_node',
            name='thrusters_driver_node',
            output='screen'
        )
    ])
