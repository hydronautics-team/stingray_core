from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stingray_core_communication',                 # твой пакет
            executable='thrusters_driver_node',      # имя исполняемого файла после сборки
            name='thrusters_driver_node',
            output='screen'
        )
    ])