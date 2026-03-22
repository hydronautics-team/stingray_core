from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    # Путь к пакету serial_driver
    serial_driver_dir = get_package_share_directory('serial_driver')
    
    # Путь к вашему файлу параметров
    thruster_params_file = PathJoinSubstitution([
        get_package_share_directory('stingray_core_communication'),
        'params',
        'thruster.params.yaml'
    ])

    # Включение launch-файла serial_driver с передачей аргумента params_file
    serial_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_driver_dir, 'launch', 'serial_driver_bridge_node.launch.py')
        ),
        launch_arguments={
            'params_file': thruster_params_file,
            
        }.items()
    )

    return LaunchDescription([
        serial_bridge_launch,
        Node(
            package='stingray_core_communication',
            executable='thrusters_driver_node',
            name='thrusters_driver_node',
            
            output='screen'
        )
    ])