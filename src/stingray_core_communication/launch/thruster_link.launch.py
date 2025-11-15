from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='thruster')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('stingray_core_communication'),
            'params',
            'thruster.params.yaml'
        ])
    )

    serial_lc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stingray_core_communication'),
                'launch',
                'serial_bridge_lc.launch.py'
            ])
        ),
        launch_arguments={
            'ns': LaunchConfiguration('ns'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    link_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        Node(
            package='stingray_core_communication',
            executable='thrusters_driver_node',
            name='thrusters_driver',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])

    return LaunchDescription([ns_arg, params_arg, serial_lc, link_node])