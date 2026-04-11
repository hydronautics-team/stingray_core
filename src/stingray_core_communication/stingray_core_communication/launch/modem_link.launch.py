from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='modem')

    thruster_params_file = PathJoinSubstitution([
        get_package_share_directory('stingray_core_communication'),
        'params',
        'modem.params.yaml'
    ])

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=thruster_params_file,
    )

    serial_lc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('stingray_core_communication'),
                'launch',
                'serial_bridge_lc.launch.py'
            )
        ),
        launch_arguments={
            'ns': LaunchConfiguration('ns'),
            'params_file': LaunchConfiguration('params_file'),
        }.items()
    )

    link_node = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        Node(
            package='stingray_core_communication',
            executable='modem_link_node',
            name='modem_link_node',
            parameters=[LaunchConfiguration('params_file')],
            output='screen'
        )
    ])

    return LaunchDescription([
        ns_arg,
        params_arg,
        serial_lc,
        link_node
    ])
