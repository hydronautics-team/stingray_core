from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ns_arg = DeclareLaunchArgument('ns', default_value='thruster')
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=f"{get_package_share_directory('stingray_core_communication')}/params/thruster.params.yaml"
    )

    serial_lc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f"{get_package_share_directory('stingray_core_communication')}/launch/serial_bridge_lc.launch.py"
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
            executable='thruster_link_node',
            name='thruster_link',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],  # Используем переданный файл параметров
            # Убраны избыточные ремаппинги
        ),
    ])

    return LaunchDescription([ns_arg, params_arg, serial_lc, link_node])