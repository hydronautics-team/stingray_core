from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_dir = get_package_share_directory(
        'stingray_core_description'
    )

    xacro_file = os.path.join(
        package_dir,
        'urdf',
        'neowelt2.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file
        ]),
        value_type=str
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description
                }
            ]
        )
    ])
