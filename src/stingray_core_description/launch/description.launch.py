from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():
    robot_description = ParameterValue(
        Command([
            'xacro ',
            LaunchConfiguration('description_file')
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'description_file',
            description='Absolute path to the platform URDF or Xacro file',
        ),
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
