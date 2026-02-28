import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    default_params_file = PathJoinSubstitution([
        FindPackageShare('power_control'),
        'config',
        'power_gpio.param.yaml'
    ])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to YAML parameters file'
        ),

        Node(
            package='power_control',
            executable='power_gpio_node',
            name='power_gpio',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            emulate_tty=True,
        ),
    ])
