from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gpio_chip_arg = DeclareLaunchArgument('gpio_chip', default_value='gpiochip0')
    gpio_line_arg = DeclareLaunchArgument('gpio_line', default_value='-1')
    active_high_arg = DeclareLaunchArgument('active_high', default_value='true')
    default_on_arg = DeclareLaunchArgument('default_on', default_value='true')

    return LaunchDescription([
        gpio_chip_arg,
        gpio_line_arg,
        active_high_arg,
        default_on_arg,
        Node(
            package='power_control',
            executable='power_gpio_node',
            name='power_gpio',
            output='screen',
            parameters=[{
                'gpio_chip': LaunchConfiguration('gpio_chip'),
                'gpio_line': LaunchConfiguration('gpio_line'),
                'active_high': LaunchConfiguration('active_high'),
                'default_on': LaunchConfiguration('default_on'),
            }],
        ),
    ])
