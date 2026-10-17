from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='9600'),
        DeclareLaunchArgument('rate_hz', default_value='10'),          # 0,5,10,20,25,50
        DeclareLaunchArgument('axis_map', default_value='x,y,z'),      # напр. 'y,x,-z'
        DeclareLaunchArgument('save_settings', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        Node(
            package='parser',
            executable='imu_node',
            name='imu_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
                'rate_hz': LaunchConfiguration('rate_hz'),
                'axis_map': LaunchConfiguration('axis_map'),
                'save_settings': LaunchConfiguration('save_settings'),
                'debug': LaunchConfiguration('debug'),
                'frame_id': 'imu_link',
            }],
        )
    ])
