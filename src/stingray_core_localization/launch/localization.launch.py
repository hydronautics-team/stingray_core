from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    package_share = get_package_share_directory(
        'stingray_core_localization'
    )

    ekf_config = os.path.join(
        package_share,
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('sensor_backend', default_value='vectornav'),
        DeclareLaunchArgument('imu_input_topic', default_value='/core/sensors/imu_raw'),
        DeclareLaunchArgument(
            'pressure_input_topic', default_value='/stingray_core/pressure_sensor/depth'
        ),
        DeclareLaunchArgument('pressure_input_type', default_value='point_stamped'),
        DeclareLaunchArgument('pressure_depth_scale', default_value='1.0'),
        DeclareLaunchArgument('enable_dvl', default_value='true'),

        Node(
            package='stingray_core_localization',
            executable='vectornav_adapter',
            name='vectornav_adapter',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                'output_topic': '/core/sensors/imu_raw',
            }],
            condition=LaunchConfigurationEquals('sensor_backend', 'vectornav'),
        ),

        Node(
            package='stingray_core_localization',
            executable='imu_adapter',
            name='imu_adapter',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('imu_input_topic'),
                'output_topic': '/core/sensors/imu',
                'frame_id': 'imu_link',
            }],
        ),

        Node(
            package='stingray_core_localization',
            executable='dvl_adapter',
            name='dvl_adapter',
            output='screen',
            parameters=[{
                'output_frame': 'dvl_link',
            }],
            condition=IfCondition(LaunchConfiguration('enable_dvl')),
        ),

        Node(
            package='stingray_core_localization',
            executable='pressure_adapter',
            name='pressure_adapter',
            output='screen',
            parameters=[{
                'depth_variance': 0.01,
                'output_frame': 'odom',
                'input_topic': LaunchConfiguration('pressure_input_topic'),
                'input_type': LaunchConfiguration('pressure_input_type'),
                'depth_scale': LaunchConfiguration('pressure_depth_scale'),
            }],
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
            remappings=[
                (
                    '/odometry/filtered',
                    '/core/state/odometry'
                ),
            ],
        ),
    ])
