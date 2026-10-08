from launch import LaunchDescription
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

        Node(
            package='stingray_core_localization',
            executable='vectornav_adapter',
            name='vectornav_adapter',
            output='screen',
            parameters=[{
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
        ),

        Node(
            package='stingray_core_localization',
            executable='pressure_adapter',
            name='pressure_adapter',
            output='screen',
            parameters=[{
                'depth_variance': 0.01,
                'output_frame': 'odom',
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
