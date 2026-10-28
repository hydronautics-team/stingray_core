import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_prefix = get_package_share_directory('stingray_core_control')

    default_params_file = os.path.join(
        package_prefix, "params", "stingray_core_control_node.yaml"
    )
    default_params_file_thruster = os.path.join(
        package_prefix, "params", "thruster_matrix.yaml"
    )
    default_params_file_controllers = os.path.join(
        package_prefix, "params", "controllers.yaml"
    )
    params_file = LaunchConfiguration('params_file')
    params_file_thruster = LaunchConfiguration('params_file_thruster')
    params_file_controllers = LaunchConfiguration('params_file_controllers')
    rate_hz = LaunchConfiguration('rate_hz')
    topic_imu = LaunchConfiguration('topic_imu')
    topic_odometry = LaunchConfiguration('topic_odometry')
    topic_loop_flags = LaunchConfiguration('topic_loop_flags')
    topic_control_data = LaunchConfiguration('topic_control_data')
    # thruster_direction_matrix = LaunchConfiguration('thruster_direction_matrix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to stingray_core_control_node params YAML'
        ),

        DeclareLaunchArgument(
            'params_file_thruster',
            default_value=default_params_file_thruster,
            description='Path to thruster_matrix YAML'
        ),

        DeclareLaunchArgument(
            'params_file_controllers',
            default_value=default_params_file_controllers,
            description='Path to controllers YAML'
        ),

        DeclareLaunchArgument(
            'rate_hz',
            default_value='100.0',
            description='ToControl loop frequency in Hz'
        ),

        DeclareLaunchArgument(
            'topic_imu',
            default_value='/core/sensors/imu',
            description='Standardized localization IMU topic (sensor_msgs/Imu)'
        ),

        DeclareLaunchArgument(
            'topic_odometry',
            default_value='/core/state/odometry',
            description='Fused localization state (nav_msgs/Odometry)'
        ),

        DeclareLaunchArgument(
            'topic_loop_flags',
            default_value='/control/loop_flags',
            description='Topic for control loop flags (UInt8)'
        ),

        DeclareLaunchArgument(
            'topic_control_data',
            default_value='/control/data',
            description='Topic for external control data (Twist)'
        ),
        # DeclareLaunchArgument(
        #     'thruster_direction_matrix',
        #     default_value='',
        #     description='Override thruster direction matrix (optional)'
        # ),  

        Node(
            package='stingray_core_control',
            executable='stingray_core_control_node',
            name='stingray_core_control_node',
            output='screen',
            parameters=[
                params_file,
                params_file_thruster,
                params_file_controllers,
                {
                    'rate_hz': rate_hz,
                    'topic_imu': topic_imu,
                    'topic_odometry': topic_odometry,
                    'topic_loop_flags': topic_loop_flags,
                    'topic_control_data': topic_control_data,
                    # 'thruster_direction_matrix': thruster_direction_matrix,
                }
            ],
        )
    ])
