import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_prefix = get_package_share_directory('stingray_core_control')

    params_file = os.path.join(
        package_prefix, "params", "stingray_core_control_node.yaml"
    )
    params_file_thruster = os.path.join(
        package_prefix, "params", "thruster_matrix.yaml"
    )
    params_file_controllers = os.path.join(
        package_prefix, "params", "controllers.yaml"
    )
    rate_hz = LaunchConfiguration('rate_hz')
    topic_imu_linear_accel = LaunchConfiguration('topic_imu_linear_accel')
    topic_imu_angular_rate = LaunchConfiguration('topic_imu_angular_rate')
    topic_loop_flags = LaunchConfiguration('topic_loop_flags')
    topic_pressure_sensor = LaunchConfiguration('topic_pressure_sensor')
    topic_control_data = LaunchConfiguration('topic_control_data')
    # thruster_direction_matrix = LaunchConfiguration('thruster_direction_matrix')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rate_hz',
            default_value='100.0',
            description='ToControl loop frequency in Hz'
        ),

        DeclareLaunchArgument(
            'topic_imu_linear_accel',
            default_value='/vectornav/imu_accel',
            description='Topic for IMU linear acceleration (Vector3)'
        ),

        DeclareLaunchArgument(
            'topic_imu_angular_rate',
            default_value='/vectornav/imu_rate',
            description='Topic for IMU angular rate (Vector3)'
        ),

        DeclareLaunchArgument(
            'topic_loop_flags',
            default_value='/control/loop_flags',
            description='Topic for control loop flags (UInt8)'
        ),

        DeclareLaunchArgument(
            'topic_pressure_sensor',
            default_value='/sensors/pressure',
            description='Topic for pressure/depth (Float32)'
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
                    'topic_imu_linear_accel': topic_imu_linear_accel,
                    'topic_imu_angular_rate': topic_imu_angular_rate,
                    'topic_loop_flags': topic_loop_flags,
                    'topic_pressure_sensor': topic_pressure_sensor,
                    'topic_control_data': topic_control_data,
                    # 'thruster_direction_matrix': thruster_direction_matrix,
                }
            ],
        )
    ])
