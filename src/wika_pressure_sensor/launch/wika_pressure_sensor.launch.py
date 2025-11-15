from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wika_pressure_sensor',
            executable='wika_pressure_sensor_node',
            name='wika_pressure_sensor_node',
            output='screen',
            namespace='stingray_core/wika',
            parameters=[
                # Установка коэффициента давления
                {'depth_coefficient': 1.5}
            ],
            emulate_tty=True,
        )
    ])