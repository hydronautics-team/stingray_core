from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    packages_name = "stingray_core_sensors"
    rviz_file_name = "bar30.rviz"

    ping1d_node = Node(
        package='stingray_core_sensors',
        executable='bar30_node',
        output="screen",
    )

    base_to_range = Node(
            ## Configure the TF of the robot to the origin of the map coordinates
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.0', '0.0', '0.0', '0', '0.0', '0.0', 'base_link', 'bar30_link']
    )

    nodes = [
        ping1d_node,
        base_to_range,
    ]

    return LaunchDescription(nodes)
