from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_service_arg = DeclareLaunchArgument(
        "input_service", default_value="/set_twist"
    )
    input_stabilization_service_arg = DeclareLaunchArgument(
        "input_stabilization_service", default_value="/set_stabilization"
    )
    output_topic_arg = DeclareLaunchArgument(
        "output_topic", default_value="/control/data"
    )
    loop_flags_topic_arg = DeclareLaunchArgument(
        "loop_flags_topic", default_value="/control/loop_flags"
    )
    qos_depth_arg = DeclareLaunchArgument("qos_depth", default_value="1")
    qos_reliability_arg = DeclareLaunchArgument(
        "qos_reliability", default_value="reliable"
    )

    node = Node(
        package="stingray_interface_bridge",
        executable="stingray_interface_bridge",
        name="stingray_interface_bridge",
        output="screen",
        parameters=[
            {
                "input_service": LaunchConfiguration("input_service"),
                "input_stabilization_service": LaunchConfiguration("input_stabilization_service"),
                "output_topic": LaunchConfiguration("output_topic"),
                "loop_flags_topic": LaunchConfiguration("loop_flags_topic"),
                "qos_depth": LaunchConfiguration("qos_depth"),
                "qos_reliability": LaunchConfiguration("qos_reliability"),
            }
        ],
        emulate_tty=True,
    )

    return LaunchDescription(
        [
            input_service_arg,
            input_stabilization_service_arg,
            output_topic_arg,
            loop_flags_topic_arg,
            qos_depth_arg,
            qos_reliability_arg,
            node,
        ]
    )
