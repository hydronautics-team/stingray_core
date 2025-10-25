import os
from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    share_dir = get_package_share_directory('serial_driver')

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'example.params.yaml'),
        description='File path to the ROS2 parameters file to use'
    )

    serial_bridge_node = LifecycleNode(
        package='serial_driver',
        executable='serial_bridge',
        name='serial_bridge',
        namespace=TextSubstitution(text=''),
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda event: event.node_name == 'serial_bridge',
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )
    
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=serial_bridge_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda event: event.node_name == 'serial_bridge',
                        transition_id=Transition.TRANSITION_ACTIVATE
                    )
                )
            ]
        )
    )

    return LaunchDescription([
        params_file_arg,
        serial_bridge_node,
        configure_event,
        activate_event
    ])
