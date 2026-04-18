import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnShutdown
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    node_name = 'serial_bridge_node'

    ns_arg = DeclareLaunchArgument(
        'ns', default_value='',
        description='Namespace for this serial bridge instance'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('serial_driver'), 'params', 'example.params.yaml'),
        description='YAML file with serial parameters'
    )

    bridge_node = LifecycleNode(
        package='serial_driver',
        executable='serial_bridge',
        name=node_name,
        namespace=LaunchConfiguration('ns'),
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        remappings=[
            ('/serial_read',  'serial_read'),
            ('/serial_write', 'serial_write'),
        ],
    )

    cfg_ev = RegisterEventHandler(
        OnProcessStart(target_action=bridge_node, on_start=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=bridge_node,
                transition_id=Transition.TRANSITION_CONFIGURE
            ))
        ])
    )
    act_ev = RegisterEventHandler(
        OnStateTransition(target_lifecycle_node=bridge_node,
                          start_state='configuring', goal_state='inactive',
                          entities=[EmitEvent(event=ChangeState(
                              lifecycle_node_matcher=bridge_node,
                              transition_id=Transition.TRANSITION_ACTIVATE
                          ))])
    )
    sd_ev = RegisterEventHandler(
        OnShutdown(on_shutdown=[EmitEvent(event=ChangeState(
            lifecycle_node_matcher=bridge_node,
            transition_id=Transition.TRANSITION_ACTIVE_SHUTDOWN
        ))])
    )

    return LaunchDescription([ns_arg, params_arg, bridge_node, cfg_ev, act_ev, sd_ev])
