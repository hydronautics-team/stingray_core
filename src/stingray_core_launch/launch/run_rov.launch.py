from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    comm_pkg = get_package_share_directory('stingray_core_communication')
    control_pkg = get_package_share_directory('stingray_core_control')
    vectornav_pkg = get_package_share_directory('vectornav')
    stingray_interface_bridge_pkg = get_package_share_directory('stingray_interface_bridge')

    thruster_link_launch = os.path.join(comm_pkg, 'launch', 'thruster_link.launch.py')
    core_control_launch = os.path.join(control_pkg, 'launch', 'stingray_core_control.launch.py')
    vectornav_launch = os.path.join(vectornav_pkg, 'launch', 'vectornav.launch.py')
    stingray_interface_bridge_launch = os.path.join(stingray_interface_bridge_pkg, 'launch', 'stingray_interface_bridge.launch.py')

    thruster_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_link_launch)
    )

    core_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_control_launch)
    )

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vectornav_launch)
    )

    stingray_interface_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stingray_interface_bridge_launch),
        launch_arguments={
            "input_service": TextSubstitution(text="/stingray/services/set_twist"),
            "input_stabilization_service": TextSubstitution(text="/stingray/services/set_stabilization"),
        }.items(),
    )

    return LaunchDescription([
        # thruster_link,
        core_control,
        # ms5837,
        # vectornav,
        stingray_interface_bridge
    ])
