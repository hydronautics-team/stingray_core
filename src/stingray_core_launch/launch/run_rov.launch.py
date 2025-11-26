from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    comm_pkg = get_package_share_directory('stingray_core_communication')
    control_pkg = get_package_share_directory('stingray_core_control')

    thruster_link_launch = os.path.join(comm_pkg, 'launch', 'thruster_link.launch.py')
    core_control_launch = os.path.join(control_pkg, 'launch', 'stingray_core_control.launch.py')

    thruster_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_link_launch)
    )

    core_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_control_launch)
    )

    return LaunchDescription([
        thruster_link,
        core_control
    ])
