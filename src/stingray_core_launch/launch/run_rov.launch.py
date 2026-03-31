from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    comm_pkg = get_package_share_directory('stingray_core_communication')
    control_pkg = get_package_share_directory('stingray_core_control')
    ms5837_pkg = get_package_share_directory('ms5837_pressure_sensor')
    vectornav_pkg = get_package_share_directory('vectornav')

    thruster_link_launch = os.path.join(comm_pkg, 'launch', 'thruster_link.launch.py')
    core_control_launch = os.path.join(control_pkg, 'launch', 'stingray_core_control.launch.py')
    # ms5837_launch = os.path.join(ms5837_pkg, 'launch', 'ms5837.launch.py')
    vectornav_launch = os.path.join(vectornav_pkg, 'launch', 'vectornav.launch.py')

    thruster_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_link_launch)
    )

    core_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_control_launch)
    )

    # ms5837 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(ms5837_launch)
    # )

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vectornav_launch)
    )

    return LaunchDescription([
        thruster_link,
        core_control,
        # ms5837,
        vectornav
    ])
