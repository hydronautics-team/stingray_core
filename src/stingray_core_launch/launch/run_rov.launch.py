from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    comm_pkg = get_package_share_directory('stingray_core_communication')
    control_pkg = get_package_share_directory('stingray_core_control')
<<<<<<< HEAD
    ms5837_pkg = get_package_share_directory('ms5837_pressure_sensor')
=======
    pressure_pkg = get_package_share_directory('pressure_sensor')
>>>>>>> origin/develop
    vectornav_pkg = get_package_share_directory('vectornav')
    dvl_pkg = get_package_share_directory('dvl_a50')
    stingray_interface_bridge_pkg = get_package_share_directory('stingray_interface_bridge')

    thruster_link_launch = os.path.join(comm_pkg, 'launch', 'thruster_link.launch.py')
    core_control_launch = os.path.join(control_pkg, 'launch', 'stingray_core_control.launch.py')
<<<<<<< HEAD
    ms5837_launch = os.path.join(ms5837_pkg, 'launch', 'ms5837.launch.py')
=======
    pressure_sensor_launch = os.path.join(pressure_pkg, 'launch', 'pressure_sensor.launch.py')
>>>>>>> origin/develop
    vectornav_launch = os.path.join(vectornav_pkg, 'launch', 'vectornav.launch.py')
    dvl_launch = os.path.join(dvl_pkg, 'launch', 'dvl_a50.launch.py')
    stingray_interface_bridge_launch = os.path.join(stingray_interface_bridge_pkg, 'launch', 'stingray_interface_bridge.launch.py')

    thruster_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_link_launch)
    )

    core_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_control_launch)
    )

<<<<<<< HEAD
    ms5837 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ms5837_launch)
    )

=======
>>>>>>> origin/develop
    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(vectornav_launch)
    )

    dvl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dvl_launch)
    )

    stingray_interface_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stingray_interface_bridge_launch),
        launch_arguments={
            "input_service": TextSubstitution(text="/stingray/services/set_twist"),
            "input_stabilization_service": TextSubstitution(text="/stingray/services/set_stabilization"),
        }.items(),
    )

    return LaunchDescription([
        thruster_link,
        core_control,
<<<<<<< HEAD
        ms5837,
        vectornav
=======
        pressure_link,
        panel_link,
        pressure_sensor,
        vectornav,
        dvl,
        stingray_interface_bridge
>>>>>>> origin/develop
    ])
