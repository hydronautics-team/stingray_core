import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    comm_pkg = get_package_share_directory("stingray_core_communication")
    control_pkg = get_package_share_directory("stingray_core_control")
    ms5837_pkg = get_package_share_directory("ms5837_pressure_sensor")
    ah127c_sensor_pkg = get_package_share_directory("parser")

    thruster_link_launch = os.path.join(comm_pkg, "launch", "thruster_link.launch.py")
    core_control_launch = os.path.join(
        control_pkg, "launch", "stingray_core_control.launch.py"
    )
    ms5837_launch = os.path.join(ms5837_pkg, "launch", "ms5837.launch.py")
    ah127c_launch = os.path.join(ah127c_sensor_pkg, "launch", "ah127c_sensor.launch.py")

    thruster_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_link_launch)
    )

    core_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(core_control_launch)
    )

    ms5837 = IncludeLaunchDescription(PythonLaunchDescriptionSource(ms5837_launch))
    ah127c = IncludeLaunchDescription(PythonLaunchDescriptionSource(ah127c_launch))

    return LaunchDescription([thruster_link, core_control, ms5837, ah127c])
