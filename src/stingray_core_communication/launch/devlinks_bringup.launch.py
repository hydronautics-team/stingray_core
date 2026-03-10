from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory("stingray_core_communication")
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{share}/launch/pressure_link.launch.py")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{share}/launch/display_link.launch.py")
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(f"{share}/launch/power_link.launch.py")
            ),
        ]
    )
