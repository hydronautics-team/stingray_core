from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    comm_pkg = get_package_share_directory('stingray_core_communication')
    control_pkg = get_package_share_directory('stingray_core_control')
    pressure_pkg = get_package_share_directory('pressure_sensor')
    #ms5837_pkg = get_package_share_directory('ms5837_pressure_sensor')
    vectornav_pkg = get_package_share_directory('vectornav')

    pressure_link_launch = os.path.join(comm_pkg, 'launch', 'pressure_link.launch.py')
    thruster_link_launch = os.path.join(comm_pkg, 'launch', 'thruster_link.launch.py')
    panel_link_launch = os.path.join(comm_pkg, 'launch', 'panel_link.launch.py')
    pressure_params = os.path.join(comm_pkg, 'params', 'pressure.params.yaml')
    panel_params = os.path.join(comm_pkg, 'params', 'panel.params.yaml')
    thruster_params = os.path.join(comm_pkg, 'params', 'thruster.params.yaml')
    core_control_launch = os.path.join(control_pkg, 'launch', 'stingray_core_control.launch.py')
    pressure_sensor_launch = os.path.join(pressure_pkg, 'launch', 'pressure_sensor.launch.py')
    #ms5837_launch = os.path.join(ms5837_pkg, 'launch', 'ms5837.launch.py')
    vectornav_launch = os.path.join(vectornav_pkg, 'launch', 'vectornav.launch.py')

    thruster_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(thruster_link_launch),
        launch_arguments={
            'ns': 'thruster',
            'params_file': thruster_params,
        }.items()
    )

    pressure_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pressure_link_launch),
        launch_arguments={
            'ns': 'pressure',
            'params_file': pressure_params,
        }.items()
    )
    
    panel_link = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(panel_link_launch),
        launch_arguments={
            'ns': 'panel',
            'params_file': panel_params,
        }.items()
    )

    pressure_sensor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pressure_sensor_launch)
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
        pressure_link,
        # panel_link,
        # pressure_sensor,
        # vectornav,
    ])
