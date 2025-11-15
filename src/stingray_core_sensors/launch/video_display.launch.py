from pathlib import Path

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    try:
        calibration_config_dir = Path(get_package_share_directory('stingray_core_sensors')) / "configs"
        camera_calibration_path = calibration_config_dir / "camera.yaml"
    except Exception:
        camera_calibration_path = ""

    camera_topic_arg_1 = DeclareLaunchArgument(
        "camera_topic_1", 
        default_value='/stingray_core/topics/camera_1'
    )
    camera_topic_arg_2 = DeclareLaunchArgument(
        "camera_topic_2", 
        default_value='/stingray_core/topics/camera_2'
    )
    camera_path_arg_1 = DeclareLaunchArgument(
        "camera_path_1", 
        default_value='/dev/video0'
    )
    camera_path_arg_2 = DeclareLaunchArgument(
        "camera_path_2", 
        default_value='/dev/video2'
    )



    camera_calibration_path_arg = DeclareLaunchArgument(
        "camera_calibration_path", default_value=str(camera_calibration_path)
    )
 
    
    camera_1_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='video_display_node_1',
        output='screen',
        remappings=[
            ('/image_raw', LaunchConfiguration("camera_topic_1")), 
        ],
        parameters=[
            {'video_device': LaunchConfiguration("camera_path_1")},
            {'params-file': LaunchConfiguration("camera_calibration_path")},
        ],
        respawn=True,
        respawn_delay=1,
    )

    camera_2_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='video_display_node_2',
        output='screen',
        remappings=[
            ('/image_raw', LaunchConfiguration("camera_topic_2")), 
        ],
        parameters=[
            {'video_device': LaunchConfiguration("camera_path_2")},
            {'params-file': LaunchConfiguration("camera_calibration_path")},
        ],
        respawn=True,
        respawn_delay=1,
    )


    return LaunchDescription([
        camera_topic_arg_1,
        camera_topic_arg_2,
        camera_path_arg_1,
        camera_path_arg_2,
        camera_calibration_path_arg, 
        camera_1_node,
        camera_2_node,
    ])