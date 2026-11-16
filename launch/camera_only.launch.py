"""Raw camera diagnostic using the Phase 2 driver settings, without Phase 3 processing."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rm_phase3_odometry')
    driver_share = get_package_share_directory('robomaster_ros')
    driver = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(driver_share, 'launch', 'ep.launch')),
        condition=IfCondition(LaunchConfiguration('start_driver')),
        launch_arguments={
            'name': '',
            'driver_prefix': os.path.expanduser('~/rm_venv/bin/python3'),
            'model': 'ep', 'conn_type': 'ap', 'tof_0': 'true',
            'arm': 'true', 'arm_rate': '5', 'camera': 'true',
            'video_raw': '1', 'video_ffmpeg': '0',
            'camera_calibration_file': LaunchConfiguration('camera_calibration_file'),
            'with_model_description': 'true', 'chassis_timeout': '0.45',
        }.items())
    viewer = Node(
        package='image_view', executable='image_view', name='raw_camera_check',
        remappings=[('image', '/camera/image_color')], output='screen')
    return LaunchDescription([
        DeclareLaunchArgument('start_driver', default_value='true'),
        DeclareLaunchArgument('camera_calibration_file', default_value=os.path.join(
            share, 'config', 'camera_calibration.yaml')),
        driver, viewer,
    ])
