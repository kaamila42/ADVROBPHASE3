"""Use an existing driver by default; optionally start exactly one EP driver."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def validate_params_file(context):
    """Fail before starting any driver/node if the selected parameter file is unreadable."""
    path = os.path.abspath(os.path.expanduser(LaunchConfiguration('params_file').perform(context)))
    try:
        with open(path, 'r') as stream:
            stream.read(1)
    except OSError as exc:
        raise RuntimeError(
            f'Cannot read params_file: {path}. Create it from config/phase3_params.yaml '
            'and supply measured calibration values before enabling motion.') from exc
    context.launch_configurations['params_file'] = path
    return []


def generate_launch_description():
    share = get_package_share_directory('rm_phase3_odometry')
    driver_share = get_package_share_directory('robomaster_ros')
    config = LaunchConfiguration
    declarations = [
        DeclareLaunchArgument('start_driver', default_value='false'),
        DeclareLaunchArgument('enable_motion', default_value='false'),
        DeclareLaunchArgument('imu_calibration_use_odom', default_value='false'),
        DeclareLaunchArgument('suppress_stationary_wheel_noise', default_value='false'),
        DeclareLaunchArgument('use_saved_gyro_bias', default_value='false'),
        DeclareLaunchArgument('target_width_navigation', default_value='false'),
        DeclareLaunchArgument('saved_gyro_bias_z_radps', default_value='0.0'),
        DeclareLaunchArgument('generate_movement_plots', default_value='true'),
        DeclareLaunchArgument('show_movement_plots', default_value='true'),
        DeclareLaunchArgument('video_low_latency', default_value='true'),
        DeclareLaunchArgument('video_stats_period_s', default_value='5.0'),
        DeclareLaunchArgument('view_camera', default_value='true',
                              description='Open the annotated live camera window'),
        DeclareLaunchArgument('estimation_mode', default_value='wheel_imu'),
        DeclareLaunchArgument('start_reference_verified', default_value='false'),
        DeclareLaunchArgument('start_distance_m', default_value='2.0'),
        DeclareLaunchArgument('camera_calibration_file', default_value=os.path.join(
            share, 'config', 'camera_calibration.yaml')),
        DeclareLaunchArgument('python_executable', default_value=os.path.expanduser('~/rm_venv/bin/python3')),
        DeclareLaunchArgument('driver_python', default_value=os.path.expanduser('~/rm_venv/bin/python3')),
        DeclareLaunchArgument('output_dir', default_value=os.path.expanduser('~/ros2_ws/src/rm_phase3_odometry/results')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'phase3_params.yaml')),
    ]
    driver = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(driver_share, 'launch', 'ep.launch')),
        condition=IfCondition(config('start_driver')),
        launch_arguments={
            'name': '', 'driver_prefix': config('driver_python'), 'model': 'ep',
            'conn_type': 'ap', 'tof_0': 'true', 'arm': 'true', 'arm_rate': '5',
            'camera': 'true', 'video_raw': '1', 'video_ffmpeg': '0',
            'video_low_latency': config('video_low_latency'),
            'video_stats_period_s': config('video_stats_period_s'),
            'camera_calibration_file': config('camera_calibration_file'),
            'with_model_description': 'true', 'chassis_timeout': '0.45',
            'chassis_odom_twist_in_odom': 'false',
        }.items())
    navigation = ExecuteProcess(
        cmd=[config('python_executable'), '-u', '-m', 'rm_phase3_odometry.nav_node',
             '--ros-args', '--params-file', config('params_file'),
             '-p', ['camera_calibration_file:=', config('camera_calibration_file')],
             '-p', ['enable_motion:=', config('enable_motion')],
             '-p', ['imu_calibration_use_odom:=', config('imu_calibration_use_odom')],
             '-p', ['suppress_stationary_wheel_noise:=', config('suppress_stationary_wheel_noise')],
             '-p', ['use_saved_gyro_bias:=', config('use_saved_gyro_bias')],
             '-p', ['target_width_navigation:=', config('target_width_navigation')],
             '-p', ['saved_gyro_bias_z_radps:=', config('saved_gyro_bias_z_radps')],
             '-p', ['generate_movement_plots:=', config('generate_movement_plots')],
             '-p', ['show_movement_plots:=', config('show_movement_plots')],
             '-p', ['estimation_mode:=', config('estimation_mode')],
             '-p', ['start_reference_verified:=', config('start_reference_verified')],
             '-p', ['start_distance_m:=', config('start_distance_m')],
             '-p', ['output_dir:=', config('output_dir')]], output='screen')
    camera_processing = ExecuteProcess(
        cmd=[config('python_executable'), '-u', '-m', 'rm_phase3_odometry.camera_node',
             '--ros-args', '--params-file', config('params_file')], output='screen')
    camera_view = Node(
        package='image_view', executable='image_view',
        name='annotated_camera_view',
        parameters=[{'image_transport': 'compressed'}],
        remappings=[('image', '/camera/annotated')],
        condition=IfCondition(config('view_camera')), output='screen')
    return LaunchDescription(declarations+[OpaqueFunction(function=validate_params_file), driver, camera_processing, navigation, camera_view])
