"""Complete physical OAK-D mission. No Gazebo, joystick, or remote compute needed."""
from datetime import datetime
import math
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    duration = float(value('mission_duration_sec'))
    size = float(value('tag_size_m'))
    if not math.isfinite(duration) or duration < 0:
        raise ValueError('mission_duration_sec must be finite and >= 0 (0 means untimed)')
    if not math.isfinite(size) or size <= 0:
        raise ValueError('tag_size_m must be a positive measured tag edge length')
    share = get_package_share_directory('asr_summer_school')
    output = str(Path(value('output_root')).expanduser() /
                 datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    config = value('mission_config')

    def include(package, filename, arguments):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory(package), 'launch', filename)),
            launch_arguments=arguments.items())

    common = [config, {'use_sim_time': False, 'output_dir': output,
                       'mission_duration_sec': duration,
                       'challenge_mode': value('challenge_mode').lower() == 'true',
                       'camera_info_topic': value('camera_info_topic')}]
    return [
        include('turtlebot3_bringup', 'robot.launch.py', {'use_sim_time': 'false'}),
        include('depthai_ros_driver', 'camera.launch.py', {
            'rs_compat': 'true', 'name': 'camera', 'namespace': 'camera',
            'parent_frame': 'base_footprint', 'use_rviz': 'false', 'rectify_rgb': 'true',
            'camera_model': value('camera_model'),
            'params_file': os.path.join(share, 'config', 'oakd_mission.yaml'),
            'rgb_camera.color_profile': value('camera_profile'),
            'enable_depth': 'false', 'enable_infra1': 'false', 'enable_infra2': 'false',
            **{name: value(name) for name in (
                'cam_pos_x', 'cam_pos_y', 'cam_pos_z', 'cam_roll', 'cam_pitch', 'cam_yaw')},
        }),
        Node(package='apriltag_ros', executable='apriltag_node', namespace='camera',
             name='apriltag', output='screen',
             parameters=[{'use_sim_time': False, 'family': '36h11', 'size': size,
                          'max_hamming': 0, 'detector.threads': 2, 'detector.decimate': 2.0,
                          'qos_profile': 'sensor_data'}],
             remappings=[('image_rect', value('image_topic')),
                         ('camera_info', value('camera_info_topic'))]),
        include('asr_summer_school', 'slam_toolbox.launch.py', {'use_sim_time': 'false'}),
        include('asr_summer_school', 'nav2.launch.py', {
            'use_sim_time': 'false', 'slam': 'False', 'use_localization': 'False'}),
        Node(package='asr_summer_school', executable='frontier_detection_node_exe',
             parameters=[{'use_sim_time': False, 'use_tf_pose': True,
                          'base_frame': 'base_link', 'active_area_radius': 1000000.0,
                          'epsilon': 0.5, 'min_points': 3, 'min_frontier_size': 5}]),
        Node(package='nav2_map_server', executable='map_saver_server', name='map_saver',
             parameters=[{'use_sim_time': False, 'save_map_timeout': 5.0}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_saver', parameters=[{
                 'use_sim_time': False, 'autostart': True, 'node_names': ['map_saver']}]),
        Node(package='asr_summer_school', executable='tag_mapper.py',
             output='screen', parameters=common),
        Node(package='asr_summer_school', executable='example_nav_to_pose.py',
             output='screen', parameters=common),
    ]


def generate_launch_description():
    share = get_package_share_directory('asr_summer_school')
    defaults = {
        'mission_duration_sec': '0.0',
        'challenge_mode': 'false',
        'mission_config': os.path.join(share, 'config', 'mission.yaml'),
        'output_root': '~/challenge_results', 'tag_size_m': '0.16',
        'camera_model': 'OAK-D', 'camera_profile': '640,480,15',
        'image_topic': '/camera/camera/color/image_rect',
        'camera_info_topic': '/camera/camera/color/camera_info',
        # Repository mounting defaults; replace with physical measurements.
        'cam_pos_x': '-0.062', 'cam_pos_y': '0.0', 'cam_pos_z': '0.245',
        'cam_roll': '0.0', 'cam_pitch': '0.0', 'cam_yaw': '0.0',
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default) for name, default in defaults.items()],
        OpaqueFunction(function=setup),
    ])
