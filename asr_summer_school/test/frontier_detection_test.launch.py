from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            description='Absolute path to the map YAML file'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml}]
        ),

        # Configures and activates the map_server lifecycle node
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),

        Node(
            package='asr_summer_school',
            executable='frontier_detection_node_exe',
            name='frontier_detection_node',
            output='screen',
            parameters=[{
                'map_topic': 'map',
                'use_tf_pose': False,
                'epsilon': 0.5,
                'min_points': 3,
                'min_frontier_size': 5,
            }]
        ),
    ])
