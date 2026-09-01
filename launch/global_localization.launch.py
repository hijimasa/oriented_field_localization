"""oriented_field_localization の起動 (大域位置推定のみ)。

    ros2 launch oriented_field_localization global_localization.launch.py \
        map_yaml_path:=/absolute/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('oriented_field_localization')
    default_params = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'map_yaml_path', default_value='',
            description='map_server 形式の YAML。空なら /map を待つ'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        Node(
            package='oriented_field_localization',
            executable='ofl_node',
            name='oriented_field_localization',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'map_yaml_path': LaunchConfiguration('map_yaml_path')},
            ],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('map', LaunchConfiguration('map_topic')),
            ],
        ),
    ])
