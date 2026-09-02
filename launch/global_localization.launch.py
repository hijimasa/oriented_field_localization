"""oriented_field_localization の起動 (大域位置推定のみ)。

    ros2 launch oriented_field_localization global_localization.launch.py \
        map_yaml_path:=/absolute/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('oriented_field_localization')
    default_params = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        # OpenMP は既定で並列領域の合間にビジーウェイトするため、CPU 時間が
        # 実際の計算の 1.6 倍まで膨らむ (実測 148 -> 246 CPU 秒 / 300 秒走行、
        # 遅延は 11 -> 10 ms でほとんど変わらない)。passive にすると無料で減る。
        # スレッド数は 2 が最も効率が良い (1 スレッド比で 1.38 倍速く CPU 1.31 倍。
        # 6 スレッドは 2.6 倍速いが CPU 1.57 倍)。docs/nav2_closed_loop.md を参照。
        SetEnvironmentVariable('OMP_WAIT_POLICY', 'passive'),
        DeclareLaunchArgument('omp_num_threads', default_value='2'),
        SetEnvironmentVariable(
            'OMP_NUM_THREADS', LaunchConfiguration('omp_num_threads')),
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
