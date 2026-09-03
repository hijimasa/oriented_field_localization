"""AMCL の監視役として起動する (map -> odom は AMCL が出す)。

AMCL は大域位置推定を持たないので、初期姿勢を人が与える必要があり、いちど外れると
自力では戻れない。このノードを横に置くと、

  * 起動時の初期姿勢を自動で与え (/initialpose)、
  * 走行中も AMCL の姿勢を毎スキャン検証して、壊れていたら撒き直させる

ようになる。**AMCL 側は何も変えなくてよい** (set_initial_pose を false にして、
人が初期姿勢を与えるのをやめるだけ)。

    ros2 launch oriented_field_localization amcl_supervisor.launch.py \
        map_yaml_path:=/absolute/path/to/map.yaml

判定と鈍らせ方は config/params.yaml の supervise_* と
include/oriented_field_localization/reseed_policy.hpp を参照。

自分で map -> odom を出す単体構成は global_localization.launch.py のほう。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('oriented_field_localization')
    default_params = os.path.join(pkg, 'config', 'params.yaml')

    return LaunchDescription([
        # global_localization.launch.py と同じ理由 (OpenMP のビジーウェイト)
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
        DeclareLaunchArgument(
            'amcl_pose_topic', default_value='/amcl_pose',
            description='監視する側が出す姿勢'),
        Node(
            package='oriented_field_localization',
            executable='ofl_node',
            name='oriented_field_localization',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'map_yaml_path': LaunchConfiguration('map_yaml_path'),
                    # TF を出すのは AMCL。こちらは監視と /initialpose だけ
                    'tf_mode': 'none',
                    'supervise_amcl': True,
                    # 監視は毎スキャン要るので、GLOBAL も自動で回す
                    'auto_localize': True,
                    'publish_initialpose': True,
                },
            ],
            remappings=[
                ('scan', LaunchConfiguration('scan_topic')),
                ('map', LaunchConfiguration('map_topic')),
                ('amcl_pose', LaunchConfiguration('amcl_pose_topic')),
            ],
        ),
    ])
