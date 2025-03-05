#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart

package_name = 'sim_lcmcl'

def generate_launch_description():
    map_yaml_file = os.path.join(
        get_package_share_directory('lc_map'), 
        'config', 'aws_small_warehouse', 'map.yaml'
    )

    rosbag_file = LaunchConfiguration('rosbag_file')

    # 1. rviz2 ノードの定義
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace="localization",
        name='mcl_rviz2_debug',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'abu2024_debug.rviz')],
        parameters=[{
            'rosbag_panel_bagfile': rosbag_file
            }]
    )

    # 2. map_server ノードの定義
    map_server = Node(
        package='nav2_map_server',  # 使用するパッケージ名に合わせて変更してください
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )

    # 3. lifecycle_manager ノードの定義
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    # 4. rviz の起動完了を検知した後、一定の遅延（ここでは5秒）を置いてから残りのノードを起動する
    delayed_start = TimerAction(
        period=1.0,  # rviz 起動後 5 秒後に実行（必要に応じて調整）
        actions=[start_lifecycle_manager_cmd]
    )

    # 5. rviz ノードのプロセスが開始されたときに delayed_start を実行するイベントハンドラの登録
    start_after_rviz_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_node,
            on_start=[delayed_start]
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbag_file',
            default_value='None',
            description='Whether to execute gzclient)'),
        rviz_node,
        map_server,
        start_after_rviz_handler
    ])