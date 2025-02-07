#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    pkg_share = get_package_share_directory('lc_map')
    map_yaml_file = os.path.join(pkg_share, 'config', 'aws_small_warehouse', 'map.yaml')

    map_server_node = Node(
        package='nav2_map_server',  # 使用するパッケージ名に合わせて変更してください
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_file,
            # autostart がサポートされない場合はここには設定しなくてもよい
        }],
    )

    # ノード起動後、2秒後に configure 遷移を実行
    configure_map_server = TimerAction(
        period=0.2,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
                output='screen'
            )
        ]
    )

    # さらに2秒後に activate 遷移を実行
    activate_map_server = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        map_server_node,
        configure_map_server,
        activate_map_server,
    ])
