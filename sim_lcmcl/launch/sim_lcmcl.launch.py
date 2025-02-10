from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'sim_lcmcl'

def generate_launch_description():
    subprocess.run(["blackbox_create"]) 

    ld = LaunchDescription()

    config_file = os.path.join(
        get_package_share_directory('lcmcl'), 'config', 'param.yaml')

    ld.add_action(Node(
        package=package_name,
        executable="sim_lcmcl",
        namespace="localization",
        parameters=[config_file]
    ))

    ld.add_action(Node(
        package=package_name,
        executable='tf_broadcaster',
        name='tf_broadcaster',
        parameters=[{'odom_topic': '/waffle_1d/true_position'}]
    ))

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('simulation_launcher'), 'launch', 'robot_state_publisher.launch.py')
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('lcmcl'), 'launch', 'lcmcl.launch.py')
            )
        )
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'debug_rviz.launch.py')
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('lc_map'), 'launch', 'map_publisher.launch.py')
            )
        )
    )

    ld.add_action(ExecuteProcess(
    cmd=[
        'xterm', '-e',
        'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
        '--ros-args', '--remap', '/cmd_vel:=/waffle_1d/cmd_vel'
    ],
    output='screen'
))

    return ld