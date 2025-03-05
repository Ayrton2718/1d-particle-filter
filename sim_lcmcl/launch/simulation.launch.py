from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'sim_lcmcl'

def generate_launch_description():
    subprocess.run(["blackbox_create"])

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'initial_pos_x',
        default_value='0.0',
        description='Initial robot pose (x-axis)'))

    ld.add_action(DeclareLaunchArgument(
        'initial_pos_y',
        default_value='0.0',
        description='Initial robot pose (y-axis)'))

    ld.add_action(DeclareLaunchArgument(
        'initial_pos_yaw',
        default_value='0.0',
        description='Initial robot pose (degree)'))
    

    ld.add_action(
        SetParameter(name='use_sim_time', value=True)
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'sim_lcmcl.launch.py')
            ),
            launch_arguments={
                'initial_pos_x' : initial_pos_x,
                'initial_pos_y' : initial_pos_y,
                'initial_pos_yaw' : initial_pos_yaw
            }.items()
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('lcmcl'), 'launch', 'lcmcl.launch.py')
            ),
            launch_arguments={
                'initial_pos_x' : initial_pos_x,
                'initial_pos_y' : initial_pos_y,
                'initial_pos_yaw' : initial_pos_yaw,
                'publish_topic_pf': 'pf_odom',
                'publish_topic_kf': 'kf_odom',
                'subscribe_topic_odom': '/waffle_1d/odom',
            }.items()
        )
    )

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
                os.path.join(get_package_share_directory(package_name), 'launch', 'debug_rviz.launch.py')
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