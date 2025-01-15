from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

import datetime

package_name = 'lcmcl'

def generate_launch_description():
    subprocess.run(["blackbox_create"]) 

    ld = LaunchDescription()
    bag_path = LaunchConfiguration('bag_path')

    ld.add_action(
        SetParameter(name='use_sim_time', value=True)
    )


    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robot_publisher'), 'launch', 'robot_publisher.launch.py')
            )
        )
    )
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sim_lcmcl'), 'launch', 'debug_rviz.launch.py')
            )
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package_name), 'launch', 'lcmcl.launch.py')
            )
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            'bag_path',
            default_value='/home/sen/ros_workspaces/rosbag/rosbag_0208'
        )
    )
    ld.add_action(
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_path, '--topics', '/localization/sensor_odometry', '/localization/sensor_laser', '--clock'],
            output='screen',
            prefix=['xterm -e']
        )
    )

    return ld