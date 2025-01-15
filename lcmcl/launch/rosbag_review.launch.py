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
    

    ld.add_action(Node(
        package='sim_lcmcl',
        executable="rviz_bridge",
        namespace="localization",
        parameters=[]
    ))
    
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sim_lcmcl'), 'launch', 'rviz.launch.py')
            )
        )
    )

    ld.add_action(Node(
        package='sim_lcmcl',
        executable="lcmcl_tf_publisher",
        namespace="localization",
        parameters=[
        ]
    ))
    
    return ld