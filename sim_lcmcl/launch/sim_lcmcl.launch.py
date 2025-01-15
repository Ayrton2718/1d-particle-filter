from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix
import subprocess

package_name = 'sim_lcmcl'

def generate_launch_description():
    subprocess.run(["blackbox_create"]) 

    ld = LaunchDescription()

    ld.add_action(Node(
        package=package_name,
        executable="sim_lcmcl",
        namespace="localization",
        parameters=[]
    ))

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
                os.path.join(get_package_share_directory(package_name), 'launch', 'debug_rviz.launch.py')
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
    

    ld.add_action(Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux',
        arguments=['--ros-args', '-p', 'dev:="/dev/input/js0"']
    ))

    return ld