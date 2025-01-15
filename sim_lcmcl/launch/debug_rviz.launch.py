from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

package_name = 'sim_lcmcl'

def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        namespace="localization",
        name='mcl_rviz2_debug',
        arguments=['-d', os.path.join(get_package_share_directory(package_name), 'rviz', 'abu2024_debug.rviz')]
    ))

    return ld