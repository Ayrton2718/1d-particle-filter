from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

package_name = 'lcmcl'

def generate_launch_description():
    ld = LaunchDescription()

    config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'param.yaml')

    ld.add_action(Node(
        package=package_name,
        executable="lcmcl",
        namespace="localization",
        parameters=[config_file]
    ))

    return ld