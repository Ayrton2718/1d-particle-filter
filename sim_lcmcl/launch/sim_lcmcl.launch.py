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

    config_file = os.path.join(
        get_package_share_directory('lcmcl'), 'config', 'param.yaml')

    initial_pos_x = LaunchConfiguration('initial_pos_x')
    initial_pos_y = LaunchConfiguration('initial_pos_y')
    initial_pos_yaw = LaunchConfiguration('initial_pos_yaw')

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

    ld.add_action(Node(
        package=package_name,
        executable="sim_lcmcl",
        namespace="localization",
        parameters=[{
            'initial_pos.x' : initial_pos_x,
            'initial_pos.y' : initial_pos_y,
            'initial_pos.yaw' : initial_pos_yaw,
        }, config_file]
    ))

    ld.add_action(Node(
        package=package_name,
        executable='tf_broadcaster',
        name='tf_broadcaster',
        parameters=[{
            'odom_topic': '/waffle_1d/gazebo_position',
            'offset_topic': '/waffle_1d/true_position',
            'x_offset': 0.0,
            'y_offset': 0.0,
            'yaw_offset': 0.0
        }]
    ))

    return ld