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
namespace = 'localization'

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'param.yaml')

    publish_topic_pf = LaunchConfiguration('publish_topic_pf')
    publish_topic_kf = LaunchConfiguration('publish_topic_kf')
    subscribe_topic_odom = LaunchConfiguration('subscribe_topic_odom')

    initial_pos_x = LaunchConfiguration('initial_pos_x')
    initial_pos_y = LaunchConfiguration('initial_pos_y')
    initial_pos_yaw = LaunchConfiguration('initial_pos_yaw')

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
        'publish_topic_pf',
        default_value='pf_odom',
        description='pf predict result topic name'))
    
    ld.add_action(DeclareLaunchArgument(
        'publish_topic_kf',
        default_value='kf_odom',
        description='kf predict result topic name'))
    
    ld.add_action(DeclareLaunchArgument(
        'subscribe_topic_odom',
        default_value='odom',
        description='kf predict result topic name'))

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
        executable="lcmcl",
        namespace=namespace,
        parameters=[{
            'initial_pos.x' : initial_pos_x,
            'initial_pos.y' : initial_pos_y,
            'initial_pos.yaw' : initial_pos_yaw,
        }, config_file],
        remappings=[
                ('pf_odom', publish_topic_pf),
                ('kf_odom', publish_topic_kf),
                ('odom', subscribe_topic_odom)
            ]
    ))

    return ld