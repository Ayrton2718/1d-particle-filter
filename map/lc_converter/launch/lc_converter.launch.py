from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

package_name = 'lc_converter'

def generate_launch_description():
    ld = LaunchDescription()
    
    prefix_path = get_package_prefix(package_name)
    prefix_path = os.path.normpath(prefix_path)
    split_prefix_path = prefix_path.split(os.sep)
    ws_path = os.path.join(os.sep, *split_prefix_path[0:len(split_prefix_path) - 2])
    
    node = Node(
        package=package_name,
        executable=package_name,
        namespace="localization",
        parameters=[
            os.path.join(get_package_share_directory('lc_map'), 'config/target_map.yaml'),
            {"ws_dir": ws_path}
        ]
    )

    ld.add_action(node)

    return ld