from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('apriltag_ros')

    param_file = os.path.join(pkg_dir, 'config', 'apriltag_params.yaml')

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_ros_node',
            name='apriltag_ros',
            output='screen',
            parameters=[param_file]
        )
    ])
