from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('rover_bringup')

    return LaunchDescription([

        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml')],
            output='screen'
        )
    ])
