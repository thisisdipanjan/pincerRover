from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('rover_bringup')

    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[os.path.join(pkg, 'config', 'ekf.yaml')]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            parameters=[os.path.join(pkg, 'config', 'amcl.yaml')]
        )
    ])
