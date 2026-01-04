#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    params_file = LaunchConfiguration('params_file')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Full path to the ROS2 parameters file to use.',
    )

    driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        namespace='',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.2', '0', '0', '0', '1', 'base_link', 'laser'],
        output='screen',
    )

    return LaunchDescription([
        declare_params_file_cmd,
        driver_node,
        tf2_node,
    ])
