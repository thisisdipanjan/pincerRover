from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    nav2_dir = get_package_share_directory('nav2_bringup')
    bringup_dir = get_package_share_directory('rover_bringup')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': os.path.join(bringup_dir, 'maps', 'map.yaml'),
                'use_sim_time': 'false',
                'params_file': os.path.join(bringup_dir, 'config', 'nav2_params.yaml')
            }.items()
        )
    ])
