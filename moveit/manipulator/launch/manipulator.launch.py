from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    desc_pkg = get_package_share_directory('rover_description')
    moveit_pkg = get_package_share_directory('rover_moveit')

    urdf = os.path.join(desc_pkg, 'urdf', 'rover.urdf.xacro')

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', urdf])
            }]
        ),

        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                os.path.join(moveit_pkg, 'config', 'ompl_planning.yaml'),
                os.path.join(moveit_pkg, 'config', 'kinematics.yaml'),
                os.path.join(moveit_pkg, 'config', 'joint_limits.yaml'),
                os.path.join(moveit_pkg, 'config', 'planning_scene_monitor.yaml'),
                os.path.join(moveit_pkg, 'config', 'moveit_controllers.yaml')
            ]
        )
    ])
