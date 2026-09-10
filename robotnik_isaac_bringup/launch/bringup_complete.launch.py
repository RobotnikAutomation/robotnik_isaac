"""KAIROS Isaac bringup entry point.

This deliberately mirrors Robotnik argument names without importing Gazebo,
navigation or MoveIt launch files.  Those integrations are a later layer.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('robot_model', default_value='rbkairos'),
        DeclareLaunchArgument('headless', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robotnik_isaac_bringup'), 'launch',
                'spawn_robot.launch.py',
            ])),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_model': LaunchConfiguration('robot_model'),
                'headless': LaunchConfiguration('headless'),
            }.items(),
        ),
    ])
