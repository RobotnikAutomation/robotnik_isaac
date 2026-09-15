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
    arguments = [
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('robot_model', default_value='rbkairos'),
        DeclareLaunchArgument('arm_type', default_value='ur10e'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('frame_prefix', default_value=[LaunchConfiguration('robot_id'), '_']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('run_rviz', default_value='true'),
        DeclareLaunchArgument('world_path', default_value=PathJoinSubstitution([
            FindPackageShare('robotnik_isaac_bringup'), 'worlds', 'empty.usda',
        ])),
        DeclareLaunchArgument('isaac_sim_path', default_value='/home/robotnik/isaacsim'),
    ]
    return LaunchDescription(arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robotnik_isaac_bringup'), 'launch',
                'spawn_robot.launch.py',
            ])),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_model': LaunchConfiguration('robot_model'),
                'headless': LaunchConfiguration('headless'),
                'arm_type': LaunchConfiguration('arm_type'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'yaw': LaunchConfiguration('yaw'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'run_rviz': LaunchConfiguration('run_rviz'),
                'world_path': LaunchConfiguration('world_path'),
                'isaac_sim_path': LaunchConfiguration('isaac_sim_path'),
            }.items(),
        ),
    ])
