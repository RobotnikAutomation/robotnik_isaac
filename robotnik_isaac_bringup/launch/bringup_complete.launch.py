"""KAIROS Isaac bringup entry point.

This deliberately mirrors Robotnik argument names without importing Gazebo,
navigation or MoveIt launch files.  Those integrations are a later layer.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('robot_model', default_value='rbkairos'),
        DeclareLaunchArgument(
            'gripper_model', default_value='',
            description=(
                'Optional gripper model; empty disables it, 2fg7 enables it '
                'for RB-KAIROS+.'),
        ),
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
    common_arguments = {
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
    }
    bringup_share = FindPackageShare('robotnik_isaac_bringup')
    grippers_share = FindPackageShare('robotnik_isaac_grippers')
    return LaunchDescription(arguments + [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, 'launch', 'spawn_robot.launch.py',
            ])),
            launch_arguments=common_arguments.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('gripper_model'), "' == ''",
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                grippers_share, 'launch', 'gripper.launch.py',
            ])),
            launch_arguments={
                'robot_id': LaunchConfiguration('robot_id'),
                'robot_model': LaunchConfiguration('robot_model'),
                'gripper_model': LaunchConfiguration('gripper_model'),
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('gripper_model'), "' != ''",
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, 'launch', 'spawn_robot.launch.py',
            ])),
            launch_arguments={
                **common_arguments,
                'stage_extension': PathJoinSubstitution([
                    grippers_share, 'runtime', 'gripper_extension.py',
                ]),
                'stage_extension_model': LaunchConfiguration('gripper_model'),
            }.items(),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('gripper_model'), "' != ''",
            ])),
        ),
    ])
