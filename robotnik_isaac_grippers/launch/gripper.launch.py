"""Publish the description and TF frames for a supported Robotnik gripper."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _launch_gripper(context):
    robot_model = LaunchConfiguration('robot_model').perform(context)
    gripper_model = LaunchConfiguration('gripper_model').perform(context)
    if robot_model != 'rbkairos_plus':
        raise RuntimeError(
            f'gripper_model:={gripper_model} can only be loaded with '
            'robot_model:=rbkairos_plus.')
    if gripper_model != '2fg7':
        raise RuntimeError(
            f'Unsupported gripper_model:={gripper_model}; this integration '
            'currently supports only 2fg7.')

    description_share = get_package_share_directory('robotnik_isaac_description')
    runtime_share = Path(get_package_share_directory('robotnik_isaac_grippers')) / 'runtime'
    if str(runtime_share) not in sys.path:
        sys.path.insert(0, str(runtime_share))
    from gripper_composition import resolve_2fg7_asset, resolve_2fg7_description

    resolve_2fg7_asset(description_share)
    macro, meshes = resolve_2fg7_description(description_share)
    gripper_description = PathJoinSubstitution([
        FindPackageShare('robotnik_isaac_grippers'), 'urdf',
        'onrobot_2fg7.urdf.xacro',
    ])
    robot_id = LaunchConfiguration('robot_id')
    robot_description = ParameterValue(Command([
        'xacro ', gripper_description,
        ' onrobot_macro:=', str(macro),
        ' mesh_path:=', str(meshes),
    ]), value_type=str)
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='gripper_state_publisher',
        namespace=robot_id,
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': ParameterValue(
                [robot_id, '_gripper_'], value_type=str),
            'use_sim_time': True,
        }],
        remappings=[('joint_states', 'isaac/joint_states')],
        output='screen',
    )
    mount_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gripper_mount_frame',
        namespace=robot_id,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'arm_tool0',
            '--child-frame-id', [robot_id, '_gripper_arm_tool0'],
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return [mount_frame, state_publisher]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('robot_model', default_value='rbkairos_plus'),
        DeclareLaunchArgument('gripper_model'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_gripper)])
