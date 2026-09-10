"""Launch MoveIt for an already running RB-KAIROS+ Isaac integration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')

    moveit_config = (
        MoveItConfigsBuilder(
            'rbkairos_plus', package_name='robotnik_isaac_moveit_config'
        )
        .robot_description(file_path='config/rbkairos_plus.urdf.xacro')
        .robot_description_semantic(file_path='config/rbkairos_plus.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl', 'pilz_industrial_motion_planner'],
        )
        .pilz_cartesian_limits(file_path='config/pilz_cartesian_limits.yaml')
        .trajectory_execution(
            file_path='config/moveit_controllers.yaml',
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    common_parameters = {
        'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
    }
    move_group_parameters = {
        'allow_trajectory_execution': True,
        'monitor_dynamics': False,
    }

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        namespace=robot_id,
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            common_parameters,
            move_group_parameters,
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace=robot_id,
        output='log',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            common_parameters,
        ],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_id',
            default_value='robot',
            description='ROS namespace. It never changes URDF joint names.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('robotnik_isaac_moveit_config'),
                'config',
                'moveit.rviz',
            ]),
        ),
        move_group,
        rviz,
    ])
