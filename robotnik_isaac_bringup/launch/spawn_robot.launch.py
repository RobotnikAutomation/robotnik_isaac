"""Declare the stable Robotnik Isaac spawn interface.

The actual Kit runtime is introduced after USD import validation.  Keeping the
interface here makes KAIROS and KAIROS+ launch-compatible from day one.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    robot_model = LaunchConfiguration('robot_model')
    headless = LaunchConfiguration('headless')
    frame_prefix = LaunchConfiguration('frame_prefix')
    isaac_sim_path = LaunchConfiguration('isaac_sim_path')
    description_share = FindPackageShare('robotnik_isaac_description')
    sensor_share = FindPackageShare('robotnik_isaac_sensors')
    runtime = PathJoinSubstitution([
        FindPackageShare('robotnik_isaac_bringup'), 'runtime', 'isaac_robot_runtime.py',
    ])

    arguments = [
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('robot', default_value='rbkairos'),
        DeclareLaunchArgument('robot_model', default_value='rbkairos'),
        DeclareLaunchArgument('arm_type', default_value='ur10e'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('frame_prefix', default_value=[robot_id, '_']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('run_rviz', default_value='true'),
        DeclareLaunchArgument(
            'isaac_sim_path', default_value='/home/robotnik/isaacsim',
            description='Isaac Sim installation directory; override per host.',
        ),
    ]

    isaac = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([isaac_sim_path, 'python.sh']), runtime,
            '--robot-id', robot_id,
            '--robot-model', robot_model,
            '--headless', headless,
            '--x', LaunchConfiguration('x'), '--y', LaunchConfiguration('y'),
            '--z', LaunchConfiguration('z'), '--yaw', LaunchConfiguration('yaw'),
            '--frame-prefix', frame_prefix,
            '--description-share', description_share,
            '--sensors-share', sensor_share,
        ],
        output='screen',
    )
    control_description = PathJoinSubstitution([
        FindPackageShare('robotnik_isaac_ros2_control'), 'urdf',
        'rbkairos.ros2_control.urdf',
    ])
    controller_config = PathJoinSubstitution([
        FindPackageShare('robotnik_isaac_bringup'), 'config',
        'rbkairos_controller_params.yaml',
    ])
    controller_profile = ParameterFile(
        PathJoinSubstitution([
            FindPackageShare('robotnik_isaac_bringup'), 'config',
            'rbkairos_controller_profile.yaml',
        ]),
        allow_substs=True,
    )
    robot_description = ParameterValue(
        Command(['cat ', control_description]), value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace=robot_id,
        remappings=[('joint_states', 'description_joint_states_unused')],
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
        output='screen',
    )
    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        namespace=robot_id,
        parameters=[controller_config, controller_profile],
        output='screen',
    )
    controller_manager_name = ['/', robot_id, '/controller_manager']
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner', namespace=robot_id,
        arguments=[
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
            'joint_state_broadcaster',
            '--controller-manager', controller_manager_name,
        ],
        output='screen',
    )
    robotnik_base_controller = Node(
        package='controller_manager', executable='spawner', namespace=robot_id,
        arguments=[
            '--controller-manager-timeout', '60',
            '--service-call-timeout', '60',
            '--switch-timeout', '60',
            'robotnik_base_control',
            '--controller-manager', controller_manager_name,
        ],
        output='screen',
    )

    def spawn_base_after_joint_states(event, _context):
        if event.returncode == 0:
            return [robotnik_base_controller]
        return [LogInfo(
            msg='ERROR: joint_state_broadcaster failed; robotnik_base_control will not be spawned.',
        )]

    spawn_base_after_joint_states = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=spawn_base_after_joint_states,
        )
    )
    return LaunchDescription(arguments + [
        isaac, robot_state_publisher, controller_manager,
        spawn_base_after_joint_states, joint_state_broadcaster,
    ])
