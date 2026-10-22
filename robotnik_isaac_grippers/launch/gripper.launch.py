"""Mount the 2FG7 description and start its independent Isaac control stack."""

from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    robot_namespace = robot_id.perform(context).strip('/')
    gripper_namespace = f'/{robot_namespace}/gripper'
    manager_path = f'{gripper_namespace}/controller_manager'
    robot_description = ParameterValue(Command([
        'xacro ', gripper_description,
        ' onrobot_macro:=', str(macro),
        ' mesh_path:=', str(meshes),
        ' backend:=isaac',
        ' isaac_joint_commands_topic:=',
        f'{gripper_namespace}/isaac/joint_command',
        ' isaac_joint_states_topic:=',
        f'/{robot_namespace}/isaac/joint_states',
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
        remappings=[
            ('joint_states', 'isaac/joint_states'),
            ('robot_description', f'{gripper_namespace}/robot_description'),
        ],
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

    controller_manager = Node(
        package='controller_manager', executable='ros2_control_node',
        name='controller_manager', namespace=gripper_namespace,
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('onrobot_2fg7'), 'config',
                'ros2_control_controllers.yaml',
            ]),
            {'use_sim_time': True},
        ],
        remappings=[('robot_description', f'{gripper_namespace}/robot_description')],
        output='screen',
    )
    controllers = [
        'joint_state_broadcaster', 'parallel_gripper_limit_broadcaster',
        'gripper_state_broadcaster', 'recovery_controller', 'gripper_controller',
    ]
    spawners = []
    for controller in controllers:
        arguments = [
            controller, '--controller-manager', manager_path,
            '--controller-manager-timeout', '120', '--switch-timeout', '120',
        ]
        if controller == 'gripper_controller':
            arguments += [
                '--controller-ros-args',
                '--ros-args -p conventional_speed_control:=false',
            ]
        spawners.append(Node(
            package='controller_manager', executable='spawner',
            namespace=gripper_namespace, arguments=arguments, output='screen',
        ))

    def continue_after_success(next_action, controller):
        def on_exit(event, _context):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(
                    reason=f'2FG7 controller {controller} failed to start'))]
            return [next_action] if next_action is not None else []
        return on_exit

    handlers = [
        RegisterEventHandler(OnProcessExit(
            target_action=spawner,
            on_exit=continue_after_success(
                spawners[index + 1] if index + 1 < len(spawners) else None,
                controllers[index],
            ),
        ))
        for index, spawner in enumerate(spawners)
    ]
    # Register handlers before starting any spawner, including fast failures.
    return [mount_frame, state_publisher, controller_manager, *handlers, spawners[0]]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('robot_model', default_value='rbkairos_plus'),
        DeclareLaunchArgument('gripper_model'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=_launch_gripper)])
