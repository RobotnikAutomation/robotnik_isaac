"""Launch the empty Isaac PhysX world for asset and renderer validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    isaac_path = LaunchConfiguration('isaac_sim_path')
    headless = LaunchConfiguration('headless')
    runtime = PathJoinSubstitution([
        FindPackageShare('robotnik_isaac_bringup'), 'runtime', 'isaac_robot_runtime.py',
    ])
    args = [
        DeclareLaunchArgument('world', default_value='empty'),
        DeclareLaunchArgument('world_path', default_value=''),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('isaac_sim_path', default_value='/home/robotnik/isaacsim'),
    ]
    world = ExecuteProcess(
        cmd=[PathJoinSubstitution([isaac_path, 'python.sh']), runtime,
             '--robot-id', 'world', '--robot-model', 'rbkairos',
             '--description-share', FindPackageShare('robotnik_isaac_description'),
             '--sensors-share', FindPackageShare('robotnik_isaac_sensors'),
             '--headless', headless, '--world-only'],
        output='screen',
    )
    return LaunchDescription(args + [world])