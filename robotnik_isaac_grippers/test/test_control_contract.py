"""Check the integration boundary using ROS launch and the external 2FG7 macro."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node


PACKAGE = Path(__file__).resolve().parents[1]


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('robot_id', ['robot', 'test_robot'])
def test_control_description_and_namespace(robot_id):
    launch = load_module('gripper_launch', PACKAGE / 'launch/gripper.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        robot_id=robot_id, robot_model='rbkairos_plus', gripper_model='2fg7')
    actions = launch._launch_gripper(context)
    nodes = [action for action in actions if isinstance(action, Node)]
    manager = next(node for node in nodes if node.node_executable == 'ros2_control_node')
    manager._perform_substitutions(context)
    assert manager.expanded_node_namespace == f'/{robot_id}/gripper'
    # All exit handlers must be installed before the first spawner executes.
    assert sum(isinstance(action, RegisterEventHandler) for action in actions) == 5
    assert isinstance(actions[-2], RegisterEventHandler)
    assert isinstance(actions[-1], Node)

    from gripper_composition import resolve_2fg7_description
    from ament_index_python.packages import get_package_share_directory
    macro, meshes = resolve_2fg7_description(
        get_package_share_directory('robotnik_isaac_description'))
    root = ET.fromstring(xacro.process_file(
        str(PACKAGE / 'urdf/onrobot_2fg7.urdf.xacro'), mappings={
            'onrobot_macro': str(macro), 'mesh_path': str(meshes),
            'backend': 'isaac',
            'isaac_joint_commands_topic': f'/{robot_id}/gripper/isaac/joint_command',
            'isaac_joint_states_topic': f'/{robot_id}/isaac/joint_states',
        }).toxml())
    control = root.find('ros2_control')
    assert control.find('hardware/plugin').text == 'onrobot_gripper_hardware/OnRobotIsaacSystem'
    parameters = {p.attrib['name']: p.text for p in control.findall('hardware/param')}
    assert parameters['isaac_joint_states_topic'] == f'/{robot_id}/isaac/joint_states'
    assert parameters['isaac_joint_commands_topic'] == f'/{robot_id}/gripper/isaac/joint_command'
    assert [j.attrib['name'] for j in control.findall('joint')] == ['grip_stroke', 'finger_stroke']
    assert control.find("joint/command_interface[@name='conventional_speed_percent']") is None
    assert root.find("joint[@name='mount_joint']/origin").attrib['xyz'] == '0 0 0'


@pytest.mark.parametrize('model, gripper', [('rbkairos', '2fg7'), ('rbkairos_plus', 'rg2')])
def test_unsupported_configuration_fails_before_nodes(model, gripper):
    launch = load_module('gripper_launch_invalid', PACKAGE / 'launch/gripper.launch.py')
    context = LaunchContext()
    context.launch_configurations.update(
        robot_id='robot', robot_model=model, gripper_model=gripper)
    with pytest.raises(RuntimeError):
        launch._launch_gripper(context)


def test_extension_hook_is_optional_and_validated(tmp_path):
    runtime = load_module('isaac_runtime', PACKAGE.parent /
                          'robotnik_isaac_bringup/runtime/isaac_robot_runtime.py')
    extension = tmp_path / 'extension.py'
    extension.write_text('def validate(*args): pass\ndef compose(*args): pass\n')
    assert runtime._load_stage_extension(extension) is not None
    extension.write_text(extension.read_text() + 'configure_ros_graph = 42\n')
    with pytest.raises(TypeError, match='configure_ros_graph'):
        runtime._load_stage_extension(extension)
    assert runtime._load_stage_extension('') is None
