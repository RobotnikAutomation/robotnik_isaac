"""Validate the installed model registry against each ros2_control adapter."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


CONFIG = Path(__file__).parents[1] / 'config'


@pytest.mark.parametrize('model_name', ('rbkairos', 'rbkairos_plus'))
def test_model_registry_matches_control_urdf(model_name):
    with (CONFIG / f'{model_name}.yaml').open(encoding='utf-8') as stream:
        model = yaml.safe_load(stream)
    root = ET.parse(CONFIG / f'{model_name}.ros2_control.urdf').getroot()
    control = root.find('ros2_control')
    assert control is not None

    joints = control.findall('joint')
    names = [joint.attrib['name'] for joint in joints]
    assert len(names) == len(set(names))
    assert model['robot_model'] == model_name
    assert set(names) == (
        set(model['velocity_drive_joints']) | set(model['position_drive_targets'])
    )

    by_command = {
        interface: {
            joint.attrib['name']
            for joint in joints
            if joint.find(f"command_interface[@name='{interface}']") is not None
        }
        for interface in ('position', 'velocity', 'effort')
    }
    assert by_command['velocity'] == set(model['velocity_drive_joints'])
    assert by_command['position'] == set(model['position_drive_targets'])
    assert by_command['effort'] == set()

    parameters = {
        parameter.attrib['name']: (parameter.text or '').strip()
        for parameter in control.findall('./hardware/param')
    }
    assert parameters['state_topic'] == model['state_topic']
    assert parameters['command_topic'] == model['command_topic']
    assert (CONFIG.parent / model['usd']).is_file()
