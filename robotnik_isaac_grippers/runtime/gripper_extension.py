"""Generic Isaac runtime adapter for Robotnik gripper compositions."""

from gripper_composition import (
    compose_2fg7, resolve_2fg7_asset, resolve_2fg7_description,
)


def validate(robot_model, extension_model, description_share):
    """Validate the selected gripper and resolve its USD before Isaac starts."""
    if robot_model != 'rbkairos_plus':
        raise ValueError(
            f'gripper_model:={extension_model} requires robot_model:=rbkairos_plus.')
    if extension_model != '2fg7':
        raise ValueError(
            f'Unsupported gripper_model:={extension_model}; currently supports only 2fg7.')
    asset = resolve_2fg7_asset(description_share)
    macro, meshes = resolve_2fg7_description(description_share)
    return {'asset': asset, 'macro': macro, 'meshes': meshes}


def compose(stage, robot_root_path, validated_asset):
    """Compose the selected Robotnik gripper into the active Isaac stage."""
    return compose_2fg7(stage, robot_root_path, validated_asset['asset'])
