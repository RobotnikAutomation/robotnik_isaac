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


def configure_ros_graph(app, root_path, articulation_path, robot_id, extension_state):
    """Give the gripper its own command input on the shared articulation."""
    import omni.graph.core as og

    keys = og.Controller.Keys
    graph, _, _, _ = og.Controller.edit(
        {'graph_path': f'{root_path}/gripper_ros_graph',
         'evaluator_name': 'execution'},
        {
            keys.CREATE_NODES: [
                ('tick', 'omni.graph.action.OnPlaybackTick'),
                ('context', 'isaacsim.ros2.bridge.ROS2Context'),
                ('command', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
                ('controller', 'isaacsim.core.nodes.IsaacArticulationController'),
            ],
            keys.CONNECT: [
                ('tick.outputs:tick', 'command.inputs:execIn'),
                # Execute only after this subscriber has processed its input.
                ('command.outputs:execOut', 'controller.inputs:execIn'),
                ('context.outputs:context', 'command.inputs:context'),
                ('command.outputs:jointNames', 'controller.inputs:jointNames'),
                ('command.outputs:positionCommand', 'controller.inputs:positionCommand'),
                ('command.outputs:velocityCommand', 'controller.inputs:velocityCommand'),
                ('command.outputs:effortCommand', 'controller.inputs:effortCommand'),
            ],
            keys.SET_VALUES: [
                ('command.inputs:nodeNamespace', robot_id),
                ('command.inputs:topicName', 'gripper/isaac/joint_command'),
                ('controller.inputs:robotPath', articulation_path),
            ],
        },
    )
    og.Controller.evaluate_sync(graph)
    app.update()
