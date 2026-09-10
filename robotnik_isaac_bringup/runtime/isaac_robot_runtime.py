"""Isaac-native RB-KAIROS runtime.

The runtime only loads imported USD articulations from Robotnik Isaac packages.
It creates the PhysX scene and ROS graph at runtime so every robot instance
receives an independent namespace and pose.
"""

import argparse
import math
from pathlib import Path
import traceback


WHEEL_JOINTS = (
    'robot_front_left_wheel_joint',
    'robot_front_right_wheel_joint',
    'robot_back_left_wheel_joint',
    'robot_back_right_wheel_joint',
)

ARM_JOINTS = (
    'arm_shoulder_pan_joint',
    'arm_shoulder_lift_joint',
    'arm_elbow_joint',
    'arm_wrist_1_joint',
    'arm_wrist_2_joint',
    'arm_wrist_3_joint',
)

# Folded ``home`` pose shared with the isolated MoveIt SRDF.  ROS represents
# revolute joints in radians; USD/PhysX angular drive attributes use degrees.
ARM_HOME_POSITIONS_RAD = {
    'arm_shoulder_pan_joint': 0.0,
    'arm_shoulder_lift_joint': -2.4125,
    'arm_elbow_joint': 2.4125,
    'arm_wrist_1_joint': -1.57,
    'arm_wrist_2_joint': -1.57,
    'arm_wrist_3_joint': 0.0,
}

VISUAL_FIXED_LINKS = (
    ('robot_top_cover', 'robot_top_cover_joint'),
    ('robot_base_logo_front', 'robot_base_logo_front_joint'),
    ('robot_base_logo_left', 'robot_base_logo_left_joint'),
    ('robot_base_logo_rear', 'robot_base_logo_rear_joint'),
    ('robot_base_logo_right', 'robot_base_logo_right_joint'),
)


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot-id', required=True)
    parser.add_argument('--robot-model', choices=('rbkairos', 'rbkairos_plus'), required=True)
    parser.add_argument('--headless', choices=('true', 'false'), default='false')
    parser.add_argument('--description-share', required=True)
    parser.add_argument('--sensors-share', required=True)
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    # ``base_footprint`` is the wheel contact plane, therefore z=0 places
    # the robot on the ground created by this runtime.
    parser.add_argument('--z', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--frame-prefix', default='robot_')
    parser.add_argument('--world-only', action='store_true')
    return parser.parse_args()


def _create_world(stage):
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    ground = UsdGeom.Cube.Define(stage, '/World/ground')
    ground.CreateSizeAttr(1.0)
    # One matrix op avoids the incompatible scale/translate xform stack that
    # Isaac otherwise warns about every simulation tick.
    ground_transform = Gf.Matrix4d(1.0)
    ground_transform.SetScale(Gf.Vec3d(30.0, 30.0, 0.05))
    # The cube is 5 cm thick after scaling, hence its centre must be at
    # -3 cm for its upper face to sit 5 mm below base_footprint (z = 0).
    # Starting a rigid wheel exactly on a contact plane can make PhysX
    # alternate contact corrections (jitter).
    ground_transform.SetTranslateOnly(Gf.Vec3d(0.0, 0.0, -0.030))
    ground.AddTransformOp().Set(ground_transform)
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    material = UsdShade.Material.Define(stage, '/World/materials/ground_white')
    shader = UsdShade.Shader.Define(stage, '/World/materials/ground_white/preview')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.55, 0.58, 0.62))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(0.75)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    # Match the wheel-contact reference used by the Gazebo RB-KAIROS model.
    # This is a PhysX material layered on the visual ground material, so it
    # does not alter the source robot USD.
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr().Set(1.0)
    physics_material.CreateDynamicFrictionAttr().Set(0.9)
    UsdShade.MaterialBindingAPI.Apply(ground.GetPrim()).Bind(material)

    dome = UsdLux.DomeLight.Define(stage, '/World/lights/environment')
    dome.CreateIntensityAttr(900.0)
    dome.CreateColorAttr(Gf.Vec3f(0.20, 0.26, 0.36))
    dome.CreateTextureFormatAttr('latlong')
    key = UsdLux.DistantLight.Define(stage, '/World/lights/key')
    key.CreateIntensityAttr(2500.0)
    key.CreateColorAttr(Gf.Vec3f(1.0, 0.98, 0.95))
    key_xform = UsdGeom.Xformable(key.GetPrim())
    key_xform.AddRotateYOp().Set(35.0)
    key_xform.AddRotateXOp().Set(-45.0)


def _create_robot(stage, args, asset):
    from pxr import Gf, UsdGeom

    root_path = f'/World/{args.robot_id}'
    root = stage.DefinePrim(root_path, 'Xform')
    if not root.GetReferences().AddReference(str(asset)):
        raise RuntimeError(f'Could not reference robot USD: {asset}')
    xform = UsdGeom.Xformable(root)
    xform.AddTranslateOp().Set(Gf.Vec3d(args.x, args.y, args.z))
    xform.AddRotateZOp().Set(args.yaw)
    # The imported USD already contains the single ArticulationRoot, rigid
    # bodies, collision meshes, four wheel joints and (for KAIROS+) UR10e
    # joints.  Creating proxy bodies here would yield duplicate physics and
    # visually detached meshes.
    if not stage.GetPrimAtPath(f'{root_path}/Physics').IsValid():
        raise RuntimeError(f'Imported robot physics is missing from USD: {asset}')
    return root_path


def _find_articulation_root(stage, root_path):
    """Return the unique descendant carrying PhysX ArticulationRootAPI."""
    from pxr import Usd, UsdPhysics

    robot_prim = stage.GetPrimAtPath(root_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f'Robot prim is missing from the stage: {root_path}')

    articulation_roots = [
        prim for prim in Usd.PrimRange(robot_prim)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_roots) != 1:
        found_paths = ', '.join(str(prim.GetPath()) for prim in articulation_roots) or 'none'
        raise RuntimeError(
            f'Expected exactly one ArticulationRootAPI below {root_path}; found: {found_paths}')
    return str(articulation_roots[0].GetPath())


def _configure_wheel_velocity_drives(stage, root_path):
    """Make the imported wheel drives respond to velocity targets.

    The converted USD declares each wheel's angular drive and its maximum
    force, but omits the damping gain.  A velocity target with zero damping
    generates no torque in PhysX, so Isaac receives the ROS command while all
    wheel velocities remain zero.  Author the missing velocity-drive gains on
    the composed in-memory stage; the source USD remains untouched.
    """
    from pxr import UsdPhysics

    for joint_name in WHEEL_JOINTS:
        joint_path = f'{root_path}/Physics/{joint_name}'
        joint = stage.GetPrimAtPath(joint_path)
        if not joint.IsValid():
            raise RuntimeError(f'Wheel joint is missing from the stage: {joint_path}')

        drive = UsdPhysics.DriveAPI.Apply(joint, 'angular')
        drive.CreateStiffnessAttr().Set(0.0)
        # Damping is the gain of a PhysX velocity drive.  This value is well
        # below the existing 100000 N·m max force and avoids a zero-torque
        # drive while retaining a sufficiently stiff wheel-speed response.
        drive.CreateDampingAttr().Set(10000.0)


def _configure_arm_position_drives(stage, root_path):
    """Start and hold the UR10e in its folded home pose.

    The URDF importer creates angular drives and effort limits for the arm but
    leaves their stiffness and damping at zero.  Such drives exert no torque,
    so gravity makes the arm collapse as soon as playback begins.  Give every
    arm joint a damped position servo targeting the configured home pose.
    Author that pose as the initial joint state too, preventing a fast motion
    from the imported zero pose at the first physics frame.  A later ROS
    position command can update these same targets; MoveIt
    is not required merely to hold the robot upright.
    """
    from pxr import Sdf, UsdPhysics

    for joint_name in ARM_JOINTS:
        joint_path = f'{root_path}/Physics/{joint_name}'
        joint = stage.GetPrimAtPath(joint_path)
        if not joint.IsValid():
            raise RuntimeError(f'Arm joint is missing from the stage: {joint_path}')

        drive = UsdPhysics.DriveAPI.Apply(joint, 'angular')
        drive.CreateTypeAttr().Set(UsdPhysics.Tokens.force)
        drive.CreateStiffnessAttr().Set(10000.0)
        drive.CreateDampingAttr().Set(1000.0)
        home_degrees = math.degrees(ARM_HOME_POSITIONS_RAD[joint_name])
        drive.CreateTargetPositionAttr().Set(home_degrees)
        drive.CreateTargetVelocityAttr().Set(0.0)
        joint.CreateAttribute(
            'state:angular:physics:position', Sdf.ValueTypeNames.Float,
        ).Set(home_degrees)
        joint.CreateAttribute(
            'state:angular:physics:velocity', Sdf.ValueTypeNames.Float,
        ).Set(0.0)


def _release_imported_root_joint(stage, root_path):
    """Disable the URDF-converter joint that otherwise anchors the robot.

    ``root_joint`` connects the non-rigid USD container Xform to
    ``base_footprint``.  PhysX resolves such a fixed joint against the world,
    which lets the wheels spin but prevents any chassis translation.  The
    container is solely used to place the robot instance, so the joint must
    not take part in the simulated articulation.
    """
    from pxr import UsdPhysics

    joint_path = f'{root_path}/Physics/root_joint'
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not joint_prim.IsValid():
        raise RuntimeError(f'Imported root joint is missing from the stage: {joint_path}')
    UsdPhysics.Joint(joint_prim).CreateJointEnabledAttr().Set(False)
    joint_prim.SetActive(False)


def _attach_arm_to_physical_base(stage, root_path):
    """Keep the UR10e in the base articulation when its visual mount is stripped.

    The importer represents ``robot_top_cover`` as a rigid body even though the
    URDF gives it no inertia.  The generic sanitizer removes that invalid rigid
    body.  On RB-KAIROS+, however, the generated fixed joint for the UR10e root
    targets the massless ``arm_base_link`` below the cover.  Once the cover is
    stripped, that target no longer provides a physical connection to the
    chassis and PhysX rejects the articulation when playback starts.

    Retarget the generated arm-root fixed joint to ``robot_base_link`` and move
    its body-0 anchor to the arm mount pose.  This is authored only in the
    composed in-memory stage; the source USD remains unchanged.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    geometry_path = f'{root_path}/Geometry/robot_base_footprint'
    base_link = stage.GetPrimAtPath(f'{geometry_path}/robot_base_link')
    arm_mount = stage.GetPrimAtPath(
        f'{geometry_path}/robot_base_link/robot_top_cover/arm_base_link')
    arm_body_path = (
        f'{geometry_path}/robot_base_link/robot_top_cover/'
        'arm_base_link/arm_base_link_inertia')
    arm_body = stage.GetPrimAtPath(arm_body_path)
    if not base_link.IsValid() or not arm_mount.IsValid() or not arm_body.IsValid():
        raise RuntimeError('Imported RB-KAIROS+ arm mount links are missing from the stage.')

    arm_root_joint = None
    physics = stage.GetPrimAtPath(f'{root_path}/Physics')
    for prim in Usd.PrimRange(physics):
        if not prim.IsA(UsdPhysics.FixedJoint):
            continue
        joint = UsdPhysics.Joint(prim)
        body1_targets = joint.GetBody1Rel().GetTargets()
        if body1_targets == [arm_body.GetPath()]:
            arm_root_joint = joint
            break
    if arm_root_joint is None:
        raise RuntimeError('Imported RB-KAIROS+ arm-root fixed joint is missing.')

    # The mount is a massless Xform beneath the cover.  Preserve its complete
    # pose relative to the physical chassis when changing the joint target.
    relative, reset_xform_stack = UsdGeom.XformCache().ComputeRelativeTransform(
        arm_mount, base_link)
    if reset_xform_stack:
        raise RuntimeError('RB-KAIROS+ arm mount unexpectedly resets its Xform stack.')
    mount_transform = Gf.Transform(relative)
    mount_rotation = mount_transform.GetRotation().GetQuat()

    old_position = arm_root_joint.GetLocalPos0Attr().Get() or Gf.Vec3f(0.0)
    old_rotation = arm_root_joint.GetLocalRot0Attr().Get() or Gf.Quatf(1.0)
    anchor_position = relative.Transform(Gf.Vec3d(old_position))
    anchor_rotation = mount_rotation * Gf.Quatd(old_rotation)

    arm_root_joint.GetBody0Rel().SetTargets([base_link.GetPath()])
    arm_root_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(anchor_position))
    arm_root_joint.CreateLocalRot0Attr().Set(Gf.Quatf(anchor_rotation))


def _sanitize_imported_physics(stage, root_path, robot_model):
    """Discard converter-created rigid bodies for non-physical fixed links.

    Robotnik's Xacro defines base_footprint, the cover and the logos as fixed
    visual links without inertias.  The USD converter nevertheless marked
    them as rigid bodies, leading PhysX to synthesize invalid masses.  Make
    base_link the articulation root instead: it already carries the Robotnik
    mass, inertia and collision geometry.  The child Xforms continue to move
    visually with it, without taking part in the physics articulation.
    """
    from pxr import UsdPhysics

    footprint_path = f'{root_path}/Geometry/robot_base_footprint'
    footprint = stage.GetPrimAtPath(footprint_path)
    base_link = stage.GetPrimAtPath(f'{footprint_path}/robot_base_link')
    if not footprint.IsValid() or not base_link.IsValid():
        raise RuntimeError('Imported RB-KAIROS base links are missing from the stage.')

    footprint.RemoveAPI(UsdPhysics.RigidBodyAPI)
    footprint.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    # Isaac Sim 6 imports the same root through its Newton schema as well.
    # Keeping that weaker applied schema causes the runtime schema bridge to
    # restore PhysicsArticulationRootAPI immediately after we remove it,
    # leaving both footprint and base_link as articulation roots.
    footprint.RemoveAppliedSchema('NewtonArticulationRootAPI')
    UsdPhysics.ArticulationRootAPI.Apply(base_link)

    footprint_joint = stage.GetPrimAtPath(f'{root_path}/Physics/robot_base_footprint_joint')
    if not footprint_joint.IsValid():
        raise RuntimeError('Imported base_footprint joint is missing from the stage.')
    UsdPhysics.Joint(footprint_joint).CreateJointEnabledAttr().Set(False)
    footprint_joint.SetActive(False)

    # The plus model needs an explicit base-to-arm physical connection before
    # robot_top_cover loses its importer-created rigid body below.
    if robot_model == 'rbkairos_plus':
        _attach_arm_to_physical_base(stage, root_path)

    for link_name, joint_name in VISUAL_FIXED_LINKS:
        visual_link = stage.GetPrimAtPath(f'{footprint_path}/robot_base_link/{link_name}')
        fixed_joint = stage.GetPrimAtPath(f'{root_path}/Physics/{joint_name}')
        if not visual_link.IsValid() or not fixed_joint.IsValid():
            raise RuntimeError(f'Imported visual fixed link is missing: {link_name}')
        visual_link.RemoveAPI(UsdPhysics.RigidBodyAPI)
        UsdPhysics.Joint(fixed_joint).CreateJointEnabledAttr().Set(False)
        fixed_joint.SetActive(False)


def _configure_ros_graph(app, root_path, articulation_path, robot_id):
    """Create native ROS 2 control, clock, state and TF OmniGraph nodes."""
    import omni.graph.core as og
    import usdrt

    graph_path = f'{root_path}/ros_graph'
    keys = og.Controller.Keys
    nodes = [
        ('tick', 'omni.graph.action.OnPlaybackTick'),
        ('read_sim_time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
        ('read_joint_state', 'isaacsim.sensors.physics.IsaacReadJointState'),
        ('context', 'isaacsim.ros2.bridge.ROS2Context'),
        ('clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
        ('subscribe_joint_command', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
        ('articulation_controller', 'isaacsim.core.nodes.IsaacArticulationController'),
        ('joint_state', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
        ('compute_tf', 'isaacsim.core.nodes.IsaacComputeTransformTree'),
        ('tf', 'isaacsim.ros2.bridge.ROS2PublishTransformTree'),
    ]
    connections = [
        ('tick.outputs:tick', 'read_joint_state.inputs:execIn'),
        ('read_joint_state.outputs:execOut', 'joint_state.inputs:execIn'),
        ('read_joint_state.outputs:jointNames', 'joint_state.inputs:jointNames'),
        ('read_joint_state.outputs:jointPositions', 'joint_state.inputs:jointPositions'),
        ('read_joint_state.outputs:jointVelocities', 'joint_state.inputs:jointVelocities'),
        ('read_joint_state.outputs:jointEfforts', 'joint_state.inputs:jointEfforts'),
        ('read_joint_state.outputs:jointDofTypes', 'joint_state.inputs:jointDofTypes'),
        ('read_joint_state.outputs:stageMetersPerUnit', 'joint_state.inputs:stageMetersPerUnit'),
        ('read_joint_state.outputs:sensorTime', 'joint_state.inputs:sensorTime'),
        ('tick.outputs:tick', 'clock.inputs:execIn'),
        ('tick.outputs:tick', 'subscribe_joint_command.inputs:execIn'),
        ('tick.outputs:tick', 'articulation_controller.inputs:execIn'),
        ('tick.outputs:tick', 'compute_tf.inputs:execIn'),
        ('compute_tf.outputs:execOut', 'tf.inputs:execIn'),
        ('compute_tf.outputs:parentFrames', 'tf.inputs:parentFrames'),
        ('compute_tf.outputs:childFrames', 'tf.inputs:childFrames'),
        ('compute_tf.outputs:translations', 'tf.inputs:translations'),
        ('compute_tf.outputs:orientations', 'tf.inputs:orientations'),
        ('read_sim_time.outputs:simulationTime', 'clock.inputs:timeStamp'),
        ('read_sim_time.outputs:simulationTime', 'tf.inputs:timeStamp'),
        ('context.outputs:context', 'clock.inputs:context'),
        ('context.outputs:context', 'subscribe_joint_command.inputs:context'),
        ('context.outputs:context', 'joint_state.inputs:context'),
        ('context.outputs:context', 'tf.inputs:context'),
        ('subscribe_joint_command.outputs:jointNames', 'articulation_controller.inputs:jointNames'),
        ('subscribe_joint_command.outputs:positionCommand', 'articulation_controller.inputs:positionCommand'),
        ('subscribe_joint_command.outputs:velocityCommand', 'articulation_controller.inputs:velocityCommand'),
    ]
    values = [
        ('subscribe_joint_command.inputs:nodeNamespace', robot_id),
        ('subscribe_joint_command.inputs:topicName', 'isaac/joint_command'),
        ('articulation_controller.inputs:robotPath', articulation_path),
        ('read_joint_state.inputs:prim', [usdrt.Sdf.Path(articulation_path)]),
        ('joint_state.inputs:nodeNamespace', robot_id),
        ('joint_state.inputs:topicName', 'isaac/joint_states'),
        ('compute_tf.inputs:targetPrims', [usdrt.Sdf.Path(articulation_path)]),
        ('tf.inputs:nodeNamespace', robot_id),
    ]
    graph, _, _, _ = og.Controller.edit(
        {'graph_path': graph_path, 'evaluator_name': 'execution'},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connections, keys.SET_VALUES: values},
    )
    og.Controller.evaluate_sync(graph)
    app.update()


def main():
    args = parse_arguments()
    from isaacsim import SimulationApp

    app = SimulationApp({'headless': args.headless == 'true', 'sync_loads': True})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        from omni.usd import get_context
        import omni.timeline

        print('RB-KAIROS: enabling ROS 2 bridge.')
        enable_extension('isaacsim.ros2.bridge')
        # Kit registers bridge and renderer services asynchronously.  Drive
        # two frames before creating the stage, as the Isaac launch helpers
        # do, otherwise GUI runs can report is_running()==False immediately.
        app.update()
        print(f'RB-KAIROS: bridge frame 1 complete (Kit running={app.is_running()}).')
        app.update()
        print(f'RB-KAIROS: bridge frame 2 complete (Kit running={app.is_running()}).')
        if not app.is_running():
            raise RuntimeError('Isaac Sim stopped while enabling the ROS 2 bridge.')
        description = Path(args.description_share)
        context = get_context()
        context.new_stage()
        stage = context.get_stage()
        if stage is None:
            raise RuntimeError('Isaac Sim did not create the simulation stage.')
        print('RB-KAIROS: creating world.')
        _create_world(stage)
        if not args.world_only:
            asset_name = 'rbkairos_base' if args.robot_model == 'rbkairos' else 'rbkairos_plus'
            asset = description / 'assets' / 'rbkairos' / 'usd' / asset_name / f'{asset_name}.usda'
            if not asset.is_file():
                raise FileNotFoundError(f'Robot USD is missing: {asset}')
            print(f'RB-KAIROS: composing {asset.name}.')
            root_path = _create_robot(stage, args, asset)
            print('RB-KAIROS: robot USD composed.', flush=True)
            _release_imported_root_joint(stage, root_path)
            print('RB-KAIROS: imported root joint released.', flush=True)
            _sanitize_imported_physics(stage, root_path, args.robot_model)
            print('RB-KAIROS: imported physics sanitized.', flush=True)
            _configure_wheel_velocity_drives(stage, root_path)
            print('RB-KAIROS: wheel drives configured.', flush=True)
            if args.robot_model == 'rbkairos_plus':
                _configure_arm_position_drives(stage, root_path)
                print('RB-KAIROS: arm position drives configured.', flush=True)
            articulation_path = _find_articulation_root(stage, root_path)
            print(f'RB-KAIROS: using articulation root {articulation_path}.')
            _configure_ros_graph(app, root_path, articulation_path, args.robot_id)
            print('RB-KAIROS: robot and ROS graph configured.')
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        app.update()
        print('RB-KAIROS simulation running. Close the Isaac window or press Ctrl+C to stop.')
        while app.is_running():
            app.update()
    except BaseException as error:
        # Kit replaces Python's default exception hook and can otherwise hide
        # failures that occur while composing a USD.  Emit the traceback while
        # the logging and terminal streams are still alive, before app.close().
        print(
            f'RB-KAIROS: fatal runtime error ({type(error).__name__}): {error}',
            flush=True,
        )
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == '__main__':
    main()
