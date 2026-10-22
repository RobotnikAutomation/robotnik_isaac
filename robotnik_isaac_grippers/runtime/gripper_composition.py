"""Compose the supported OnRobot gripper into the Robotnik USD stage."""

import json
import os
from pathlib import Path


def resolve_2fg7_asset(description_share):
    """Resolve the LFS-backed 2FG7 asset from a path or workspace checkout."""
    candidates = []
    override = os.environ.get('ONROBOT_ISAAC_ASSET_REPOSITORY')
    if override:
        candidates.append(
            Path(override).expanduser() /
            'assets/onrobot_2fg7/simready_usd/onrobot_2fg7.usda')
    description_path = Path(description_share).resolve()
    for parent in description_path.parents:
        candidates.append(
            parent / 'src/onrobot/onrobot-isaacsim/assets/onrobot_2fg7/simready_usd/onrobot_2fg7.usda')
    for asset in candidates:
        if not asset.is_file():
            continue
        if asset.read_bytes().startswith(b'version https://git-lfs.github.com/spec/v1'):
            raise RuntimeError(
                f'OnRobot 2FG7 USD is an unresolved Git LFS pointer: {asset}')
        repository = asset.parents[3]
        manifest_path = repository / 'asset_manifest.json'
        try:
            files = json.loads(manifest_path.read_text(encoding='utf-8'))['files']
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f'OnRobot asset manifest is missing or invalid: {manifest_path}') from error
        for relative_path in files:
            if not relative_path.startswith('assets/onrobot_2fg7/'):
                continue
            file_path = repository / relative_path
            if not file_path.resolve().is_relative_to(repository.resolve()):
                raise RuntimeError(
                    f'OnRobot asset manifest path escapes the repository: {relative_path}')
            if not file_path.is_file():
                raise RuntimeError(f'OnRobot asset payload is missing: {file_path}')
            if file_path.read_bytes().startswith(b'version https://git-lfs.github.com/spec/v1'):
                raise RuntimeError(
                    f'OnRobot asset is an unresolved Git LFS pointer: {file_path}. '
                    'Run git lfs pull in src/onrobot/onrobot-isaacsim.')
        return asset.resolve()
    raise FileNotFoundError(
        'OnRobot 2FG7 USD was not found. Import the repositories from '
        'repos/onrobot.repos or set '
        'ONROBOT_ISAAC_ASSET_REPOSITORY.')


def resolve_2fg7_description(description_share):
    """Resolve the external ROS description source needed for the URDF frames."""
    description_path = Path(description_share).resolve()
    for parent in description_path.parents:
        repository = parent / 'src/onrobot/onrobot-ros2'
        macro = repository / 'onrobot_2fg7/urdf/onrobot_2fg7_macro.xacro'
        meshes = repository / 'onrobot_2fg7/meshes'
        if macro.is_file() and meshes.is_dir():
            return macro.resolve(), meshes.resolve()
    raise FileNotFoundError(
        'OnRobot 2FG7 description was not found. Import src/onrobot/onrobot-ros2.')


def compose_2fg7(stage, robot_root_path, asset):
    """Reference the 2FG7 beneath arm_tool0 and join it to the UR10e body.

    All changes are authored in the current stage. The checked-out OnRobot USD
    and its relative payload layers remain untouched.
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    tool_path = (
        f'{robot_root_path}/Geometry/robot_base_footprint/robot_base_link/'
        'robot_top_cover/arm_base_link/arm_base_link_inertia/'
        'arm_shoulder_link/arm_upper_arm_link/arm_forearm_link/'
        'arm_wrist_1_link/arm_wrist_2_link/arm_wrist_3_link/arm_flange/arm_tool0'
    )
    wrist_path = tool_path.rsplit('/arm_flange/arm_tool0', 1)[0]
    tool = stage.GetPrimAtPath(tool_path)
    wrist = stage.GetPrimAtPath(wrist_path)
    if not tool.IsValid() or not wrist.IsValid():
        raise RuntimeError('RB-KAIROS+ arm_tool0 or wrist body is missing from USD.')
    if not wrist.HasAPI(UsdPhysics.RigidBodyAPI):
        raise RuntimeError(f'UR10e wrist prim is not a rigid body: {wrist_path}')

    mount_path = f'{tool_path}/onrobot_2fg7_mount'
    mount = UsdGeom.Xform.Define(stage, mount_path)
    # The asset base frame is already its mounting face. Attach it directly
    # to tool0; an extra translation leaves a gap without an adapter body.
    mount.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    asset_path = f'{mount_path}/asset'
    asset_prim = stage.DefinePrim(asset_path, 'Xform')
    if not asset_prim.GetReferences().AddReference(str(asset)):
        raise RuntimeError(f'Could not reference OnRobot 2FG7 USD: {asset}')

    body1_path = f'{asset_path}/Geometry/base_link'
    body1 = stage.GetPrimAtPath(body1_path)
    joint_path = f'{asset_path}/Physics/root_joint'
    joint_prim = stage.GetPrimAtPath(joint_path)
    if not body1.IsValid() or not joint_prim.IsValid():
        raise RuntimeError('OnRobot 2FG7 USD is missing its base body or root joint.')

    # The stand-alone asset declares a fixed joint to world and a second
    # articulation root. Disable that mount, remove its nested root API, and
    # author the wrist connection alongside the robot's own physics joints.
    joint_prim.RemoveAppliedSchema('PhysicsArticulationRootAPI')
    joint_prim.RemoveAppliedSchema('NewtonArticulationRootAPI')
    UsdPhysics.Joint(joint_prim).CreateJointEnabledAttr().Set(False)
    joint_prim.SetActive(False)
    mount_joint_path = f'{robot_root_path}/Physics/onrobot_2fg7_mount_joint'
    joint = UsdPhysics.FixedJoint.Define(stage, mount_joint_path)
    joint.GetBody0Rel().SetTargets([wrist.GetPath()])
    joint.GetBody1Rel().SetTargets([body1.GetPath()])

    relative, reset_xform_stack = UsdGeom.XformCache().ComputeRelativeTransform(
        body1, wrist)
    if reset_xform_stack:
        raise RuntimeError('OnRobot 2FG7 mount unexpectedly resets its Xform stack.')
    mount_transform = Gf.Transform(relative)
    rotation = mount_transform.GetRotation().GetQuat()
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(mount_transform.GetTranslation()))
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(rotation))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))

    if stage.GetPrimAtPath(f'{robot_root_path}/Physics/root_joint').IsValid():
        return asset_path, body1_path, mount_joint_path
    raise RuntimeError('RB-KAIROS+ articulation root is missing after composition.')
