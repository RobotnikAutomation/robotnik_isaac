# Robotnik Isaac architecture and extraction boundary

This document records the implemented boundary while the generic package still
shares the `robotnik_isaac_sim` Git repository. The future integration
repository is named `robotnik_isaac`; renaming or history extraction is not
part of this migration.

## Responsibility matrix

| Concern | Owner | Current files |
| --- | --- | --- |
| Generic ros2_control core and JointState protocol adapter | `robotnik_isaac_ros2_control` | `include/`, `src/`, plugin XML |
| Per-robot ros2_control declarations, joint order, interfaces and limits | `robotnik_isaac_description` | `config/*.ros2_control.urdf` |
| USD/URDF assets, model registry, import and metadata | `robotnik_isaac_description` | `assets/`, `urdf_templates/`, `scripts/`, `config/*.yaml` |
| Isaac runtime, graph construction and model-specific PhysX sanitation | `robotnik_isaac_bringup` | `runtime/isaac_robot_runtime.py` |
| Stable launch surface, controller composition and TF | `robotnik_isaac_bringup` | `launch/`, `config/` |
| Sensor profiles | `robotnik_isaac_sensors` | `config/` |
| Isaac-specific MoveIt adaptation | `robotnik_isaac_moveit_config` | all package files |
| Legacy RB-Watcher benchmark | directory `robotnik_isaac_benchmark`, package `isaac_sim` | entire package |

The two Kairos control URDFs were moved, without renaming, from the generic
package to description `config/`. They were not duplicated. Launch now resolves
the installed description package. The generic CMake no longer installs an
`urdf/` directory and its manifest has no platform dependency. The description
manifest has the one-way runtime dependency on the plugin.

## Stable integration surface

`spawn_robot.launch.py` remains the low-level entry point and
`bringup_complete.launch.py` remains the composition entry point. Stable inputs
are `robot_id`, `robot_model`, `arm_type`, pose (`x`, `y`, `z`, `yaw`),
`frame_prefix`, `headless`, `run_rviz` and `isaac_sim_path`. Relative bridge
topics become `/<robot_id>/isaac/...`; TF and controllers remain namespaced.
This mirrors the Gazebo integration at the launch/controller boundary, while
USD composition, OmniGraph and PhysX correction stay Isaac-native.

The model registry is `robotnik_isaac_description/config/<model>.yaml`. It
selects the USD, ROS topic contract, velocity-drive joints and position-drive
home targets. Generic graph construction reads the registry. Kairos-specific
functions still release converter-created joints, repair invalid inertias and
retarget the arm mount; those are deliberately not part of the C++ bridge.

## Kairos contract and consistency

Both models publish `isaac/joint_states` and consume `isaac/joint_command`
inside `robot_id`. RB-Kairos declares four wheel velocity commands. RB-Kairos+
adds six UR10e position commands: `arm_shoulder_pan_joint`,
`arm_shoulder_lift_joint`, `arm_elbow_joint`, `arm_wrist_1_joint`,
`arm_wrist_2_joint`, `arm_wrist_3_joint`. Its home pose is
`[0, -2.4125, 2.4125, -1.57, -1.57, 0]` radians. The runtime converts these
targets to PhysX degrees. The base frame is `robot_base_footprint`, odometry is
`robot_odom`, and the arm root is `arm_base_link`.

The contract test compares model YAML against the control URDF, including
topics, command-interface membership and duplicate joint names. MoveIt and the
controller YAML use the same six `arm_*` names. Limits currently remain in the
control URDF and MoveIt configuration because they serve different consumers;
the contract test prevents the structural names from diverging. The imported
USD is validated at runtime because USD Python APIs are unavailable in normal
ROS CI.

## Benchmark decision

The `robotnik_isaac_benchmark` directory and its `isaac_sim` package are retained and explicitly classified as legacy. It
is not included by platform bringup and receives no new platform features.
Existing package and launch names remain valid. A later major release should
move it to an examples/benchmarks repository after one deprecation release;
renaming it now would break users for no benefit to the common runtime.

## Distribution and future Git extraction

No `.gitmodules` is added yet: there is no independent remote or immutable
commit to pin, and pointing a submodule back into the current repository would
not be reproducible. Once the control directory is extracted, add it at
`vendor/robotnik_isaac_ros2_control`, use a release tag plus recorded commit,
and never track a moving branch in release manifests. Development checkouts may
use a `.repos` file instead. Released `robotnik_isaac` packages should prefer a
rosdep-resolvable binary dependency; the submodule is a source-development
fallback, not nested package content in a Debian release.

The independent repository needs Apache-2.0 license and copyright files,
README, changelog, semantic tags and a compatibility matrix. CI should build on
supported ROS distributions without Isaac; load the plugin through pluginlib;
inject complete, partial, duplicate, stale and non-finite JointState messages;
verify mixed interface output and namespace isolation; and run URDF launch smoke
tests. An Isaac-enabled nightly job should validate stock bridge node ports,
base motion and arm trajectory execution.

## Compatibility and diagnostics

The locally validated build baseline is ROS 2 Jazzy with ros2_control 4.48.0
and Isaac Sim 6.0.1 RC 7. ros2_control 4.48.0 must be paired with
`realtime_tools >= 3.12.0`; older binaries lack `set_current_thread_name` and
cannot link ResourceManager tests. The installed Isaac node definitions expose
position, velocity and effort arrays on `ROS2SubscribeJointState` and
`IsaacArticulationController`; the runtime connects all three. Other Isaac or
ROS releases are not claimed until CI adds them to this matrix. A missing
plugin points to an unsourced install or pluginlib export; no joint states
usually means a namespace/topic or QoS mismatch; a read failure with
`require_all_joints` means a partial/renamed state message; watchdog failures
mean simulation clock or publishing stopped; spinning wheels with a stationary
base points to the imported root-joint sanitation; and a collapsing arm points
to missing position-drive gains or home targets.

## Scope

Immediate support is base control, native clock/joint state/command/TF graph,
installed sensor profiles and the isolated UR10e MoveIt adapter. Sensor graph
instantiation and `run_rviz` composition are not implemented yet; the existing
arguments are retained for launch compatibility. Navigation,
perception pipelines, battery, docking, supervisor behavior and multiple robot
instances in one Isaac process remain later phases. Independent namespaced
processes are supported now; one stage containing multiple instances needs a
runtime composition API before it can be claimed.

## Change classification

| Change | Classification | Compatibility |
| --- | --- | --- |
| Discover joints/interfaces from URDF | API extension and internal refactor | Existing Kairos declarations remain valid. |
| Mixed position/velocity/effort JointState arrays | Documented existing protocol plus effort extension | Stock Isaac 6 ports; no message type change. |
| QoS, timeout, validation and limits parameters | New API and optional behavior | Defaults preserve no-watchdog/partial-message operation. |
| Move control URDFs to description | File move and new one-way dependency | Filenames and launch names unchanged. |
| YAML model registry and absolute asset resolution | Internal runtime refactor and robustness fix | Existing model names and assets unchanged. |
| `world_path` and `ISAAC_SIM_PATH` | Launch API extension | Legacy Isaac path remains fallback. |
| Benchmark legacy classification | Documentation only | Package `isaac_sim` is unchanged. |
