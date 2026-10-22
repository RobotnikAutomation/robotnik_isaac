# Robotnik Isaac architecture and repository boundary

This document describes the implemented architecture of the `robotnik_isaac`
repository. The generic `robotnik_isaac_ros2_control` package has already been
separated into its own repository and is included here as a Git submodule at
`robotnik_isaac_ros2_control/`. It can also be checked out independently in a
ROS 2 workspace.

## Responsibility matrix

| Concern | Owner | Current files |
| --- | --- | --- |
| Generic ros2_control core and JointState protocol adapter | `robotnik_isaac_ros2_control` | `include/`, `src/`, plugin XML |
| Per-robot ros2_control declarations, joint order, interfaces and limits | `robotnik_isaac_description` | `config/*.ros2_control.urdf` |
| USD/URDF assets, model registry, import and metadata | `robotnik_isaac_description` | `assets/`, `urdf_templates/`, `scripts/`, `config/*.yaml` |
| Isaac runtime, graph construction, model-specific PhysX sanitation and generic stage extension loading | `robotnik_isaac_bringup` | `runtime/` |
| Stable robot launch surface and controller composition | `robotnik_isaac_bringup` | `launch/`, `config/` |
| Gripper launch, descriptions and vendor asset adapters | `robotnik_isaac_grippers` | `robotnik_isaac_grippers/` |
| Sensor profiles | `robotnik_isaac_sensors` | `config/` |
| Isaac-specific MoveIt adaptation | `robotnik_isaac_moveit_config` | all package files |
| Benchmarking integration | `robotnik_sim_benchmarking` owns the `robotnik_isaac_benchmark/` directory, whose ROS package is named `isaac_sim` | outside this simulation system |

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

## Robot description: USD and URDF

The description is split by runtime responsibility. Isaac Sim loads the USD
composition root selected by the model registry: `rbkairos_base.usda` for
`rbkairos`, or `rbkairos_plus.usda` for `rbkairos_plus`. Each root composes its
own `payloads/` layers, which contain the imported robot geometry, materials
and physics. Keep each root beside its payload directory so its relative USD
references remain valid. These USD assets are the runtime robot in Isaac Sim:
they provide the simulated articulation, visual and collision geometry, and
physics behavior.

The runtime control URDFs in `robotnik_isaac_description/config/` serve a
different purpose. They are intentionally lightweight integration skeletons,
not the visual or physical robot model. They declare the joint/link names and
`ros2_control` state and command interfaces consumed by controller manager
and `robotnik_isaac_ros2_control`; the bridge exchanges joint state and command
data with Isaac over the configured ROS topics. The launch also gives this
URDF to `robot_state_publisher`, while isolating it from Isaac's joint-state
stream, so it does not replace the simulated robot's transform tree.

The separate xacro templates and generated, xacro-free URDFs under
`robotnik_isaac_description/urdf_templates/` and
`robotnik_isaac_description/assets/rbkairos/urdf/` are offline import inputs
and references. They are used to prepare or regenerate Isaac assets; bringup
does not load them in place of the USD runtime model. In short, USD defines
what Isaac simulates, while the runtime URDF defines what ROS 2 controllers
can observe and command. The model YAML connects the runtime to its USD and
declares the associated drive and topic configuration.

When `gripper_model:=2fg7` is selected for `rbkairos_plus`,
`robotnik_isaac_grippers` references the OnRobot USD beneath the UR10e
`arm_tool0` with the documented 80 mm mount offset. Its fixed root joint is
connected to the wrist body through a fixed joint under the robot physics
scope, and the standalone gripper mount joint/root schemas are disabled or
removed so the KAIROS+ remains one physics articulation. These changes exist
only in the in-memory stage; the imported OnRobot repositories are not
modified. A separate lightweight URDF description publishes gripper frames
prefixed by `robot_id` and consumes the simulated `finger_stroke` state.
The physical joint name remains unchanged. No gripper control interfaces or
MoveIt planning group are added in this phase. With the default empty
`gripper_model`, neither the USD nor URDF gripper description is launched.

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

## Benchmarking boundary

The `robotnik_isaac_benchmark/` directory is exclusively for
`robotnik_sim_benchmarking`. Although it is present in this checkout and its
ROS package is named `isaac_sim`, it is not part of the `robotnik_isaac`
simulation system described here. The platform bringup, package responsibilities
and launch guide in the root README do not include that directory.

## Repository separation and distribution

`robotnik_isaac_ros2_control` is maintained in its own repository and is
included in this repository through `.gitmodules` for source-based development.
The submodule records an exact commit, while the `jazzy_devel` branch is the
development target. Release manifests must pin a release tag or immutable
commit rather than a moving branch. The package remains independent of Isaac
Sim, Robotnik descriptions, controllers, MoveIt and USD, so it can be built and
tested on supported ROS 2 distributions without Isaac Sim.

The independent repository provides its own Apache-2.0 license and copyright
files, README, changelog, semantic tags and compatibility coverage. Its CI
should load the plugin through pluginlib, test complete and partial
`JointState` messages, duplicate and non-finite values, mixed interface output
and namespace isolation. Isaac-enabled validation belongs to the integration
repository and should cover stock bridge node ports, base motion and arm
trajectory execution.

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
