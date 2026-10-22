# Robotnik Isaac Sim

Isaac Sim integration for Robotnik robots using ROS 2. The supported models are
RB-KAIROS (`rbkairos`) and RB-KAIROS+ with a UR10e arm (`rbkairos_plus`). This
repository contains the robot descriptions, Isaac runtime and ROS 2 bringup
packages for that integration.

> **Development status:** This repository is a preliminary work in progress.
> Its functionality is currently limited and under testing; it does not yet
> implement the complete Robotnik robot software stack. Capabilities will be
> added incrementally as development continues. The early-stage
> `robotnik_isaac_ros2_control` integration is currently available for testing
> by the development team only.

> **Benchmarking boundary:** `robotnik_isaac_benchmark/` is reserved exclusively
> for `robotnik_sim_benchmarking`. It does not belong to this Isaac Sim
> integration. Its files and launch workflows are outside the scope of this
> guide.

## Packages

| Package | Responsibility |
| --- | --- |
| `robotnik_isaac_description` | Robot USD/URDF assets, model configuration and robot-specific control declarations. |
| `robotnik_isaac_sensors` | Sensor profiles and ROS topic configuration. |
| `robotnik_isaac_bringup` | Isaac runtime, worlds, robot launch and controller startup. |
| `robotnik_isaac_grippers` | Robotnik-owned gripper launches, descriptions and Isaac scene extensions; currently integrates the OnRobot 2FG7. |
| `robotnik_isaac_moveit_config` | Optional MoveIt 2 setup for the RB-KAIROS+ UR10e arm. |
| `robotnik_isaac_ros2_control` | Generic Isaac joint bridge, included as a Git submodule. |

See [docs/architecture.md](docs/architecture.md) for package boundaries,
runtime behavior, how the robot description is composed, and current support
limits. The bridge's transport parameters and ROS 2 control contract are documented in the
[`robotnik_isaac_ros2_control` README](robotnik_isaac_ros2_control/README.md).

## Prerequisites

- Ubuntu with ROS 2 Jazzy.
- Isaac Sim 6 installed locally. The standard `bringup_complete.launch.py`
  entry point defaults to `/home/robotnik/isaacsim`. The lower-level
  `spawn_robot.launch.py` and `spawn_world.launch.py` use `$ISAAC_SIM_PATH` when
  set, then fall back to that same path. Set the variable to the directory
  containing Isaac's `python.sh`:

  ```bash
  export ISAAC_SIM_PATH="$HOME/isaacsim"
  ```

- The ROS 2 workspace must also contain the Robotnik controller and description
  dependencies required by the package manifests.

## Get the workspace and build

The examples use `~/robotnik_isaac_ws` as the workspace and keep this
repository at `src/robotnik_isaac`.

### 1. Install workspace tools

Run from any directory:

```bash
sudo apt install python3-vcstool git-lfs
git lfs install
```

### 2. Create the workspace and clone this repository

```bash
mkdir -p ~/robotnik_isaac_ws/src
cd ~/robotnik_isaac_ws/src
git clone https://github.com/RobotnikAutomation/robotnik_isaac.git
```

### 3. Import the submodule and OnRobot repositories

Run from the workspace root:

```bash
cd ~/robotnik_isaac_ws
git -C src/robotnik_isaac submodule update --init --recursive
vcs import src < src/robotnik_isaac/repos/onrobot.repos
git -C src/onrobot/onrobot-isaacsim lfs pull
```

The VCS manifest imports the OnRobot repositories into `src/onrobot/`, next
to the `src/robotnik_isaac/` repository. The Robotnik-owned gripper package
`robotnik_isaac_grippers` lives inside `src/robotnik_isaac/`. The OnRobot
repositories are workspace checkouts, not Git submodules.
For a workspace that already has them at `src/onrobot-isaacsim` and
`src/onrobot-ros2`, move those directories under `src/onrobot/` before
re-importing the manifest to avoid duplicate checkouts.
The Isaac asset repository uses Git LFS. The manifest pins a compatible pair
of revisions; update them together so the ROS package's asset requirements
stay satisfied. The optional 2FG7 description and meshes are read directly
from `src/onrobot/onrobot-ros2`; its simulation asset is loaded from
`src/onrobot/onrobot-isaacsim`. The standard Robotnik build remains
scoped to Robotnik packages and does not build the OnRobot ROS stack.

### 4. Install dependencies and build

Run from the workspace root. The workspace must also contain the Robotnik
controller and description dependencies required by the package manifests.

```bash
cd ~/robotnik_isaac_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/robotnik_isaac --ignore-src -r -y
colcon build --base-paths src/robotnik_isaac \
  --packages-up-to robotnik_isaac_bringup robotnik_isaac_moveit_config
source install/setup.bash
```

For each new terminal, run these setup commands from the workspace root before
using ROS packages:

```bash
cd ~/robotnik_isaac_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Launch a simulation

Run launch commands from the workspace root in a sourced terminal.

Start RB-KAIROS with the default empty world and Isaac GUI:

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_bringup bringup_complete.launch.py
```

Start RB-KAIROS+ with the UR10e arm:

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_bringup bringup_complete.launch.py \
  robot_model:=rbkairos_plus
```

Add the optional OnRobot 2FG7 to RB-KAIROS+:

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_bringup bringup_complete.launch.py \
  robot_id:=robot robot_model:=rbkairos_plus gripper_model:=2fg7
```

`gripper_model` is empty by default, so the usual bringup has no gripper.
This first integration supports only `2fg7` and only with `rbkairos_plus`;
other model/robot combinations fail before Isaac starts. The
`robotnik_isaac_grippers` launch prepares the description and publishes the
prefixed gripper frames; its runtime extension composes the USD.
It does not start OnRobot's ROS control launch. To use another asset checkout,
set `ONROBOT_ISAAC_ASSET_REPOSITORY` to the `onrobot-isaacsim` repository root.
Add `headless:=true` to the command to run the gripper integration without
the Isaac GUI.

Run either model without the Isaac GUI:

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_bringup bringup_complete.launch.py \
  robot_model:=rbkairos_plus headless:=true
```

Choose another installed world file with `world_path`:

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_bringup bringup_complete.launch.py \
  robot_model:=rbkairos world_path:=/absolute/path/to/world.usda
```

Set the Isaac installation per launch with `isaac_sim_path` (this also
overrides the default on `bringup_complete.launch.py`):

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_bringup bringup_complete.launch.py \
  isaac_sim_path:=/absolute/path/to/isaacsim
```

The lower-level `spawn_robot.launch.py` exposes the same robot bringup
arguments. To open a world without spawning a robot, use
`spawn_world.launch.py`; it accepts `world_path`, `headless`, and
`isaac_sim_path`.

Useful launch arguments on the complete bringup include `robot_id` (ROS
namespace), `robot_model`, `gripper_model`, `x`, `y`, `z`, `yaw`,
`frame_prefix`, `headless`, `world_path`, and `isaac_sim_path`. The default
robot namespace is `robot`.

## Optional MoveIt for RB-KAIROS+

First launch the RB-KAIROS+ simulation and wait for the ROS controllers to
start. In another sourced terminal, launch MoveIt:

```bash
cd ~/robotnik_isaac_ws
ros2 launch robotnik_isaac_moveit_config isaac_moveit.launch.py robot_id:=robot
```

MoveIt plans for the `arm` group and uses the running simulation's joint states
and `arm_controller`. The bringup must already be running; this launch does
not start Isaac Sim or the controllers. To run MoveIt without its RViz window,
add `launch_rviz:=false`.

## Asset maintenance

The composed USD assets under
`robotnik_isaac_description/assets/rbkairos/usd/` must stay alongside their
`payloads/` directories because the USD layers use relative references. The
xacro-free URDF import inputs are under
`robotnik_isaac_description/assets/rbkairos/urdf/`.

To regenerate those URDF files after updating the upstream description or
meshes, run from the workspace root:

```bash
cd ~/robotnik_isaac_ws
python3 src/robotnik_isaac/robotnik_isaac_description/scripts/generate_urdfs.py \
  --workspace "$PWD"
```

The importer is an offline asset workflow and requires the source Robotnik
description/sensor packages and ROS 2 Jazzy's `ur_description` to be available
in the workspace/system paths used by the script.
