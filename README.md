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
sudo apt install python3-vcstool git-lfs curl
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
`src/onrobot/onrobot-isaacsim`. The control integration builds the required OnRobot ROS packages alongside Robotnik.

### 4. Install OnRobot Tool API 1.2.0

On Ubuntu 24.04 amd64 with ROS 2 Jazzy, install the matching runtime and
development packages **before building**, including for the Isaac backend:
the OnRobot hardware and controller plugins link against this SDK.
From the workspace root:

```bash
mkdir -p tool-api-debs
cd tool-api-debs
curl -fL -O https://github.com/OnRobot/onrobot-tool-api/releases/download/1.2.0/libonrobot-tool-api1_1.2.0-1_amd64.deb
curl -fL -O https://github.com/OnRobot/onrobot-tool-api/releases/download/1.2.0/libonrobot-tool-api-dev_1.2.0-1_amd64.deb
sudo apt install ./libonrobot-tool-api1_1.2.0-1_amd64.deb ./libonrobot-tool-api-dev_1.2.0-1_amd64.deb
dpkg-query -W libonrobot-tool-api1 libonrobot-tool-api-dev
cd ..
```

Both packages must report `1.2.0-1`. If CMake reports that it found Tool API
`0.1.0` but needs `1.2.0`, update both packages using the commands above and
rebuild. Do not lower the CMake version requirement. The public
[Tool API repository](https://github.com/OnRobot/onrobot-tool-api) provides
documentation; cloning it through VCS does not install the SDK. Its binary
packages are installed separately from the two repositories in `onrobot.repos`.

### 5. Install dependencies and build

Run from the workspace root. The workspace must also contain the Robotnik
controller and description dependencies required by the package manifests.

```bash
cd ~/robotnik_isaac_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/robotnik_isaac src/onrobot/onrobot-ros2 \
  --ignore-src -r -y --skip-keys onrobot_tool_api
colcon build --base-paths src \
  --packages-up-to robotnik_isaac_bringup robotnik_isaac_moveit_config \
  --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

The skipped rosdep key is the SDK installed above. The build includes the
OnRobot dependencies declared by the gripper package. Supplied OnRobot source
distributions can omit internal tests, hence `BUILD_TESTING=OFF`.

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
It also starts an independent `/robot/gripper/controller_manager` with the
OnRobot Isaac hardware plugin and conventional aperture controller. It does
not load the realtime controller or start a second standalone OnRobot launch. To use another asset checkout,
set `ONROBOT_ISAAC_ASSET_REPOSITORY` to the `onrobot-isaacsim` repository root.
Add `headless:=true` to the command to run the gripper integration without
the Isaac GUI.

### Control the 2FG7

Wait for Isaac to finish starting and for feedback to arrive. No opening goal
is sent automatically. In a second sourced terminal, check controller state:

```bash
ros2 control list_controllers -c /robot/gripper/controller_manager
ros2 action info /robot/gripper/gripper_controller/gripper_cmd
```

The joint-state, limit and typed-state broadcasters, recovery controller and
`gripper_controller` should be active. The action uses
`control_msgs/action/ParallelGripperCommand`. `grip_stroke` is the **total
aperture in metres**, from 0 to 0.073 with the standard finger configuration;
the hardware plugin maps it to the physical `finger_stroke` joint (0–0.019 m).
For an empty gripper, command an intermediate aperture:

```bash
ros2 action send_goal /robot/gripper/gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: [grip_stroke], position: [0.035], effort: [20.0]}}" --feedback
```

Use `position: [0.0]` for closed and `position: [0.073]` for fully open.
Percentage speed control is disabled for Isaac. This integration does not
establish force accuracy for contacts or grasped objects.

Feedback comes from `/robot/isaac/joint_states`; gripper commands use
`/robot/gripper/isaac/joint_command`, separate from the base/arm command topic.
The description is published on `/robot/gripper/robot_description`, and the
gripper's joint-state broadcaster uses its local topic. Replace `/robot` in
these commands when using a different `robot_id`.

For an integration check, send closed, intermediate and open goals without an
object and verify symmetric finger motion and action results. Move the arm
while sending gripper goals to check that commands remain independent. Cancel
an in-progress goal using an action client and verify that motion stops. Stop
Isaac feedback during a goal and check that stale state does not produce a
successful result; inspect the typed state and controller diagnostics.
Finally launch without `gripper_model` and confirm the base/arm still work.

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
