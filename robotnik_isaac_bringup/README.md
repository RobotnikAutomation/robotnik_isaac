# Robotnik Isaac bringup

The launch surface deliberately follows Robotnik naming (`robot_id`, `robot`,
`robot_model`, pose and arm type) while starting no Gazebo component. At this
stage it supports Isaac-native USD composition for `rbkairos` and
`rbkairos_plus`.

`isaac_sim_path` is explicit: it must point to the Isaac Sim 6 installation
and is never inferred from another simulator or ROS package.

Base control is provided by `robotnik_isaac_ros2_control` together with the
official `robotnik_controllers/RBKairosController`. For `rbkairos_plus`, the
same hardware plugin also exposes the six `arm_*` position interfaces and the
launch activates `arm_controller` after the base controller. Mixed wheel
velocity and arm position commands share the native Isaac joint bridge.

The robot-specific control URDF is installed by
`robotnik_isaac_description/config` and selected from `robot_model`:
`rbkairos` keeps the original
four-wheel interface, while `rbkairos_plus` adds the UR10e. Controllers start
serially as `joint_state_broadcaster`, `robotnik_base_control`, then
`arm_controller`.

Model assets, drive joints, initial position targets and bridge topic names are
read from `robotnik_isaac_description/config/<robot_model>.yaml`. Adding a model
there removes model selection from generic graph code, while Kairos USD repair
functions remain intentionally local to this runtime. Relative bridge topics
resolve under `robot_id`, permitting isolated simultaneous robot processes.
