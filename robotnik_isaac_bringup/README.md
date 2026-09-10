# Robotnik Isaac bringup

The launch surface deliberately follows Robotnik naming (`robot_id`, `robot`,
`robot_model`, pose and arm type) while starting no Gazebo component. At this
stage it supports Isaac-native USD composition for `rbkairos` and
`rbkairos_plus`.

`isaac_sim_path` is explicit: it must point to the Isaac Sim 6 installation
and is never inferred from another simulator or ROS package.

Base control is provided by `robotnik_isaac_ros2_control` together with the
official `robotnik_controllers/RBKairosController`. No transitional Python
kinematics or state bridge is launched.
