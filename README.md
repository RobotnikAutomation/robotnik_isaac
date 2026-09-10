# Robotnik Isaac Sim

Isaac Sim-specific simulation packages.  These packages deliberately do not
modify `robotnik_description`, `robotnik_sensors` or Gazebo packages.

## Packages

| Package | Responsibility |
| --- | --- |
| `robotnik_isaac_description` | USD composition and robot metadata. |
| `robotnik_isaac_sensors` | Sensor profiles and ROS topic contracts. |
| `robotnik_isaac_ros2_control` | `ros2_control` bridge between Isaac JointState topics and Robotnik controllers. |
| `robotnik_isaac_bringup` | Isaac Sim world/robot launch files. |
| `isaac_sim` | Legacy RB-Watcher benchmark; not a platform bringup package. |

The first supported platform is `rbkairos`, followed by `rbkairos_plus` with
an UR10e arm.  The USD assets are the Isaac source of truth. Robotnik's xacro
and Gazebo files are reference material only.
