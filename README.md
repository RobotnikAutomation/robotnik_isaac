# Robotnik Isaac Sim

Isaac Sim-specific simulation packages.  These packages deliberately do not
modify `robotnik_description`, `robotnik_sensors` or Gazebo packages.

> **Development disclaimer:** This integration is currently under active
> development and is not yet fully functional with the complete simulation
> system. The `isaac_ros2_control` integration layer is also being developed
> privately and will be used to connect the different components into a
> complete system.

## Packages

| Package | Responsibility |
| --- | --- |
| `robotnik_isaac_description` | USD composition and robot metadata. |
| `robotnik_isaac_sensors` | Sensor profiles and ROS topic contracts. |
| `robotnik_isaac_ros2_control` | Robot-agnostic `ros2_control` bridge for Isaac JointState topics. |
| `robotnik_isaac_bringup` | Isaac Sim world/robot launch files. |
| `robotnik_isaac_benchmark` (`isaac_sim`) | Legacy RB-Watcher benchmark; not a platform bringup package. |

The first supported platform is `rbkairos`, followed by `rbkairos_plus` with
an UR10e arm.  The USD assets are the Isaac source of truth. Robotnik's xacro
and Gazebo files are reference material only.

See [docs/architecture.md](docs/architecture.md) for repository boundaries,
the transport contract, compatibility scope and the staged extraction policy.
