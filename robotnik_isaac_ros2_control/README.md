# robotnik_isaac_ros2_control

Robot-agnostic `ros2_control` `SystemInterface` for the native Isaac Sim ROS 2
Joint State nodes. The package has no dependency on Robotnik descriptions,
controllers, MoveIt, USD or Isaac Python modules and can be built without
Isaac Sim.

## Transport contract

The joint list and its order come exclusively from the joints declared in the
URDF `<ros2_control>` block. Every joint may independently expose `position`,
`velocity` and/or `effort` state and command interfaces. Other interfaces fail
initialization with an actionable error.

The state subscriber accepts partial `sensor_msgs/msg/JointState` messages.
Unknown joints are ignored, the first occurrence of a duplicate wins, absent
fields retain their last valid value, and non-finite state values are ignored
by default. Initial values are those declared by ros2_control in the URDF.

The command publisher preserves declared joint order. It emits an array for
each interface used by at least one joint and writes IEEE NaN at indices where
that joint does not expose that interface. This is the native mixed-command
convention consumed by Isaac Sim 6's `ROS2SubscribeJointState` connected to
`IsaacArticulationController`. Thus Kairos wheels can use velocity while its
UR10e uses position in one message. No custom message is introduced: doing so
would require a second Isaac graph adapter and would lose compatibility with
the stock ROS 2 bridge nodes.

Relative topic names are resolved in the hardware component/controller-manager
namespace. Absolute names bypass that namespace. For multiple robots, use
relative topics and one namespaced controller manager per robot.

## Hardware parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `state_topic` | `isaac/joint_states` | Incoming JointState topic. |
| `command_topic` | `isaac/joint_command` | Outgoing JointState topic. |
| `qos_depth` | `5` | Keep-last depth for both endpoints. |
| `state_qos_reliability` | `best_effort` | Reliability for incoming states. |
| `command_qos_reliability` | `reliable` | Reliability for outgoing commands. |
| `qos_reliability` | unset | Optional shorthand overriding both directions. |
| `state_timeout` | `0.0` | Watchdog seconds; zero disables it for backward compatibility. |
| `require_all_joints` | `false` | Return read error until every declared state field was received. |
| `reject_nonfinite_state` | `true` | Ignore NaN/Inf state samples. |
| `enforce_command_limits` | `true` | Clamp finite commands to URDF interface min/max. |

`header.stamp` on commands uses the controller update time. State freshness is
measured on receipt with the hardware node clock, so `use_sim_time` follows the
controller manager. A non-zero watchdog starts after the first valid sample;
set `require_all_joints=true` when startup must also be fail-fast.

Configure creates handles and ROS endpoints, activate permits publication,
deactivate stops it, and cleanup/error release endpoints and received-state
flags. Callback data and the control loop are protected by a mutex. Publishing
is performed from `write()`, never from the subscription callback.

## Scope and extraction

Kairos integration URDFs live in `robotnik_isaac_description/config`; this
package does not install robot-specific files. Before extracting this directory
to its own repository, copy the repository Apache-2.0 `LICENSE`, enable the CI
described in `../docs/architecture.md`, tag a release, and make the integration
repository consume that immutable tag or binary package.
