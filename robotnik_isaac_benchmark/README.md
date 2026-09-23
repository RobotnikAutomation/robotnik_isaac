# Robotnik Isaac Sim Benchmark

> **Work in Progress**
>
> This package is currently maintained for evaluation and benchmarking
> purposes. It is not yet the general-purpose Robotnik Isaac Sim bringup.

## Purpose

`robotnik_isaac_benchmark` provides the launch files, runtime scripts, USD
worlds and RB-Watcher assets used to evaluate Isaac Sim alongside the other
simulation backends in the Robotnik benchmarking system.

The package currently supports:

- GUI and headless execution;
- one, two or three RB-Watcher robots;
- configurable worlds;
- deterministic GPU selection;
- a fixed 50 Hz simulation profile;
- optional central RViz execution.

The benchmark runner launches this package through:

```bash
ros2 launch isaac_sim isaac_sim_complete.launch.py
```

The broader Isaac packages in this repository (`robotnik_isaac_bringup`,
`robotnik_isaac_description` and related packages) are developed separately
and are not required to be treated as part of this legacy benchmark package.

## Scope

This package is intended to compare Isaac Sim performance and sensor behaviour
with Gazebo, Webots, O3DE, MuJoCo and Unity. Interfaces and assets may change
while the benchmark contract is being consolidated.
