# Robotnik Isaac description

This package owns the Isaac-native USD composition roots for RB-KAIROS and
RB-KAIROS+. It intentionally contains neither ROS launch code nor Gazebo
plugins. The asset contract is:

- `assets/rbkairos/usd/rbkairos_base/rbkairos_base.usda`: imported mecanum base.
- `assets/rbkairos/usd/rbkairos_plus/rbkairos_plus.usda`: imported mobile UR10e articulation.
- `config/*.yaml`: registered model assets, runtime drive joints, topics and home targets.
- `config/*.ros2_control.urdf`: Kairos integration declarations consumed by
  controller manager. These are robot-specific adapters, not part of the
  generic bridge.

The initial USD files are collision/kinematic composition roots. Visual mesh,
wheel articulation and UR10e importer output must be authored in sibling USD
layers so that their updates do not change the contract.

The package depends at runtime on `robotnik_isaac_ros2_control` because its
control URDFs instantiate that plugin. The plugin has no reverse dependency.
