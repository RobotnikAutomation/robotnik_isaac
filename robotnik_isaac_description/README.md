# Robotnik Isaac description

This package owns the Isaac-native USD composition roots for RB-KAIROS and
RB-KAIROS+. It intentionally contains neither ROS launch code nor Gazebo
plugins. The asset contract is:

- `assets/rbkairos/usd/rbkairos_base/rbkairos_base.usda`: imported mecanum base.
- `assets/rbkairos/usd/rbkairos_plus/rbkairos_plus.usda`: imported mobile UR10e articulation.
- `config/*.yaml`: canonical wheel and arm joint names used by control.

The initial USD files are collision/kinematic composition roots. Visual mesh,
wheel articulation and UR10e importer output must be authored in sibling USD
layers so that their updates do not change the contract.
