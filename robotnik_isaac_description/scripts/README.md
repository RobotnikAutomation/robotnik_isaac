# Import scripts

Asset conversion is intentionally an offline step.  It must be run with the
target Isaac Sim release and must produce only USD layers under `assets/`.
Runtime launch files never read xacro, URDF, Gazebo SDF or the Robotnik source
packages.

The importer will be added with the validated mesh and UR10e asset paths. Its
acceptance checks are the prim/joint names declared in `config/`.
