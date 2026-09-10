"""Offline URDF/xacro-to-USD importer for RB-KAIROS assets.

Run this file with ``$ISAACSIM_PATH/python.sh``. It is the only component that
reads Robotnik reference packages; the produced USD is self-contained.
"""

import argparse
import os
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--model', choices=('rbkairos', 'rbkairos_plus'), required=True)
    return parser.parse_args()


def main():
    args = parse_arguments()
    from isaacsim import SimulationApp

    app = SimulationApp({'headless': True, 'sync_loads': True})
    try:
        description = args.workspace / 'src/robotnik/robotnik_description'
        robot = description / 'robots/rbkairos' / f'{args.model}.urdf.xacro'
        if not robot.is_file():
            raise FileNotFoundError(robot)
        urdf = args.output / f'{args.model}.urdf'
        args.output.mkdir(parents=True, exist_ok=True)
        # Isaac's python.sh intentionally starts with a minimal ROS/ament
        # environment.  xacro uses the ament index to resolve the *offline*
        # Robotnik source descriptions, so restore this workspace's install
        # prefix only for that subprocess.
        xacro_env = os.environ.copy()
        # This workspace uses isolated colcon installs, hence each package
        # contributes its own ament prefix (the aggregate install directory
        # alone does not contain resource_index entries).
        install_prefixes = [
            str(args.workspace / 'install' / 'robotnik_description'),
            str(args.workspace / 'install' / 'robotnik_sensors'),
            str(args.workspace / 'install'),
        ]
        current_prefixes = xacro_env.get('AMENT_PREFIX_PATH', '')
        xacro_env['AMENT_PREFIX_PATH'] = ':'.join(
            prefix for prefix in (*install_prefixes, current_prefixes) if prefix)
        subprocess.run([
            'xacro', str(robot), 'namespace:=robot', 'prefix:=robot_',
            'gazebo_ignition:=false', 'ur_type:=ur10e', '-o', str(urdf),
        ], check=True, env=xacro_env)

        # The Isaac 6 URDF importer does not query the ament index when it
        # sees package:// URIs.  Resolve every mesh before import so the USD
        # contains actual mesh prims rather than empty visual Xforms.
        package_roots = {
            'robotnik_description': description,
            'robotnik_sensors': args.workspace / 'src/robotnik/robotnik_sensors',
            'ur_description': Path('/opt/ros/jazzy/share/ur_description'),
        }
        expanded_urdf = urdf.read_text()
        for package, root in package_roots.items():
            if not root.is_dir():
                raise FileNotFoundError(f'Mesh package is unavailable: {root}')
            expanded_urdf = expanded_urdf.replace(
                f'package://{package}/', f'{root}/')
        # This asset is a render-only layer.  Strip collision elements before
        # importing: high-poly STL/DAE triangle meshes must never become
        # children of the dynamic proxy bodies created at runtime.
        urdf_root = ET.fromstring(expanded_urdf)
        for link in urdf_root.findall('link'):
            for collision in link.findall('collision'):
                link.remove(collision)
        urdf.write_text(ET.tostring(urdf_root, encoding='unicode'))
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

        config = URDFImporterConfig(
            urdf_path=str(urdf), usd_path=str(args.output), fix_base=False,
            collision_from_visuals=False, collision_type='Convex Decomposition',
            allow_self_collision=False, merge_fixed_joints=False,
            joint_drive_type='force', joint_target_type='velocity',
            run_asset_transformer=True,
        )
        output = URDFImporter(config).import_urdf()
        print(f'Imported {args.model} to {output}')
    finally:
        app.close()


if __name__ == '__main__':
    main()
