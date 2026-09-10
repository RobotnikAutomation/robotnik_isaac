#!/usr/bin/env python3
"""Generate portable, xacro-free URDF assets for Isaac Sim."""

import argparse
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path


MODELS = ("rbkairos_base", "ur10e", "rbkairos_plus")


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def xacro_environment(workspace: Path) -> dict[str, str]:
    environment = os.environ.copy()
    prefixes = (workspace / "install" / "robotnik_description", workspace / "install" / "robotnik_sensors", workspace / "install")
    inherited = environment.get("AMENT_PREFIX_PATH", "")
    available = [str(prefix) for prefix in prefixes if prefix.is_dir()]
    environment["AMENT_PREFIX_PATH"] = ":".join(available + ([inherited] if inherited else []))
    return environment


def source_mesh(uri: str, package_roots: dict[str, Path]) -> tuple[str, Path, Path]:
    for package, root in package_roots.items():
        marker = f"package://{package}/"
        if uri.startswith(marker):
            relative = Path(uri.removeprefix(marker))
            return package, root / relative, relative
    path = Path(uri.removeprefix("file://"))
    return "external", path, Path(path.name)


def main():
    args = arguments()
    package_root = Path(__file__).resolve().parents[1]
    output = args.output or package_root / "assets" / "rbkairos" / "urdf"
    output.mkdir(parents=True, exist_ok=True)
    package_roots = {
        "robotnik_description": args.workspace / "src/robotnik/robotnik_description",
        "robotnik_sensors": args.workspace / "src/robotnik/robotnik_sensors",
        "ur_description": Path("/opt/ros/jazzy/share/ur_description"),
    }
    for name, root in package_roots.items():
        if not root.is_dir():
            raise FileNotFoundError(f"Required source package is unavailable: {name}: {root}")

    for model in MODELS:
        destination = output / f"{model}.urdf"
        subprocess.run(["xacro", str(package_root / "urdf_templates" / f"{model}.urdf.xacro"), "-o", str(destination)], check=True, env=xacro_environment(args.workspace))
        document = ET.parse(destination)
        for mesh in document.findall(".//mesh"):
            filename = mesh.get("filename")
            if not filename:
                raise ValueError(f"Mesh without filename in {destination}")
            package, source, relative = source_mesh(filename, package_roots)
            if not source.is_file():
                raise FileNotFoundError(f"Referenced mesh does not exist: {source}")
            copied = output / "meshes" / package / relative
            copied.parent.mkdir(parents=True, exist_ok=True)
            if not copied.exists() or copied.stat().st_mtime_ns < source.stat().st_mtime_ns:
                shutil.copy2(source, copied)
            mesh.set("filename", copied.relative_to(output).as_posix())
        ET.indent(document, space="  ")
        document.write(destination, encoding="utf-8", xml_declaration=True)
        print(f"Generated {destination}")


if __name__ == "__main__":
    main()
