#!/usr/bin/env python3
"""Run the configured world-or-mesh PCD conversion stage."""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_common import load_config, relative_path


def gazebo_transform_pcd(source: Path, target: Path, floor_offset: float) -> None:
    with source.open(encoding="ascii") as input_file, target.open("w", encoding="ascii") as output_file:
        for line in input_file:
            output_file.write(line)
            if line.strip().lower() == "data ascii":
                break
        for line in input_file:
            x, y, z = (float(value) for value in line.split())
            output_file.write(f"{x:.6f} {-z:.6f} {y + floor_offset:.6f}\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=Path(__file__).resolve().parents[2] / "config/default.yaml")
    args = parser.parse_args()
    root, config = load_config(args.config)
    settings = config.get("to_pcd", {})
    if not isinstance(settings, dict):
        raise SystemExit("to_pcd must be a YAML mapping")
    input_value = str(settings.get("input_file", "")).strip()
    if not input_value:
        world_settings = config.get("glb_to_world", {})
        if not isinstance(world_settings, dict):
            raise SystemExit("to_pcd.input_file is empty and glb_to_world is not a YAML mapping")
        input_value = str(world_settings.get("input_glb", "")).strip()
    input_path = relative_path(root, input_value)
    if not input_path.is_file():
        raise SystemExit(f"to_pcd.input_file does not exist: {input_path}")
    output = relative_path(root, str(config["paths"].get("pcd_dir", "pcd"))) / str(settings.get("output_file", ""))
    if not output.name:
        raise SystemExit("to_pcd.output_file must not be empty")
    resolution = str(settings.get("resolution", 0.05))
    voxel = str(settings.get("voxel_size", 0.0))
    kind = str(settings.get("input_type", "mesh")).lower()
    here = Path(__file__).resolve().parent
    output.parent.mkdir(parents=True, exist_ok=True)
    if kind == "world":
        subprocess.run([sys.executable, str(here / "world_to_pcd.py"), str(input_path), str(output), "--resolution", resolution, "--voxel-size", voxel], check=True)
    elif kind == "mesh":
        if bool(settings.get("gazebo_coordinates", True)):
            with tempfile.TemporaryDirectory(prefix="gz_sim_tools_") as temp:
                raw = Path(temp) / "raw.pcd"
                subprocess.run([sys.executable, str(here / "obj_to_pcd.py"), str(input_path), str(raw), "--resolution", resolution, "--voxel-size", voxel, "--seed", "0"], check=True)
                import open3d as o3d
                mesh = o3d.io.read_triangle_mesh(str(input_path))
                gazebo_transform_pcd(raw, output, -float(mesh.get_min_bound()[1]))
        else:
            subprocess.run([sys.executable, str(here / "obj_to_pcd.py"), str(input_path), str(output), "--resolution", resolution, "--voxel-size", voxel, "--seed", "0"], check=True)
    else:
        raise SystemExit("to_pcd.input_type must be mesh or world")
    print(f"PCD written: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
