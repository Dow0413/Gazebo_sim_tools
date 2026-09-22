#!/usr/bin/env python3
"""Sample the configured GLB and write a Gazebo-coordinate ASCII PCD map."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_common import load_config, model_name, relative_path


def write_pcd(path: Path, points: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA ascii\n")
        np.savetxt(stream, points, fmt="%.6f")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=Path(__file__).resolve().parents[2] / "config/default.yaml")
    args = parser.parse_args()
    root, config = load_config(args.config)
    paths, settings = config["paths"], config["gz_pcd"]
    glb = relative_path(root, str(paths.get("glb_dir", "glb"))) / str(config.get("input_glb", ""))
    if not glb.is_file():
        raise SystemExit(f"input_glb does not exist: {glb}")
    output = relative_path(root, str(paths.get("pcd_dir", "pcd"))) / str(settings.get("output_pcd", f"{model_name(config)}.pcd"))
    mesh = o3d.io.read_triangle_mesh(str(glb))
    if not mesh.has_triangles():
        raise SystemExit(f"failed to load a triangle mesh from {glb}")
    resolution = float(settings.get("resolution", 0.05))
    voxel_size = float(settings.get("voxel_size", resolution))
    if resolution <= 0 or voxel_size < 0:
        raise SystemExit("resolution must be > 0 and voxel_size must be >= 0")
    o3d.utility.random.seed(int(settings.get("seed", 0)))
    cloud = mesh.sample_points_uniformly(number_of_points=max(1, round(mesh.get_surface_area() / resolution**2)))
    if voxel_size:
        cloud = cloud.voxel_down_sample(voxel_size)
    points = np.asarray(cloud.points, dtype=np.float64)
    floor_offset = -float(mesh.get_min_bound()[1])
    points = np.column_stack((points[:, 0], -points[:, 2], points[:, 1] + floor_offset))
    write_pcd(output, points)
    lower, upper = points.min(axis=0), points.max(axis=0)
    print(f"wrote {len(points)} points: {output}")
    print(f"Gazebo bounds: x[{lower[0]:.2f}, {upper[0]:.2f}] y[{lower[1]:.2f}, {upper[1]:.2f}] z[{lower[2]:.2f}, {upper[2]:.2f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
