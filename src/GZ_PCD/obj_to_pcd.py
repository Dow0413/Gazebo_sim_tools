#!/usr/bin/env python3
"""Sample a triangle-mesh surface into an ASCII PCD point cloud.

Companion to world_to_pcd.py: that tool only reads *inline* primitive
(box/cylinder/plane) models inside a .world file. It does not resolve
<include><uri>model://...</uri></include> references and does not sample
<mesh> geometry, so worlds whose geometry lives in a mesh (e.g. galileo_map)
come out empty. Run this script directly on the mesh file instead:

    python3 scripts/obj_to_pcd.py <mesh.(obj|stl|ply)> out.pcd \
        --resolution 0.05 --voxel-size 0.05

Requires: numpy, open3d.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def write_pcd(path: Path, points: np.ndarray) -> None:
    """Write an ASCII PCD with FIELDS x y z, matching world_to_pcd.py output."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as stream:
        stream.write("# .PCD v0.7 - Point Cloud Data file format\n")
        stream.write("VERSION 0.7\n")
        stream.write("FIELDS x y z\n")
        stream.write("SIZE 4 4 4\n")
        stream.write("TYPE F F F\n")
        stream.write("COUNT 1 1 1\n")
        stream.write(f"WIDTH {len(points)}\n")
        stream.write("HEIGHT 1\n")
        stream.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        stream.write(f"POINTS {len(points)}\n")
        stream.write("DATA ascii\n")
        np.savetxt(stream, points, fmt="%.6f")


def positive_float(text: str) -> float:
    value = float(text)
    if value <= 0.0:
        raise argparse.ArgumentTypeError("must be > 0")
    return value


def non_negative_float(text: str) -> float:
    value = float(text)
    if value < 0.0:
        raise argparse.ArgumentTypeError("must be >= 0")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert a mesh (.obj/.stl/.ply) surface to an ASCII PCD point cloud.",
    )
    parser.add_argument("mesh", type=Path, help="Input mesh file.")
    parser.add_argument("pcd", type=Path, help="Output .pcd path.")
    parser.add_argument(
        "-r",
        "--resolution",
        type=positive_float,
        default=0.05,
        help="Approximate surface sampling spacing in meters. Default: 0.05.",
    )
    parser.add_argument(
        "--voxel-size",
        type=non_negative_float,
        default=0.0,
        help="Optional voxel downsample size in meters. Default: disabled.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="Random seed for surface sampling (reproducible output).",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    mesh = o3d.io.read_triangle_mesh(str(args.mesh))
    if len(mesh.vertices) == 0 or len(mesh.triangles) == 0:
        parser.error(f"failed to read mesh (0 vertices/triangles): {args.mesh}")

    area = mesh.get_surface_area()
    target = max(1, int(round(area / args.resolution**2)))
    o3d.utility.random.seed(args.seed)
    cloud = mesh.sample_points_uniformly(number_of_points=target)

    sampled = len(cloud.points)
    if args.voxel_size > 0.0:
        cloud = cloud.voxel_down_sample(args.voxel_size)

    points = np.asarray(cloud.points, dtype=np.float64)
    write_pcd(args.pcd, points)

    lo, hi = points.min(axis=0), points.max(axis=0)
    print(
        f"mesh: {len(mesh.vertices)} vertices, {len(mesh.triangles)} triangles, "
        f"surface {area:.1f} m^2; sampled {sampled} points at ~{args.resolution} m spacing"
    )
    print(
        f"wrote {len(points)} points to {args.pcd}; "
        f"bounds x[{lo[0]:.2f}, {hi[0]:.2f}] y[{lo[1]:.2f}, {hi[1]:.2f}] z[{lo[2]:.2f}, {hi[2]:.2f}]"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
