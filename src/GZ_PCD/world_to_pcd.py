#!/usr/bin/env python3
"""Sample primitive SDF/Gazebo world geometry into an ASCII PCD map.

The converter is intentionally dependency-free and targets hand-authored
worlds made from static primitive collision geometry. It supports box,
cylinder, and finite plane geometry.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import DefaultDict, Iterable, Iterator, List, Optional, Sequence, Tuple


Vector = Tuple[float, float, float]
Matrix3 = Tuple[Vector, Vector, Vector]
Bounds = Tuple[float, float, float, float, float, float]


def local_name(tag: str) -> str:
    if "}" in tag:
        return tag.rsplit("}", 1)[1]
    return tag


def find_child(element: ET.Element, name: str) -> Optional[ET.Element]:
    for child in element:
        if local_name(child.tag) == name:
            return child
    return None


def find_children(element: ET.Element, name: str) -> List[ET.Element]:
    return [child for child in element if local_name(child.tag) == name]


def parse_float_list(text: Optional[str]) -> List[float]:
    if not text:
        return []
    return [float(part) for part in text.split()]


def parse_vector(element: Optional[ET.Element], count: int, default: Sequence[float]) -> List[float]:
    values = parse_float_list(element.text if element is not None else None)
    if not values:
        values = list(default)
    if len(values) != count:
        raise ValueError(f"expected {count} values, got {len(values)} in {ET.tostring(element, encoding='unicode')}")
    return values


def mat_mul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )  # type: ignore[return-value]


def mat_vec_mul(matrix: Matrix3, vector: Vector) -> Vector:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


def vec_add(a: Vector, b: Vector) -> Vector:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def cross(a: Vector, b: Vector) -> Vector:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def scale(vector: Vector, factor: float) -> Vector:
    return (vector[0] * factor, vector[1] * factor, vector[2] * factor)


def norm(vector: Vector) -> float:
    return math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)


def normalize(vector: Vector) -> Vector:
    length = norm(vector)
    if length <= 0.0:
        raise ValueError("cannot normalize a zero-length vector")
    return (vector[0] / length, vector[1] / length, vector[2] / length)


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> Matrix3:
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    # SDF poses use roll/pitch/yaw in radians; this is Rz(yaw) * Ry(pitch) * Rx(roll).
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


@dataclass(frozen=True)
class Transform:
    rotation: Matrix3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    translation: Vector = (0.0, 0.0, 0.0)

    @classmethod
    def from_pose(cls, values: Sequence[float]) -> "Transform":
        x, y, z, roll, pitch, yaw = values
        return cls(rotation_from_rpy(roll, pitch, yaw), (x, y, z))

    def compose(self, child: "Transform") -> "Transform":
        return Transform(
            mat_mul(self.rotation, child.rotation),
            vec_add(mat_vec_mul(self.rotation, child.translation), self.translation),
        )

    def apply(self, point: Vector) -> Vector:
        return vec_add(mat_vec_mul(self.rotation, point), self.translation)


def parse_pose(element: ET.Element) -> Transform:
    pose = find_child(element, "pose")
    values = parse_float_list(pose.text if pose is not None else None)
    values = values + [0.0] * (6 - len(values))
    if len(values) != 6:
        raise ValueError(f"pose must contain up to 6 values, got {len(values)}")
    return Transform.from_pose(values)


@dataclass
class Stats:
    models_seen: int = 0
    models_used: int = 0
    primitives: int = 0
    unsupported: DefaultDict[str, int] = field(default_factory=lambda: defaultdict(int))
    warnings: List[str] = field(default_factory=list)


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


def span_values(length: float, resolution: float) -> List[float]:
    if length < 0.0:
        raise ValueError(f"length must be non-negative, got {length}")
    if length == 0.0:
        return [0.0]
    intervals = max(1, int(math.ceil(length / resolution)))
    step = length / intervals
    start = -length / 2.0
    return [start + i * step for i in range(intervals + 1)]


def sample_box(size: Sequence[float], resolution: float) -> Iterator[Vector]:
    sx, sy, sz = size
    xs = span_values(sx, resolution)
    ys = span_values(sy, resolution)
    zs = span_values(sz, resolution)
    hx = sx / 2.0
    hy = sy / 2.0
    hz = sz / 2.0

    for x in (-hx, hx):
        for y in ys:
            for z in zs:
                yield (x, y, z)
    for y in (-hy, hy):
        for x in xs:
            for z in zs:
                yield (x, y, z)
    for z in (-hz, hz):
        for x in xs:
            for y in ys:
                yield (x, y, z)


def sample_cylinder(radius: float, length: float, resolution: float) -> Iterator[Vector]:
    if radius <= 0.0 or length <= 0.0:
        return

    theta_count = max(12, int(math.ceil(2.0 * math.pi * radius / resolution)))
    zs = span_values(length, resolution)
    for index in range(theta_count):
        theta = 2.0 * math.pi * index / theta_count
        x = radius * math.cos(theta)
        y = radius * math.sin(theta)
        for z in zs:
            yield (x, y, z)

    radial_intervals = max(1, int(math.ceil(radius / resolution)))
    for z in (-length / 2.0, length / 2.0):
        yield (0.0, 0.0, z)
        for ring in range(1, radial_intervals + 1):
            ring_radius = radius * ring / radial_intervals
            ring_count = max(6, int(math.ceil(2.0 * math.pi * ring_radius / resolution)))
            for index in range(ring_count):
                theta = 2.0 * math.pi * index / ring_count
                yield (ring_radius * math.cos(theta), ring_radius * math.sin(theta), z)


def plane_basis(normal: Vector) -> Tuple[Vector, Vector]:
    unit_normal = normalize(normal)
    if abs(unit_normal[0]) < 1e-9 and abs(unit_normal[1]) < 1e-9:
        return (1.0, 0.0, 0.0), (0.0, 1.0 if unit_normal[2] >= 0 else -1.0, 0.0)

    seed: Vector = (0.0, 0.0, 1.0)
    u = normalize(cross(seed, unit_normal))
    v = cross(unit_normal, u)
    return u, v


def sample_plane(
    size: Sequence[float],
    normal: Sequence[float],
    resolution: float,
    max_plane_size: float,
    stats: Stats,
) -> Iterator[Vector]:
    width, depth = size
    if max_plane_size > 0.0:
        clipped_width = min(width, max_plane_size)
        clipped_depth = min(depth, max_plane_size)
        if clipped_width != width or clipped_depth != depth:
            stats.warnings.append(
                f"clipped a {width:g} x {depth:g} plane to {clipped_width:g} x {clipped_depth:g}; "
                "use --max-plane-size 0 for the full plane"
            )
        width, depth = clipped_width, clipped_depth

    u, v = plane_basis((normal[0], normal[1], normal[2]))
    for a in span_values(width, resolution):
        for b in span_values(depth, resolution):
            yield vec_add(scale(u, a), scale(v, b))


def geometry_points(
    geometry: ET.Element,
    resolution: float,
    max_plane_size: float,
    stats: Stats,
) -> Iterator[Vector]:
    box = find_child(geometry, "box")
    if box is not None:
        size = parse_vector(find_child(box, "size"), 3, (1.0, 1.0, 1.0))
        yield from sample_box(size, resolution)
        return

    cylinder = find_child(geometry, "cylinder")
    if cylinder is not None:
        radius_values = parse_vector(find_child(cylinder, "radius"), 1, (1.0,))
        length_values = parse_vector(find_child(cylinder, "length"), 1, (1.0,))
        yield from sample_cylinder(radius_values[0], length_values[0], resolution)
        return

    plane = find_child(geometry, "plane")
    if plane is not None:
        size = parse_vector(find_child(plane, "size"), 2, (1.0, 1.0))
        normal = parse_vector(find_child(plane, "normal"), 3, (0.0, 0.0, 1.0))
        yield from sample_plane(size, normal, resolution, max_plane_size, stats)
        return

    kind = "empty"
    for child in geometry:
        kind = local_name(child.tag)
        break
    stats.unsupported[kind] += 1


def model_is_static(model: ET.Element) -> bool:
    static_element = find_child(model, "static")
    if static_element is None or static_element.text is None:
        return False
    return static_element.text.strip().lower() == "true"


def source_names(source: str) -> Tuple[str, ...]:
    if source == "both":
        return ("collision", "visual")
    return (source,)


def in_bounds(point: Vector, bounds: Optional[Bounds]) -> bool:
    if bounds is None:
        return True
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    x, y, z = point
    return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax


def iter_model_points(
    model: ET.Element,
    parent_transform: Transform,
    args: argparse.Namespace,
    stats: Stats,
    parent_static: bool = False,
) -> Iterator[Vector]:
    stats.models_seen += 1
    is_static = parent_static or model_is_static(model)
    if not is_static and not args.include_non_static:
        return

    stats.models_used += 1
    model_transform = parent_transform.compose(parse_pose(model))

    for link in find_children(model, "link"):
        link_transform = model_transform.compose(parse_pose(link))
        for name in source_names(args.source):
            for item in find_children(link, name):
                geometry = find_child(item, "geometry")
                if geometry is None:
                    continue
                item_transform = link_transform.compose(parse_pose(item))
                stats.primitives += 1
                for local_point in geometry_points(geometry, args.resolution, args.max_plane_size, stats):
                    world_point = item_transform.apply(local_point)
                    if in_bounds(world_point, args.bounds):
                        yield world_point

    for nested_model in find_children(model, "model"):
        yield from iter_model_points(nested_model, model_transform, args, stats, is_static)


def collect_points(root: ET.Element, args: argparse.Namespace, stats: Stats) -> List[Vector]:
    if local_name(root.tag) == "world":
        worlds = [root]
    else:
        worlds = find_children(root, "world")

    points: List[Vector] = []
    for world in worlds:
        world_transform = parse_pose(world)
        for model in find_children(world, "model"):
            points.extend(iter_model_points(model, world_transform, args, stats))
    return points


def voxel_filter(points: Iterable[Vector], voxel_size: float) -> List[Vector]:
    if voxel_size <= 0.0:
        return list(points)

    filtered = {}
    for point in points:
        key = (
            math.floor(point[0] / voxel_size),
            math.floor(point[1] / voxel_size),
            math.floor(point[2] / voxel_size),
        )
        if key not in filtered:
            filtered[key] = point
    return list(filtered.values())


def write_pcd(path: Path, points: Sequence[Vector]) -> None:
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
        for x, y, z in points:
            stream.write(f"{x:.6f} {y:.6f} {z:.6f}\n")


def parse_bounds(values: Optional[Sequence[float]]) -> Optional[Bounds]:
    if values is None:
        return None
    xmin, xmax, ymin, ymax, zmin, zmax = values
    if xmin > xmax or ymin > ymax or zmin > zmax:
        raise argparse.ArgumentTypeError("bounds must be ordered as min max min max min max")
    return (xmin, xmax, ymin, ymax, zmin, zmax)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert static primitive geometry in an SDF/Gazebo .world file to an ASCII PCD point cloud."
    )
    parser.add_argument("world", type=Path, help="Input .world or .sdf file.")
    parser.add_argument("pcd", type=Path, help="Output .pcd path.")
    parser.add_argument(
        "-r",
        "--resolution",
        type=positive_float,
        default=0.10,
        help="Approximate surface sampling spacing in meters. Default: 0.10.",
    )
    parser.add_argument(
        "--source",
        choices=("collision", "visual", "both"),
        default="collision",
        help="Geometry source to sample. Default: collision.",
    )
    parser.add_argument(
        "--include-non-static",
        action="store_true",
        help="Include models that do not have <static>true</static>.",
    )
    parser.add_argument(
        "--bounds",
        nargs=6,
        type=float,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
        help="Keep only points inside this world-frame box.",
    )
    parser.add_argument(
        "--max-plane-size",
        type=non_negative_float,
        default=40.0,
        help="Crop each SDF plane side to this many meters before sampling. Use 0 for full plane. Default: 40.",
    )
    parser.add_argument(
        "--voxel-size",
        type=non_negative_float,
        default=0.0,
        help="Optional voxel downsample size in meters. Default: disabled.",
    )
    parser.add_argument("--quiet", action="store_true", help="Only print errors.")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        args.bounds = parse_bounds(args.bounds)
        root = ET.parse(args.world).getroot()
        stats = Stats()
        points = collect_points(root, args, stats)
        points = voxel_filter(points, args.voxel_size)
        write_pcd(args.pcd, points)
    except (ET.ParseError, OSError, ValueError, argparse.ArgumentTypeError) as exc:
        parser.error(str(exc))

    if not args.quiet:
        print(
            f"wrote {len(points)} points from {stats.primitives} primitive geometries "
            f"({stats.models_used}/{stats.models_seen} models) to {args.pcd}"
        )
        if stats.unsupported:
            details = ", ".join(f"{name}: {count}" for name, count in sorted(stats.unsupported.items()))
            print(f"unsupported geometries skipped: {details}", file=sys.stderr)
        for warning in sorted(set(stats.warnings)):
            print(f"warning: {warning}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
