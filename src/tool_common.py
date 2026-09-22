"""Shared config, GLB and Gazebo-resource helpers for GZ_sim_tools."""

from __future__ import annotations

import json
import re
import struct
from pathlib import Path
from typing import Any

import yaml


MODEL_NAME = re.compile(r"^[A-Za-z0-9_-]+$")


def load_config(config_path: str | Path) -> tuple[Path, dict[str, Any]]:
    path = Path(config_path).expanduser().resolve()
    if not path.is_file():
        raise ValueError(f"configuration file does not exist: {path}")
    with path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError("configuration root must be a YAML mapping")
    root = path.parent.parent
    config.setdefault("paths", {})
    config.setdefault("gs_gz", {})
    config.setdefault("gz_pcd", {})
    return root, config


def relative_path(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else root / path


def model_name(config: dict[str, Any]) -> str:
    name = str(config.get("model_name", ""))
    if not MODEL_NAME.fullmatch(name):
        raise ValueError("model_name must contain only letters, digits, '_' or '-'")
    return name


def read_glb(path: Path) -> tuple[dict[str, Any], bytes]:
    with path.open("rb") as stream:
        magic, version, total_length = struct.unpack("<4sII", stream.read(12))
        if magic != b"glTF" or version != 2:
            raise ValueError(f"expected a glTF 2.0 GLB: {path}")
        chunks: list[tuple[bytes, bytes]] = []
        while stream.tell() < total_length:
            length, kind = struct.unpack("<I4s", stream.read(8))
            chunks.append((kind, stream.read(length)))
    try:
        document = json.loads(next(data for kind, data in chunks if kind == b"JSON").decode("utf-8"))
        binary = next(data for kind, data in chunks if kind == b"BIN\0")
    except StopIteration as error:
        raise ValueError(f"GLB is missing JSON or BIN chunk: {path}") from error
    return document, binary


def glb_bounds(document: dict[str, Any]) -> tuple[list[float], list[float]]:
    minimum = [float("inf")] * 3
    maximum = [float("-inf")] * 3
    positions: set[int] = set()
    for mesh in document.get("meshes", []):
        for primitive in mesh.get("primitives", []):
            accessor = primitive.get("attributes", {}).get("POSITION")
            if accessor is not None:
                positions.add(accessor)
    for accessor_index in positions:
        accessor = document["accessors"][accessor_index]
        if "min" not in accessor or "max" not in accessor:
            raise ValueError("GLB POSITION accessor has no min/max bounds")
        for index in range(3):
            minimum[index] = min(minimum[index], float(accessor["min"][index]))
            maximum[index] = max(maximum[index], float(accessor["max"][index]))
    if minimum[0] == float("inf"):
        raise ValueError("GLB contains no POSITION accessors")
    return minimum, maximum


def externalize_gltf(glb: Path, output_dir: Path) -> tuple[list[float], list[float], int]:
    """Write GLB collision asset plus external glTF/JPEG visual assets."""
    document, binary = read_glb(glb)
    lower, upper = glb_bounds(document)
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "map.glb").write_bytes(glb.read_bytes())
    texture_dir = output_dir / "textures"
    texture_dir.mkdir(exist_ok=True)
    for index, image in enumerate(document.get("images", [])):
        view = document["bufferViews"][image.pop("bufferView")]
        extension = ".png" if image.get("mimeType") == "image/png" else ".jpg"
        filename = image.get("name", f"texture_{index}") + extension
        offset = int(view.get("byteOffset", 0))
        (texture_dir / filename).write_bytes(binary[offset:offset + int(view["byteLength"])])
        image.pop("mimeType", None)
        image["uri"] = f"textures/{filename}"
    document["buffers"][0]["uri"] = "map.bin"
    document["buffers"][0]["byteLength"] = len(binary)
    document.setdefault("asset", {})["generator"] = "GZ_sim_tools external texture export"
    (output_dir / "map.bin").write_bytes(binary)
    (output_dir / "map.gltf").write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return lower, upper, len(document.get("images", []))


def gazebo_transform_bounds(lower: list[float], upper: list[float]) -> tuple[list[float], list[float], float]:
    """Map glTF (X,Y,Z; Y-up) bounds to SDF (X,-Z,Y-minY; Z-up)."""
    floor_offset = -lower[1]
    return [lower[0], -upper[2], 0.0], [upper[0], -lower[2], upper[1] + floor_offset], floor_offset
