#!/usr/bin/env python3
"""Convert a GLB in GZ_sim_tools/glb into a Gazebo model/world."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tool_common import externalize_gltf, gazebo_transform_bounds, load_config, model_name, relative_path


def ensure_empty(path: Path, overwrite: bool) -> None:
    if path.exists():
        if not overwrite:
            raise ValueError(f"output already exists: {path}; set gs_gz.overwrite: true to replace it")
        shutil.rmtree(path) if path.is_dir() else path.unlink()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=Path(__file__).resolve().parents[2] / "config/default.yaml")
    args = parser.parse_args()
    root, config = load_config(args.config)
    name = model_name(config)
    paths = config["paths"]
    glb = relative_path(root, str(paths.get("glb_dir", "glb"))) / str(config.get("input_glb", ""))
    if not glb.is_file():
        raise SystemExit(f"input_glb does not exist: {glb}")
    maps_dir = relative_path(root, str(paths.get("maps_dir", "maps")))
    settings = config["gs_gz"]
    overwrite = bool(settings.get("overwrite", False))
    generated_model = maps_dir / name
    ensure_empty(generated_model, overwrite)
    generated_model.mkdir(parents=True)
    mesh_dir = generated_model / "meshes"
    lower, upper, textures = externalize_gltf(glb, mesh_dir)
    floor_offset = -lower[1]
    (generated_model / "model.config").write_text(f'''<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.6">model.sdf</sdf>
<description>Generated from {glb.name}; {textures} external texture(s), GLB collision mesh.</description></model>
''', encoding="utf-8")
    (generated_model / "model.sdf").write_text(f'''<?xml version="1.0" ?>
<sdf version="1.6"><model name="{name}"><static>true</static><link name="map_link">
<visual name="visual"><pose>0 0 {floor_offset:.8f} 1.5707963 0 0</pose><geometry><mesh><uri>model://{name}/meshes/map.gltf</uri></mesh></geometry></visual>
<collision name="collision"><pose>0 0 {floor_offset:.8f} 1.5707963 0 0</pose><geometry><mesh><uri>model://{name}/meshes/map.glb</uri></mesh></geometry></collision>
</link></model></sdf>
''', encoding="utf-8")
    generated_world = generated_model / f"{name}.world"
    generated_world.write_text(f'''<?xml version="1.0" ?>
<sdf version="1.6"><world name="{name}">
<light name="sun" type="directional"><pose>0 0 10 0 0 0</pose><cast_shadows>true</cast_shadows><intensity>1</intensity><direction>-0.5 0.1 -0.9</direction></light>
<physics name="ode_1khz" type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor><real_time_update_rate>1000</real_time_update_rate></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/><plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/><plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
<include><uri>model://{name}</uri></include></world></sdf>
''', encoding="utf-8")
    gz_lower, gz_upper, _ = gazebo_transform_bounds(lower, upper)
    print(f"generated: {generated_model} (including {generated_world.name})")
    print(f"textures: {textures}; Gazebo bounds: x[{gz_lower[0]:.2f}, {gz_upper[0]:.2f}] y[{gz_lower[1]:.2f}, {gz_upper[1]:.2f}] z[{gz_lower[2]:.2f}, {gz_upper[2]:.2f}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
