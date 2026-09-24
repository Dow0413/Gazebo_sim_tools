# GZ_sim_tools

只有两个步骤，配置都在 `config/default.yaml`。

1. **GLB → world**：`GS_GZ` 将 GLB 变为 Gazebo world，输出到 `maps/<map_name>/`。
2. **world 或 mesh → PCD**：`GZ_PCD` 将基础 SDF world 或 OBJ/STL/PLY/GLB 网格转为 PCD，输出到 `pcd/`。

## 使用

将 GLB 放入 `glb/`，然后编辑：

- `glb_to_world.input_glb` 与 `glb_to_world.map_name`
- `to_pcd.input_type`（`world` 或 `mesh`）、`to_pcd.input_file`、`to_pcd.output_file`。当 `input_file` 为空时，自动复用 `glb_to_world.input_glb`。

```bash
# 按 mode 执行：all | glb_to_world | to_pcd
GZ_sim_tools/shells/run_all.sh

# 单独执行两个步骤
GZ_sim_tools/shells/run_gs_gz.sh
GZ_sim_tools/shells/run_gz_pcd.sh
```

`world_to_pcd.py` 与 `obj_to_pcd.py` 均来自 `world2pcd-main`：前者只支持 world 内的 box、cylinder、plane，后者处理网格。对于 GLB 网格，`to_pcd.gazebo_coordinates: true` 会将 glTF Y-up 点云转换为 Gazebo Z-up，使它与生成的 Gazebo world 重合。

生成的 world 可独立查看：

```bash
GZ_SIM_RESOURCE_PATH="$PWD/GZ_sim_tools/maps" \
  gz sim "$PWD/GZ_sim_tools/maps/<map_name>/<map_name>.world"
```
