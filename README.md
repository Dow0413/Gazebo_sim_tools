# GZ_sim_tools

将 GLB 地图转换为 Gazebo Sim 资源，并生成与 Gazebo 坐标系重合的 PCD。

## 使用

1. 将一个 `.glb` 文件放到 `glb/`。
2. 编辑 `config/default.yaml` 中的 `input_glb`、`model_name` 和 PCD 参数。
3. 在工作区根目录运行：

```bash
GZ_sim_tools/shells/run_all.sh
```

只生成 Gazebo 资源：`GZ_sim_tools/shells/run_gs_gz.sh`。

只生成 PCD：`GZ_sim_tools/shells/run_gz_pcd.sh`。

每个脚本都可接收自定义配置路径作为第一个参数。

## 输出

- `maps/<model_name>/`：一张地图的全部 Gazebo 资源：`model.sdf`、`model.config`、外部 glTF/JPEG 视觉资源、GLB 碰撞网格和 `<model_name>.world`。
- `pcd/<output_pcd>`：已转换为 Gazebo Z-up 坐标的 ASCII PCD。

工具不会修改或构建 `galileo_nav_sim`。要单独查看生成的地图，可让 Gazebo 搜索 `maps/`：

```bash
GZ_SIM_RESOURCE_PATH="$PWD/GZ_sim_tools/maps" \
  gz sim "$PWD/GZ_sim_tools/maps/<model_name>/<model_name>.world"
```

首次生成时保持 `gs_gz.overwrite: false`。需要重新生成同名地图时，显式将它设为 `true`。
