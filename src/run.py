#!/usr/bin/env python3
"""Run the stage(s) selected by ``mode`` in a GZ_sim_tools YAML config."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from tool_common import load_config


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=Path(__file__).resolve().parents[1] / "config/default.yaml")
    args = parser.parse_args()
    _, config = load_config(args.config)
    mode = str(config.get("mode", "both")).lower()
    root = Path(__file__).resolve().parent
    stages = {
        "gs_gz": [root / "GS_GZ/glb_to_gazebo.py"],
        "gz_pcd": [root / "GZ_PCD/gazebo_to_pcd.py"],
        "both": [root / "GS_GZ/glb_to_gazebo.py", root / "GZ_PCD/gazebo_to_pcd.py"],
    }
    if mode not in stages:
        raise SystemExit("mode must be gs_gz, gz_pcd, or both")
    for stage in stages[mode]:
        try:
            subprocess.run([sys.executable, str(stage), "--config", str(Path(args.config).resolve())], check=True)
        except subprocess.CalledProcessError as error:
            return error.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
