#!/usr/bin/env bash
set -euo pipefail
TOOL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -eq 0 ]]; then
  exec python3 "$TOOL_ROOT/src/GZ_PCD/gazebo_to_pcd.py" --config "$TOOL_ROOT/config/default.yaml"
elif [[ "$1" == -* ]]; then
  exec python3 "$TOOL_ROOT/src/GZ_PCD/gazebo_to_pcd.py" "$@"
else
  exec python3 "$TOOL_ROOT/src/GZ_PCD/gazebo_to_pcd.py" --config "$1"
fi
