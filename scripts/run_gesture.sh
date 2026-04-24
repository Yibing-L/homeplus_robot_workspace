#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: bash scripts/run_gesture.sh /absolute/path/to/checkpoint.pt [label_map.json]" >&2
  exit 1
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CHECKPOINT_PATH="$1"
LABEL_MAP_PATH="${2:-}"

source /opt/ros/humble/setup.bash
source install/setup.bash

CMD=(
  ros2 launch homeplus_vision gesture_pipeline.launch.py
  "checkpoint_path:=${CHECKPOINT_PATH}"
)

if [[ -n "$LABEL_MAP_PATH" ]]; then
  CMD+=("label_map_path:=${LABEL_MAP_PATH}")
fi

"${CMD[@]}"
