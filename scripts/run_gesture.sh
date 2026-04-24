#!/usr/bin/env bash
set -eo pipefail

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
set -u

CMD=(
  ros2 run homeplus_vision gesture_recognizer.py
  --ros-args
  "-p" "checkpoint_path:=${CHECKPOINT_PATH}"
)

if [[ -n "$LABEL_MAP_PATH" ]]; then
  CMD+=("-p" "label_map_path:=${LABEL_MAP_PATH}")
fi

"${CMD[@]}"
