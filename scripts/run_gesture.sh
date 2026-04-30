#!/usr/bin/env bash
set -eo pipefail

# Default to the trained ds_group64 model if no arg passed.
DEFAULT_CHECKPOINT="/mnt/c/users/easha/arl/homeplus_gesture/runs/ds_group64/model_7.pt"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CHECKPOINT_PATH="${1:-$DEFAULT_CHECKPOINT}"
LABEL_MAP_PATH="${2:-}"

if [[ ! -f "$CHECKPOINT_PATH" ]]; then
  echo "Checkpoint not found: $CHECKPOINT_PATH" >&2
  echo "Usage: bash scripts/run_gesture.sh [/abs/path/to/checkpoint.pt] [label_map.json]" >&2
  exit 1
fi
echo "Using checkpoint: $CHECKPOINT_PATH"

source /opt/ros/humble/setup.bash
source install/setup.bash

# Prepend isolated numpy 2.x (needed to unpickle checkpoints saved under
# numpy 2.x; system numpy stays at 1.x for ROS / scipy / etc.)
GESTURE_NUMPY_DIR="${GESTURE_NUMPY_DIR:-$HOME/.local/homeplus_gesture_pkgs}"
if [[ -d "$GESTURE_NUMPY_DIR" ]]; then
  export PYTHONPATH="$GESTURE_NUMPY_DIR:${PYTHONPATH:-}"
else
  echo "Warning: $GESTURE_NUMPY_DIR not found." >&2
  echo "  Re-run setup:" >&2
  echo "    python3 -m pip install --target $GESTURE_NUMPY_DIR --upgrade 'numpy>=2.0,<2.3'" >&2
fi

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
