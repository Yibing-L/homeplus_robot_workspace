#!/usr/bin/env bash
set -eo pipefail

# Default to the trained ds_group64 model (numpy-1.x-compatible re-save).
# The original model_7.pt was pickled under numpy 2.x; cv_bridge (compiled
# against numpy 1.x) segfaults if numpy 2.x is forced via PYTHONPATH, so we
# converted the checkpoint with scripts/convert_checkpoint_np_compat.py.
DEFAULT_CHECKPOINT="/mnt/c/users/easha/arl/homeplus_gesture/runs/ds_group64/model_7_np1compat.pt"

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
