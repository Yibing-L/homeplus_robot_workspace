#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WORKSPACE_ROOT"

TASK_ID="${1:-1}"
WORLD_FRAME="${WORLD_FRAME:-world}"
VENV_DIR="${MOLMOPOINT_VENV:-$WORKSPACE_ROOT/.venv-molmopoint}"

if [[ ! -x "$VENV_DIR/bin/python" ]]; then
  echo "MolmoPoint environment not found. Run: bash scripts/setup_molmopoint.sh --download-model" >&2
  exit 1
fi

source /opt/ros/humble/setup.bash
source install/setup.bash
source "$VENV_DIR/bin/activate"
set -u

ros2 run homeplus_vision molmopoint_node.py --ros-args \
  --params-file "$WORKSPACE_ROOT/homeplus_vision/config/target_detection_params.yaml" \
  -p task_id:="$TASK_ID" \
  -p world_frame:="$WORLD_FRAME"
