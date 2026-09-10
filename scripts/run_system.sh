#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WITH_OCC_MAP=true
WITH_MOLMOPOINT=false
TASK_ID=1
GESTURE_CHECKPOINT=""
GESTURE_LABEL_MAP=""
USE_GUI=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-occ-map)
      WITH_OCC_MAP=false
      shift
      ;;
    --with-molmopoint)
      WITH_MOLMOPOINT=true
      shift
      ;;
    --with-dino)
      echo "--with-dino is deprecated; use --with-molmopoint" >&2
      WITH_MOLMOPOINT=true
      shift
      ;;
    --task-id)
      TASK_ID="$2"
      shift 2
      ;;
    --gesture-checkpoint)
      GESTURE_CHECKPOINT="$2"
      shift 2
      ;;
    --gesture-label-map)
      GESTURE_LABEL_MAP="$2"
      shift 2
      ;;
    --use-gui)
      USE_GUI=true
      shift
      ;;
    *)
      echo "Unknown arg: $1" >&2
      echo "Usage: bash scripts/run_system.sh [--use-gui] [--with-molmopoint] [--task-id N] [--gesture-checkpoint PATH] [--gesture-label-map PATH] [--no-occ-map]" >&2
      exit 1
      ;;
  esac
done

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

pids=()

cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}

trap cleanup EXIT INT TERM

USE_GUI="$USE_GUI" bash "$ROOT/scripts/run_robot_tf.sh" &
pids+=($!)
sleep 3

ENABLE_GESTURE=false
if [[ -n "$GESTURE_CHECKPOINT" ]]; then
  ENABLE_GESTURE=true
fi

bash "$ROOT/scripts/run_vision.sh" \
  enable_target_detection:="$WITH_MOLMOPOINT" \
  enable_gesture_detection:="$ENABLE_GESTURE" \
  enable_d405_octomap:="$WITH_OCC_MAP" \
  enable_d435i_octomap:=false \
  enable_d455_octomap:=false \
  task_id:="$TASK_ID" \
  gesture_checkpoint:="$GESTURE_CHECKPOINT" \
  gesture_label_map:="$GESTURE_LABEL_MAP" &
pids+=($!)

wait
