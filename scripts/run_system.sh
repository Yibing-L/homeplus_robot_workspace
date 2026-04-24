#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WITH_OCC_MAP=true
WITH_DINO=false
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
    --with-dino)
      WITH_DINO=true
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
      echo "Usage: bash scripts/run_system.sh [--use-gui] [--with-dino] [--task-id N] [--gesture-checkpoint PATH] [--gesture-label-map PATH] [--no-occ-map]" >&2
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

bash "$ROOT/scripts/run_realsense.sh" &
pids+=($!)
sleep 5

if [[ "$WITH_OCC_MAP" == true ]]; then
  bash "$ROOT/scripts/run_occ_map.sh" &
  pids+=($!)
  sleep 3
fi

if [[ "$WITH_DINO" == true ]]; then
  bash "$ROOT/scripts/run_dino.sh" "$TASK_ID" &
  pids+=($!)
  sleep 3
fi

if [[ -n "$GESTURE_CHECKPOINT" ]]; then
  if [[ -n "$GESTURE_LABEL_MAP" ]]; then
    bash "$ROOT/scripts/run_gesture.sh" "$GESTURE_CHECKPOINT" "$GESTURE_LABEL_MAP" &
  else
    bash "$ROOT/scripts/run_gesture.sh" "$GESTURE_CHECKPOINT" &
  fi
  pids+=($!)
fi

wait
