#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REGISTRY="$ROOT/homeplus_vision/config/task_registry.yaml"

usage() {
  echo "Usage: bash scripts/task.sh TASK_ID {info|vision|execute} [vision launch args...]" >&2
  echo "Example: bash scripts/task.sh 1 vision" >&2
  echo "         bash scripts/task.sh 1 execute" >&2
}

if [[ $# -lt 2 ]]; then
  usage
  exit 2
fi

TASK_ID="$1"
ACTION="$2"
shift 2

if [[ ! "$TASK_ID" =~ ^[0-9]+$ ]]; then
  echo "TASK_ID must be a positive integer, got: $TASK_ID" >&2
  exit 2
fi

mapfile -t TASK_FIELDS < <(
  python3 - "$REGISTRY" "$TASK_ID" <<'PY'
import sys
from pathlib import Path
import yaml

registry_path = Path(sys.argv[1])
task_id = int(sys.argv[2])
payload = yaml.safe_load(registry_path.read_text(encoding="utf-8")) or {}
tasks = payload.get("tasks", {})
task = tasks.get(task_id, tasks.get(str(task_id)))
if not isinstance(task, dict):
    raise SystemExit(f"Unknown task_id {task_id} in {registry_path}")
print(str(task.get("name", f"task_{task_id}")))
print(str(task.get("caption", "")))
print(str(task.get("command_name", "")))
print(str(task.get("command_type", "")))
PY
)

TASK_NAME="${TASK_FIELDS[0]:-}"
TASK_CAPTION="${TASK_FIELDS[1]:-}"
COMMAND_NAME="${TASK_FIELDS[2]:-}"
COMMAND_TYPE="${TASK_FIELDS[3]:-}"

if [[ -z "$TASK_CAPTION" ]]; then
  echo "Task $TASK_ID has no MolmoPoint caption in $REGISTRY" >&2
  exit 2
fi

case "$ACTION" in
  info)
    echo "task_id=$TASK_ID"
    echo "name=$TASK_NAME"
    echo "molmopoint_prompt=Point to $TASK_CAPTION"
    if [[ -n "$COMMAND_NAME" ]]; then
      echo "motion_command=$COMMAND_NAME ($COMMAND_TYPE)"
    else
      echo "motion_command=none (perception-only task)"
    fi
    ;;

  vision)
    echo "Starting task $TASK_ID: $TASK_NAME"
    echo "MolmoPoint prompt: Point to $TASK_CAPTION"
    exec bash "$ROOT/scripts/run_vision.sh" \
      side_mount:=left \
      task_id:="$TASK_ID" \
      enable_d405:=true \
      enable_d435i:=false \
      enable_d455:=false \
      enable_target_detection:=true \
      enable_gesture_detection:=false \
      enable_d405_octomap:=true \
      enable_d435i_octomap:=false \
      enable_d455_octomap:=false \
      "$@"
    ;;

  execute)
    if [[ -z "$COMMAND_NAME" || -z "$COMMAND_TYPE" ]]; then
      echo "Task $TASK_ID ($TASK_NAME) is perception-only; no motion command is registered." >&2
      exit 2
    fi
    cd "$ROOT"
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    ACTIVE_TASK_ID="$(
      ros2 param get /molmopoint task_id 2>/dev/null | sed -n 's/.*: //p'
    )"
    if [[ -z "$ACTIVE_TASK_ID" ]]; then
      echo "Cannot read /molmopoint task_id. Start task vision first." >&2
      exit 1
    fi
    if [[ "$ACTIVE_TASK_ID" != "$TASK_ID" ]]; then
      echo "Refusing to execute task $TASK_ID: MolmoPoint is running task_id=$ACTIVE_TASK_ID." >&2
      echo "Restart vision with: bash scripts/task.sh $TASK_ID vision" >&2
      exit 1
    fi

    echo "Executing task $TASK_ID: $TASK_NAME -> $COMMAND_NAME"
    ros2 topic pub --once /gesture_control/command \
      homeplus_interfaces/msg/GestureCommand \
      "{gesture_id: $TASK_ID, gesture_label: '$TASK_NAME', confidence: 1.0, command_name: '$COMMAND_NAME', command_type: '$COMMAND_TYPE'}"
    ;;

  *)
    echo "Unknown action: $ACTION" >&2
    usage
    exit 2
    ;;
esac
