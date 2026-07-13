#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

TASK_ID="${1:-1}"
WORLD_FRAME="${WORLD_FRAME:-world}"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 run homeplus_vision grounding_dino_node.py --ros-args \
  -p task_id:="$TASK_ID" \
  -p world_frame:="$WORLD_FRAME" \
  -p image_topic:=/camera/camera/color/image_raw \
  -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info \
  -p camera_frame:=hand_camera_color_optical_frame
