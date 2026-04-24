#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch homeplus_moveit_config cloud_to_octomap.launch.py \
  octomap_input_topic:=/full_cloud \
  depth_topic:=/camera/camera/depth/image_rect_raw \
  camera_info_topic:=/camera/camera/depth/camera_info \
  output_frame:=base_link \
  octomap_frame:=base_link
