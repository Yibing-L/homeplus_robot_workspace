#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

USE_GUI="${USE_GUI:-false}"

# Bridge robot TF tree to RealSense TF tree
# Only publish hand_camera_link_1 → camera_link; RealSense driver handles camera_link → optical frames
ros2 run tf2_ros static_transform_publisher --frame-id hand_camera_link_1 --child-frame-id camera_link &

ros2 launch homeplus_urdf_description state_publisher.launch.py \
  use_gui:="$USE_GUI" \
  publish_joint_states:=true
