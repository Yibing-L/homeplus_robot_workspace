#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  rgb_camera.color_profile:=640x480x30 \
  depth_module.color_profile:=640x480x30 \
  depth_module.depth_profile:=640x480x30
