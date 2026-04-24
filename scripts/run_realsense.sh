#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  align_depth.enable:=true \
  enable_sync:=true \
  color_width:=640 color_height:=480 \
  depth_width:=640 depth_height:=480
