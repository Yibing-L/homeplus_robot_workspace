#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

USE_GUI="${USE_GUI:-false}"

ros2 launch homeplus_urdf_description state_publisher.launch.py \
  use_gui:="$USE_GUI" \
  publish_joint_states:=true
