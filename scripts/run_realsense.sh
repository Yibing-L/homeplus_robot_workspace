#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch homeplus_vision multi_camera.launch.py \
  d405_serial:="${D405_SERIAL:-''}" \
  d435i_serial:="${D435I_SERIAL:-''}" \
  d455_serial:="${D455_SERIAL:-''}" \
  side_mount:="${D455_SIDE:-left}"
