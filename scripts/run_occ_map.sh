#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# Vision, robot TF and settled hardware status must already be available.
exec ros2 launch homeplus_moveit_config nvblox_mapping.launch.py "$@"
