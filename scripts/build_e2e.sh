#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install --packages-select \
  homeplus_interfaces \
  homeplus_urdf_description \
  homeplus_moveit_config \
  homeplus_vision

echo
echo "Build complete."
echo "Next: source install/setup.bash"
