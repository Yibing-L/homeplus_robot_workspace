#!/usr/bin/env bash
# Run the ev-teleop console (this worktree) in place of the main one: same port, same units.
# Stop the main console first (Ctrl-C in its terminal). See homeplus_console/TELEOP.md.
set -e
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAIN="${HOMEPLUS_MAIN:-/mnt/c/users/easha/arl/homeplus_robot_workspace}"
source /opt/ros/humble/setup.bash
source "$MAIN/install/setup.bash"
[ -f "$W/install/setup.bash" ] && source "$W/install/setup.bash"
cd "$W/homeplus_console"
exec python3 -m homeplus_console.main --profile laptop \
  --profile-file "$W/homeplus_console/config/profiles/workstation_teleop.yaml" \
  --host 0.0.0.0 --port "${PORT:-8080}" "$@"
