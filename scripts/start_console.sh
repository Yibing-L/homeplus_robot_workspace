#!/usr/bin/env bash
# Start the operator console; launch TF / vision / MoveIt / Arduino bridge from the UI.
set -e
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bash "$W/scripts/usb_attach.sh" || true
source /opt/ros/humble/setup.bash
source "$W/install/setup.bash"
cd "$W/homeplus_console"
exec python3 -m homeplus_console.main --profile laptop \
  --profile-file "$W/homeplus_console/config/profiles/workstation.yaml" \
  --host 0.0.0.0 --port 8080 "$@"
