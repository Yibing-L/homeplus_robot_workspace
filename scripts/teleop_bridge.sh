#!/usr/bin/env bash
# Swap the running hp-bridge unit between the main bridge and the ev-teleop bridge (adds the
# /arduino/jog live stream; everything else is unchanged and the firmware is NOT touched).
#
#   bash scripts/teleop_bridge.sh          # run the teleop branch bridge
#   bash scripts/teleop_bridge.sh --main   # back to the main workspace bridge
#
# WARNING: on this laptop (direct USB) reopening /dev/ttyACM0 resets the Arduino: the arm drops
# for ~1 s and rises upright again, and odometry re-zeroes wherever the robot stands.
set -e
W="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MAIN="${HOMEPLUS_MAIN:-/mnt/c/users/easha/arl/homeplus_robot_workspace}"
PORT_DEV="${ARDUINO_PORT:-/dev/ttyACM0}"
SRC="source /opt/ros/humble/setup.bash && source $MAIN/install/setup.bash"
if [ "${1:-}" != "--main" ]; then
  [ -f "$W/install/homeplus_moveit_config/lib/homeplus_moveit_config/arduino_reader.py" ] || {
    echo "Build the branch first: cd $W && source /opt/ros/humble/setup.bash && source $MAIN/install/setup.bash && colcon build --packages-select homeplus_moveit_config"; exit 1; }
  SRC="$SRC && source $W/install/setup.bash"
fi
echo "This restarts hp-bridge. Opening the USB port RESETS the Arduino: the arm drops ~1 s, odometry re-zeroes."
read -r -p "Arm in a safe pose, area clear, hand on the E-stop? Type yes: " ok
[ "$ok" = "yes" ] || { echo "aborted"; exit 1; }
systemctl --user stop hp-bridge 2>/dev/null || true
systemd-run --user --unit=hp-bridge --collect --property=KillMode=control-group --property=TimeoutStopSec=10 \
  bash -lc "$SRC && ros2 run homeplus_moveit_config arduino_reader.py --port $PORT_DEV --baud 115200 --ros-args -r __node:=arduino_bridge"
sleep 4
journalctl --user -u hp-bridge -n 8 --no-pager | cut -c1-200
echo "Check: ros2 topic echo --once /arduino/bridge_info   (\"jog\": true on the branch bridge)"
