#!/usr/bin/env bash
# Expose the Mac-served Arduino serial port inside the ROS VM as a local
# pseudo-terminal. Point the Arduino bridge at it:
#   ros2 launch homeplus_moveit_config gesture_control.launch.py arduino_port:=$HOME/ttyArduino ...
# Reconnects automatically if the Mac side or the network drops.
#
# Usage: tools/arduino_serial_bridge/vm_serial_client.sh [mac_ip] [port] [link]
set -u

MAC="${1:-192.168.64.1}"
PORT="${2:-8767}"
LINK="${3:-$HOME/ttyArduino}"

while true; do
  echo "$(date +%T) connecting to $MAC:$PORT -> $LINK"
  socat -d -d pty,link="$LINK",raw,echo=0 TCP:"$MAC":"$PORT",nodelay
  echo "$(date +%T) link dropped (exit $?); reconnecting in 1 s"
  sleep 1
done
