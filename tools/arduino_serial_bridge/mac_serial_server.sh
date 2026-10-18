#!/usr/bin/env bash
# Serve the Arduino's USB serial port to the ROS VM over TCP, so the Arduino
# never has to go through UTM USB passthrough (which drops devices).
# Runs on the Mac. Restarts automatically if the Arduino is unplugged.
#
# Usage: tools/arduino_serial_bridge/mac_serial_server.sh [bind_ip] [port]
set -u

BIND="${1:-192.168.64.1}"   # Mac address on the UTM shared network
PORT="${2:-8767}"
BAUD=115200                 # must match Serial.begin() in the firmware
SOCAT="$(command -v socat || echo /opt/homebrew/bin/socat)"

while true; do
  DEV="$(ls /dev/cu.usbmodem* 2>/dev/null | head -1)"
  if [ -z "$DEV" ]; then
    echo "$(date +%T) waiting for Arduino (/dev/cu.usbmodem*) - is it detached from UTM?"
    sleep 2
    continue
  fi
  echo "$(date +%T) serving $DEV at $BAUD baud on $BIND:$PORT"
  # One client at a time; hupcl=0 keeps DTR up so closing does not reset the board.
  "$SOCAT" -d -d TCP-LISTEN:"$PORT",bind="$BIND",reuseaddr \
    "$DEV",ispeed="$BAUD",ospeed="$BAUD",raw,echo=0,clocal=1,hupcl=0,crtscts=0,ixon=0,ixoff=0
  echo "$(date +%T) connection ended (exit $?); restarting in 1 s"
  sleep 1
done
