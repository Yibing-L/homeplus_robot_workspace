#!/usr/bin/env bash
# Compile and flash firmware/homeplus_controller to the Arduino Mega from the Mac,
# then restart the Mac side of the serial bridge.
# Flashing resets the Arduino: the arm goes upright and odometry is zeroed
# wherever the robot stands.
#
# Usage: tools/flash_firmware.sh [serial_port]   (default: first /dev/cu.usbmodem*)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FQBN=arduino:avr:mega
SKETCH="$ROOT/firmware/homeplus_controller"

# 1. Release the port: the serial bridge holds it while running.
pkill -f mac_serial_server.sh 2>/dev/null || true
pkill -f "TCP-LISTEN:8767" 2>/dev/null || true
sleep 1

# 2. One-time setup (skipped once installed).
if ! command -v arduino-cli >/dev/null; then
  brew install arduino-cli
fi
if ! arduino-cli core list | grep -q '^arduino:avr'; then
  arduino-cli core update-index
  arduino-cli core install arduino:avr
fi
arduino-cli lib list | grep -qi '^AS5600' || arduino-cli lib install Servo Encoder PID AS5600

# 3. Find the Arduino (waits up to 30 s, e.g. after replugging it).
PORT="${1:-}"
for _ in $(seq 1 30); do
  [ -n "$PORT" ] && [ -e "$PORT" ] && break
  PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -1 || true)"
  [ -n "$PORT" ] && break
  echo "waiting for the Arduino (is it plugged in, and NOT attached to the VM in UTM?)"
  sleep 1
done
[ -n "$PORT" ] || { echo "No Arduino found at /dev/cu.usbmodem*" >&2; exit 1; }
echo "Arduino at $PORT"

# 4. Compile and upload.
arduino-cli compile --fqbn "$FQBN" "$SKETCH"
echo "Uploading - the arm will rise when the new firmware boots; stand clear."
arduino-cli upload --fqbn "$FQBN" -p "$PORT" "$SKETCH"

# 5. Restart the serial bridge for the ROS VM (runs in the foreground).
echo "Flashed. Restarting the serial bridge (Ctrl-C to stop it)."
exec "$ROOT/tools/arduino_serial_bridge/mac_serial_server.sh"
