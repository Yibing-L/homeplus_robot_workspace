#!/usr/bin/env bash
# Attach the robot's USB devices (RealSense cameras, Arduino) to WSL and keep them
# attached across replugs. No admin needed; each device only has to be bound once
# from an admin PowerShell:  usbipd bind --hardware-id <VID:PID>
# Safe to run repeatedly: skips devices that already have an auto-attach watcher.
set -u

HWIDS=(
  8086:0b5b   # RealSense D405
  8086:0b5c   # RealSense D455
  8086:0b3a   # RealSense D435i
  2341:0042   # Arduino Mega 2560 (genuine)
  2341:0010   # Arduino Mega 2560 (older)
  1a86:7523   # CH340 clone Mega
)

USBIPD=usbipd.exe
command -v "$USBIPD" >/dev/null || { echo "usbipd.exe not found (install usbipd-win on Windows)" >&2; exit 1; }
LIST="$("$USBIPD" list 2>/dev/null | tr -d '\r')"

for id in "${HWIDS[@]}"; do
  line="$(grep -i " $id " <<<"$LIST" | head -1)"
  [ -z "$line" ] && continue
  if grep -q "Not shared" <<<"$line"; then
    echo "$id not bound yet; run once in admin PowerShell: usbipd bind --hardware-id $id"
    continue
  fi
  if pgrep -f "attach --wsl --hardware-id $id" >/dev/null; then
    echo "$id already auto-attaching"
    continue
  fi
  nohup "$USBIPD" attach --wsl --hardware-id "$id" --auto-attach >/dev/null 2>&1 &
  echo "$id auto-attach started"
done
