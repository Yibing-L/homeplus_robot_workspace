#!/usr/bin/env bash
# Re-discover bus IDs on every pass: usbipd's auto-attach can retain an old bus ID.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUNTIME="$ROOT/log/usb_attach"
mkdir -p "$RUNTIME"
USBIPD="${USBIPD:-usbipd.exe}"
HWIDS=(8086:0b5b 8086:0b5c 8086:0b3a 2341:0042 2341:0010 1a86:7523)
command -v "$USBIPD" >/dev/null || { echo 'usbipd.exe not found' >&2; exit 1; }

attach_pass() {
  local listing id line bus verify failed=0
  listing="$("$USBIPD" list | tr -d '\r')" || return 1
  for id in "${HWIDS[@]}"; do
    line="$(awk -v id="$id" 'tolower($2)==id {print; exit}' <<<"$listing")"
    [[ -z "$line" || "$line" == *Attached* ]] && continue
    if [[ "$line" == *'Not shared'* ]]; then
      echo "$id needs administrator binding: usbipd bind --hardware-id $id" >&2
      failed=1
      continue
    fi
    bus="${line%% *}"
    echo "$(date -Is) Attaching $id at current bus $bus"
    if ! "$USBIPD" attach --wsl --busid "$bus"; then
      failed=1
      continue
    fi
    verify="$("$USBIPD" list | tr -d '\r' | awk -v bus="$bus" '$1==bus {print}')"
    if [[ "$verify" == *Attached* ]]; then
      echo "$id attached at $bus"
    else
      echo "$id attachment not confirmed at $bus; will retry" >&2
      failed=1
    fi
  done
  return "$failed"
}

if [[ "${1:-}" == --once ]]; then
  attach_pass
  exit $?
fi
if [[ "${1:-}" == --watch ]]; then
  # One persistent supervisor, including when no devices are currently connected.
  exec 9>"$RUNTIME/watcher.lock"
  flock -n 9 || exit 0
  exec >>"$RUNTIME/watcher.log" 2>&1
  while true; do
    attach_pass || true
    sleep 5
  done
fi

# Retire only this helper's legacy per-device watchers and their child watchers.
stop_tree() {
  local pid="$1" child
  for child in $(pgrep -P "$pid" || true); do stop_tree "$child"; done
  kill "$pid" 2>/dev/null || true
}
for id in "${HWIDS[@]}"; do
  for pid in $(pgrep -f "usbipd.exe attach --wsl --hardware-id $id --auto-attach" || true); do
    stop_tree "$pid"
  done
done
if systemctl --user is-active --quiet hp-usb-attach; then
  echo "USB supervisor already running; log: $RUNTIME/watcher.log"
else
  systemd-run --user --unit=hp-usb-attach --collect \
    --property=Restart=on-failure --property=RestartSec=2 \
    --setenv="USBIPD=$(command -v "$USBIPD")" \
    --setenv="WSL_INTEROP=${WSL_INTEROP:-}" --setenv="WSL_DISTRO_NAME=${WSL_DISTRO_NAME:-}" \
    /bin/bash "$ROOT/scripts/usb_attach.sh" --watch || exit 1
  echo "USB supervisor started; log: $RUNTIME/watcher.log"
fi
