#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# A separately running Arduino bridge supplies measured joints and settled status.
# This helper starts no controller or motion executor.
pids=()

cleanup() {
  for pid in "${pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}

trap cleanup EXIT INT TERM

bash "$ROOT/scripts/run_robot_tf.sh" &
pids+=($!)
sleep 3

bash "$ROOT/scripts/run_vision.sh" &
pids+=($!)
sleep 5

bash "$ROOT/scripts/run_occ_map.sh" &
pids+=($!)

wait
