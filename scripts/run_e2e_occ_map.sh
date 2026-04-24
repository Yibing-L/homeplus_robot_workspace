#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

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

bash "$ROOT/scripts/run_realsense.sh" &
pids+=($!)
sleep 5

bash "$ROOT/scripts/run_occ_map.sh" &
pids+=($!)

wait
