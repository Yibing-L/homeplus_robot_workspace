#!/usr/bin/env bash
# Usage (from homeplus_closed_loop/): bash homeplus_console/test/ros_integration/run.sh
set -e
cd "$(dirname "$0")/../../.."
docker build -q -t homeplus-console-rostest -f homeplus_console/test/ros_integration/Dockerfile . >/dev/null
docker run --rm -v "$PWD":/repo:ro homeplus-console-rostest bash -c '
  source /opt/ros/humble/setup.bash && source /ws/install/setup.bash
  cd /repo/homeplus_console
  python3 test/ros_integration/fake_arduino.py & sleep 1
  python3 /repo/homeplus_moveit_config/scripts/arduino_reader.py --port /tmp/ttyFAKE > /tmp/bridge.log 2>&1 &
  python3 test/ros_integration/fake_robot.py & sleep 2
  PYTHONDONTWRITEBYTECODE=1 python3 -m homeplus_console.main --profile-file test/ros_integration/test_profile.yaml \
      --host 127.0.0.1 --port 8080 --data-dir /tmp/console > /tmp/console.log 2>&1 &
  sleep 6
  python3 test/ros_integration/drive.py; rc=$?
  echo "--- console log"; tail -8 /tmp/console.log; echo "--- bridge log"; tail -12 /tmp/bridge.log
  exit $rc'
