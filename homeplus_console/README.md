# homeplus_console

One browser console for the HomePlus robot. It replaces the 5–7 terminal workflow in `HOMEPLUS_CHEATSHEET.md`. Design and plan: `../docs/OPERATOR_CONSOLE_PLAN.md`. Product notes: `PRODUCT.md`.

The backend (Python) owns every subscription and derives the robot state. The browser only renders full snapshots, which arrive every 250 ms over `/ws`. Every value carries its age, and a stale value is shown as unknown.

## Run in sim mode (no ROS, no robot)

```bash
cd homeplus_closed_loop/homeplus_console
pip install fastapi "uvicorn[standard]" pydantic pyyaml pillow numpy   # once
python3 -m homeplus_console.main --sim            # http://127.0.0.1:8080
```

- **Sim:** a signal-level simulation of the bridge, firmware, executor, detector and cameras. It runs on any machine.
- **Faults:** the **Sim faults** button (bottom right) injects every real failure mode: E-stop/power off, Arduino link down, camera sender dead or frozen, detector scenarios (two targets, "There are none.", stale frames, z = −6.79, first sample too high), robot pushed, Arduino reboot, MoveIt crash, console link drop, and ZERO support on/off.
- **Control:** take control (header, top right) before running anything. Other tabs are view-only, plus STOP.
- **Options:** `--profile laptop` simulates direct USB, where opening the port resets the Arduino. The default `mac_vm` profile uses the serial bridge plus a remote GPU. `--port 8090` changes the port.

## Frontend

```bash
cd web
npm install
npm run dev     # Vite on :5173, proxies /api /ws /img to CONSOLE_BACKEND (default http://127.0.0.1:8080)
npm run build   # writes ../homeplus_console/static, which the backend serves
```

React 18 + TypeScript + Vite, `zustand`, plain CSS, `three` + `urdf-loader` for the 3D plan view. The robot model comes from `homeplus_urdf_description`, served by the backend at `/api/robot/urdf`.

## Tests

```bash
python3 -m pytest test/ -q
```

These cover staleness at each threshold with a frozen clock, the state priority order, firmware-boot vs bridge-restart origin handling, the detection window, action gating, dedup, the recovery acknowledgement, and the status-text fallback parser.

## On the robot (ROS mode)

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
python3 -m homeplus_console.main --profile mac_vm --host 192.168.64.2   # or --profile laptop
```

- **ROS layer:** `ros_io.py` turns topics into the same signals the sim writes.
- **Supervisor:** `supervisor.py` runs each profile unit from `config/profiles/*.yaml` as a `systemd-run --user` transient unit.
- **Degraded mode** until the §7 robot changes land (it upgrades per signal as each one appears):
  - the timeline comes from `executor_status` text, with step names from `gesture_execution_map.yaml`;
  - the approval preview comes from `/display_planned_path`, and approval is the Bool topic;
  - Return to start is composed from `gripper_open` + `home`;
  - undo and resume are unavailable;
  - odometry resets are detected from bridge log lines.

**Integration test** against real ROS 2 Humble in Docker (fake robot on today's topics):

```bash
cd homeplus_closed_loop && bash homeplus_console/test/ros_integration/run.sh
```

The **real** `arduino_reader.py` bridge runs against `fake_arduino.py`, a pty that speaks the firmware protocol: STATE4, READY4/BOOT, ID, ZERO, STOP.

It checks:
- joint unit conversion, latched ready, detections, images, GetParameters and rosout;
- boot identity, and Set start here via ZERO;
- a bridge restart without a reset keeps the origin, and a firmware reboot loses it;
- the start gate;
- GestureCommand with its request token;
- approval derived from status text, with its preview, and the Bool approval;
- abort parsing;
- STOP ×3.

**Not yet verified on hardware:** systemd units on the VM/laptop, real camera encodings, `/move_action` readiness, and TF.
