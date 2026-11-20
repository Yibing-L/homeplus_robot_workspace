# Prompt: keyboard (WASD) + mouse teleoperation of the whole HomePlus robot

Paste everything below the line into a new Claude Code session started in
`/mnt/c/users/easha/arl/homeplus_robot_workspace` (fresh terminal, Orchestra on).

---

I want to drive the entire HomePlus robot — mobile base, arm and gripper — live
from my keyboard (WASD) and mouse, from the operator console in the browser.
Build it on a branch in a git worktree; do not restart or modify the running
stack (systemd --user hp-* units, the console on :8080) and do not send any
motion to the robot until I say so. I will test on hardware myself.

## Read first
- `parallel_prompts/HANDOFF_2026-10-29.md` (context, my rules), `LAUNCH.md`.
- Console: `homeplus_console/` (FastAPI + rclpy backend; React/TS frontend in
  `homeplus_console/web`; `README.md` explains control ownership, STOP, the
  250 ms snapshot loop, `actions.py` gating, `ros_io.py`). Existing manual
  commands (gripper, back_up, home) are examples of how motion is gated.
- Executor: `homeplus_moveit_config/scripts/gesture_motion_executor.py`
  (MoveIt planning, streamed trajectories to the Arduino, base_relative moves).
- Bridge + protocol: `homeplus_moveit_config/scripts/arduino_reader.py`,
  `firmware/homeplus_controller/homeplus_controller.ino` (STATE4 streaming,
  command queue, DONE/ERROR acks, STOP, 1.5 s host-heartbeat watchdog, 3 deg
  shoulder/elbow tracking limit, open-loop wrist/hand/gripper servos, base
  x/y/theta PID with odometry). Robot model: `homeplus_urdf_description/`,
  joint limits in `homeplus_moveit_config/config/joint_limits.yaml`.
- Known motion issues: branch `origin/jerk-diagnosis-prototypes` (firmware hold
  band and speed limiter lag; base stop-go control). Teleop must stay inside the
  tracking limit (low speeds; the arm trips at ~3 deg lag).

## What I want
- Base: W/S forward/back, A/D turn (or strafe? the base is modelled with planar
  joints base_x/base_y/base_theta — check whether it can actually strafe and
  pick the right mapping), Shift for faster, Space = stop.
- Arm: mouse controls the gripper (grasp_tcp) in Cartesian space while a mouse
  button is held — e.g. move = forward/up in the arm's plane, wheel = height or
  reach, plus keys for wrist roll and hand pitch. This arm is planar (shoulder,
  elbow, hand pitch in one plane + wrist roll), so explain the mapping you choose.
- Gripper: open/close keys (and/or mouse buttons).
- A visible on/off "teleop mode" in the console with a dead-man rule: motion
  only while the console tab is focused, I hold control, and input keeps
  arriving; any gap > ~300 ms stops the robot. STOP, E-stop flow and the
  existing safety gating must keep working and take priority.
- Collision safety: at minimum joint-limit and self-collision checks, and stop
  short of the table box / OctoMap where feasible (MoveIt Servo, or IK + a
  validity check per command). Say clearly which checks run live.
- Live feedback in the UI: camera feed (D405 hand view large), current speeds,
  which keys are active, and the 3D view showing the robot.
- Optional but valuable: a "record" toggle that logs synchronized camera images,
  joint states and commanded actions as a LeRobot-format dataset episode with
  a language instruction I type — this is the demo source for a future VLA
  fine-tune (see `parallel_prompts/VLA_SIM_FINETUNE_PROMPT.md`).

## Decide and justify
- Transport: how keyboard/mouse events get to the robot with low latency
  (WebSocket from the browser to the console backend vs a separate node), and
  rate (e.g. 20-30 Hz) with smoothing/acceleration limits.
- Control path: MoveIt Servo (realtime Cartesian jogging) vs per-tick IK vs
  direct joint velocity; and how commands reach the firmware — short streamed
  trajectory segments over the existing STATE4 protocol vs a new jog command.
  Prefer no firmware change; if one is truly needed, write it on the branch and
  flag it (flashing needs my and Yibing's approval).
- Base: velocity commands vs small relative goals; odometry drift handling.
- Speed limits that respect the 3 deg tracking limit and the 1.5 s watchdog.

## Deliver
Branch name, files changed, how to run it (build steps, how to switch the
console to the branch), a short safety section (what stops the robot and when,
what is NOT checked), and a test plan I can run with the robot on blocks /
E-stop in hand. Keep the full test suites out (I don't want them run); unit
tests for new pure logic are fine. Don't use sudo.
