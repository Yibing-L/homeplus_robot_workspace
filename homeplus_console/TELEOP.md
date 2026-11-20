# Teleop: drive base, arm and gripper from the console (branch `ev-teleop`)

Keyboard drives the base, a mouse drag on the hand-camera view drives the gripper in Cartesian
space, and every limit and safety check runs in the console backend. No firmware change: the
firmware's existing STR streaming protocol is used as an open-ended stream. The bridge gains
one additive feature (`/arduino/jog`); the executor is unchanged.

## Controls (Teleop tab, after **Turn teleop on**)

| Input | Moves | Normal / Shift |
|---|---|---|
| W / S | base forward / back | 5 / 12 cm/s |
| A / D | base strafe left / right (the base is holonomic) | 5 / 12 cm/s |
| Q / E | base turn left / right | 10 / 20 °/s |
| hold left mouse on the camera, move → / ← | gripper TCP reach out / in (base_link x) | ≤ 3 / 6 cm/s |
| same, move ↑ / ↓ | gripper TCP up / down (base_link z) | ≤ 3 / 6 cm/s |
| mouse wheel, R / F | tilt the gripper (TCP pitch) tip up / down, about the TCP | 3°/notch, 15 / 25 °/s |
| Z / X | wrist roll | 30 / 60 °/s |
| O / C | gripper open / close while held (0–80°) | 40 / 80 °/s |
| G | gripper toggle: fully open (0°) ↔ closed (75°, the executor's grasp value) | |
| Space, Esc, STOP button | STOP (the console's existing STOP) | |

Mouse scale: 0.15 mm per pixel (0.4 with Shift). At most 3 cm of mouse motion is queued (the
rest is dropped) and releasing the button drops the queue, so the gripper stops when you let go
(the joints decelerate over ~0.2 s, a few mm).

**Why this mapping.** `base_x/base_y/base_theta` are planar joints because the base is a
4-wheel X/mecanum drive: the firmware runs separate x, y and θ PIDs on wheel odometry and mixes
them per wheel (`drive(vx, vy, ω)`), so it can strafe — A/D strafe, Q/E turn. The arm is planar:
shoulder, elbow and hand pitch rotate about axes parallel to base_link Y, so with wrist roll at 0
the TCP moves in base_link's x–z plane, which is what the mouse drives (x = reach, y = height).
The third in-plane DOF is tool pitch (wheel / R/F); IK keeps pitch fixed while you drag and keeps
the TCP fixed while you tilt. Lateral gripper position comes from the base (A/D strafe moves the
TCP sideways exactly; turning swings it on an arc). With wrist roll ≠ 0 the hand pitch tilts out
of the plane, so the TCP's sideways position is left free (it can drift a few cm as you pitch).

## Architecture and why

```
browser ──WebSocket /ws, input frame every 33 ms──▶ console TeleopService (25 Hz thread)
   keys held, mouse Δpx, wheel, focus/visibility       Controller: limits, IK, governor, collision probe
                                                        ├─ /check_state_validity (move_group), each tick that moves
                                                        └─ /arduino/jog  {"op":"point", q: 8 SI joints}
arduino_reader.py (bridge) ── STR stream, one point per packet, each due 300 ms ahead ──▶ firmware
```

- **Transport: the console's existing WebSocket**, not a separate node. The console already
  owns control (lease), STOP, gating and the robot state, and the browser is already connected;
  a frame every 33 ms is ~1 kB/s. A separate node would have to re-implement the lease and the
  dead-man and would add a second path to the robot.
- **Control path: per-tick IK in the console, absolute joint setpoints streamed to the firmware.**
  MoveIt Servo isn't installed, assumes a velocity/position controller interface we don't have,
  and its 6-DOF Cartesian jogging fits a 4-DOF planar arm poorly (KDL-style IK on this arm fails
  most 6-D targets). The planar IK here is a 3×3 damped least squares on the URDF chain (0.4 ms).
  Direct joint velocity would need a firmware jog command; the firmware can't take one.
- **Firmware path: the existing `STR` stream, left open (final flag 0).** The bridge sends one
  point per packet (~90 bytes; measured 20 points/s against the emulator with RX4/BUF3 flow control) timed 300 ms ahead
  of the firmware clock; the firmware interpolates toward it, enforces its 3°/2°/6 cm tracking
  limits, and stops with `BUFFER_UNDERRUN` if points stop coming. To stop, the bridge sends a
  final point equal to the last one; the firmware settles and reports `DONE`. A teleop "session"
  is one such stream; after 0.5 s with nothing commanded it ends and the next key starts a new
  one. Sessions longer than 110 s end automatically (firmware point times are capped at 120 s).
- **Base: velocity → absolute odometry setpoints.** Key velocities are acceleration-limited and
  integrated into a commanded (x, y, θ) in the firmware's odometry frame, starting each session
  at the measured pose. The firmware's PID then tracks that, so drift doesn't accumulate between
  command and robot within a session. Odometry itself still drifts against the world (wheel
  slip): the OctoMap and table box are in the odometry-anchored world frame, so collision checks
  near objects get less accurate the further you drive; re-detect/clear the map after long drives.
- **Speed limits vs the 3° tracking limit.** Shoulder 8 °/s, elbow 10 °/s, 20–25 °/s² (the
  firmware's PWM speed caps are ~30/40 °/s). Base 5/12 cm/s, 10/20 °/s. On top of that, a
  **tracking governor** estimates what the firmware is currently interpolating (our own command
  300 ms + latency ago) and stops advancing an axis whose lag behind the encoders exceeds 1.8°
  (arm), 1.2° (yaw) or 3.5 cm (x/y) — ~60 % of the firmware's trip limits. If the robot is
  ≥ 6° / 6° / 10 cm from the command, teleop stops ("not following": E-stop, power, stall).
- **Watchdog (1.5 s host heartbeat).** Every packet refreshes it; the bridge also PINGs. The
  stream underrun (≤ 0.3 s) always fires long before it.

## Safety

**What stops the robot, and how fast** (the firmware holds wherever it stops):

| Event | Detected by | Robot stops |
|---|---|---|
| STOP button, Esc, **Space** | console → `/arduino/stop` ×3 | immediately (firmware `STOP`), as before |
| no input frame for 300 ms (browser hung, network) | console | ≤ 0.3 s later (stream ends at the last point) |
| tab loses focus / hidden, or a key-up was missed (blur clears keys) | browser + console | same |
| control lost / released, executor starts a command, unacknowledged hardware fault | console | same, and teleop mode turns off |
| console stalls (no setpoint for 150 ms) | bridge watchdog | ≤ 0.3 s |
| console or bridge dies | firmware `BUFFER_UNDERRUN` | ≤ 0.3 s after the last point |
| bridge ↔ firmware link dies | firmware `HOST_LOST` | 1.5 s (unchanged) |
| robot lags the command (stall, collision, overload) | firmware tracking limits 3° / 2° / 6 cm | immediately (`TRACKING_ERROR`, sticky fault → teleop off) |
| a console bug commands a jump | bridge rate guard (≈2× the teleop limits) | immediately (`STOP`, `JOG_RATE_GUARD`) |
| next state would collide | console collision probe | the command holds; blocked contacts shown |

After any stop teleop is **latched**: release every key and the mouse button with the tab
focused before anything moves again. While teleop is on, Start/Home/gripper buttons and plan
approvals are disabled; STOP, E-stop flow and all existing gating are unchanged.

**What is checked live, every tick that moves** (MoveIt `/check_state_validity`, whole robot,
~2–3 ms measured against move_group): the state 0.5 s ahead along the current motion, and at
least 3 cm (base), 3° (yaw) or 1.5 cm (TCP) ahead, so the robot stops short of contact:
- joint limits: teleop bounds = `joint_limits.yaml` minus 2° (elbow ≤ 140° like the firmware),
  gripper ≤ 80°; plus the bridge's and firmware's own limit checks;
- self-collision (SRDF allowed-collision matrix);
- world objects in the planning scene: `detected_table`, `detected_cup`, held (attached) cup;
- the OctoMap.

Collision mode (selector in the teleop bar, shown red when not "full"):
- **full** (default): all of the above.
- **no OctoMap**: ignore voxels (e.g. voxels on the cup you are about to grasp).
- **self-collision only**: ignore the table, the cup and voxels. Joint limits still apply.

If the robot is already in contact (e.g. noisy voxels around the hand), motions that don't deepen
any contact are allowed so you can back out. The gripper may always close onto an object (finger
contacts don't block a gripper-only motion). If `/check_state_validity` fails 3 ticks in a row,
teleop stops ("collision check unavailable").

**NOT checked:**
- anything the planning scene doesn't contain: obstacles the hand camera hasn't seen (the D455/D435i
  don't feed the OctoMap), people, the floor behind the robot, other furniture. Driving the base is
  protected only against the table box and voxels the D405 saw.
- the OctoMap is not updated while a stream runs (the obstacle cloud pauses during motion) — new
  obstacles appear only after the robot settles between strokes;
- odometry drift (see above) and the hand-camera extrinsic error (cm-level) shift where voxels
  and the table are relative to the robot;
- the wrist, hand and gripper servos are open-loop: the firmware can't detect a stall there, and
  "closed on the cup" means "commanded", not measured. Gripper force is not limited;
- forces on the arm in general (no torque sensing); a soft collision under the tracking limits
  (3°) isn't detected;
- with collision mode ≠ full, the ignored categories.

Known: at low speed the current firmware's 0.5° hold deadband makes shoulder/elbow move in small
stop-go steps (diagnosed on `jerk-diagnosis-prototypes`, needs a reflash). Teleop doesn't need
that fix; it only makes slow arm motion smoother.

## Recording demonstrations (LeRobot)

With teleop on, type the language instruction and press **● Record episode**; finish with
**Save: success / failure** or **Discard**. Each episode is a directory under
`~/.homeplus_console/teleop_episodes/` (10 Hz): `meta.json`, `steps.jsonl` (measured joints =
state, streamed setpoint = action, commanded base velocity, operator input), and JPEGs of the
D405 (hand) and D455 (front) at native size. Convert to a LeRobot dataset where `lerobot` is
installed:

```bash
python3 tools/teleop_to_lerobot.py ~/.homeplus_console/teleop_episodes --repo-id local/homeplus_teleop \
    --root ~/datasets/homeplus_teleop --only-success
```

Default action = [base vx, vy, ωz (body frame)] + absolute gripper/hand/wrist/elbow/shoulder
setpoints (`--base-action absolute` for the odometry setpoint instead). The converter is untested
here (no lerobot on this machine): check the first episode with LeRobot's visualizer.

## Run it

```bash
# 1. build the branch's bridge into the worktree (doesn't touch the main install)
cd /mnt/c/users/easha/arl/homeplus_teleop
source /opt/ros/humble/setup.bash && source /mnt/c/users/easha/arl/homeplus_robot_workspace/install/setup.bash
colcon build --packages-select homeplus_moveit_config
# (the web UI is prebuilt in homeplus_console/homeplus_console/static; to rebuild: cd homeplus_console/web && npm run build)

# 2. console: stop the main console (Ctrl-C), then
bash scripts/teleop_console.sh            # same port 8080, same units, adds the Teleop tab

# 3. bridge: with the arm safe and the E-stop in hand (RESETS the Arduino on this laptop)
bash scripts/teleop_bridge.sh             # back to main: bash scripts/teleop_bridge.sh --main
```

Try the UI without the robot first: `cd homeplus_console && python3 -m homeplus_console.main --sim --port 8090`
(the sim has a table box to drive into). Tunables: `teleop:` in `config/console.yaml` overrides any
key of `DEFAULTS` in `homeplus_console/teleop.py` (speeds, limits, governor, dead-man).

Mixed versions: the branch console with the main bridge shows "the running bridge has no live
jog" and won't enable teleop; the main console works unchanged with the branch bridge.

## Test plan (robot on blocks, E-stop in hand)

Watch the Teleop tab's HUD; after each step confirm the robot is still and the phase is READY or HOLD.

0. **Sim first** (`--sim`): all steps below, plus drive into the table (stops ~3 cm before it).
1. **Bring-up**: branch bridge running, `ros2 topic echo --once /arduino/bridge_info` shows `"jog": true`.
   Take control → Turn teleop on → HOLD → release all → READY. Main buttons show "teleop mode is on".
2. **Dead-man with no motion risk**: hold W for ½ s (wheels off the ground), then Alt-Tab away while
   holding: wheels stop within ~0.3 s, HUD "dead-man: tab lost focus". Repeat by pulling the network
   cable / closing the laptop lid of the browser machine if remote.
3. **STOP paths**: hold W, press Space → immediate stop, HOLD "STOP pressed". Hold W, press Esc → same.
   Hold W, click the red STOP → same. Then the physical E-stop while holding W → TRACKING_ERROR or
   "not following" → teleop turns off; acknowledge the fault.
4. **Base on blocks**: W/S/A/D/Q/E one at a time, normal then Shift: wheel directions match
   (A = strafe left), HUD speeds 5/12 cm/s, 10/20 °/s, no TRACKING_ERROR, no "waiting for … to catch
   up" at normal speed. Release → stops within ~0.3 s; after ½ s the stream ends (STOPPING → READY).
5. **Arm, small strokes**: drag right 2 cm → TCP reach +, height constant (HUD), pitch constant.
   Drag up/down. Wheel, R/F → tilt about the TCP. Z/X roll. Watch for tracking warnings; note
   whether the arm stutters (firmware deadband, known).
6. **Gripper**: O/C ramp, G toggle open/closed.
7. **Collision**: with a box/table in the planning scene (detect the table or a test object),
   lower the gripper toward it → stops ~1.5 cm above, HUD shows the contact pair. Switch to
   "no OctoMap"/"self only" and back; the selector turns red when not full.
8. **Long stroke**: hold W (on blocks) > 2 min → the stream re-anchors at ~110 s with a short pause.
9. **Recording**: record a 10 s episode, Save: success; check `steps.jsonl` rows ≈ 10/s and
   images in `images/hand`.
10. Off: Turn teleop off → main buttons return; Start a task works as before.

Unit tests for the new logic: `python3 -m pytest -p no:anyio -q homeplus_console/test/test_teleop.py`.
Bridge jog path against a firmware emulator (no hardware, isolated ROS domain):
`homeplus_console/test/ros_integration/fake_jog_firmware.py`.
