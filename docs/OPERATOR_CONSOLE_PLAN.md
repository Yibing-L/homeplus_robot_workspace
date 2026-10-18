# HomePlus Operator Console: Implementation Plan

Goal: replace the 5–7 terminal workflow (`HOMEPLUS_CHEATSHEET.md`) with one browser UI that **never shows a state that isn't true** and **makes it hard to run the robot in a bad state**.

Grounded in branch `ev-closed-loop-pickup-fixes` (`homeplus_closed_loop`, HEAD `3201286`). Every topic, parameter and log string below was checked against the code. Where the plan depends on something the code doesn't provide yet, it says so.

---

## 0. Summary of decisions

| Decision | Choice |
|---|---|
| Architecture | **One Python backend process on the ROS machine** (`rclpy` node + FastAPI/uvicorn + WebSocket) serving a static React UI. No rosbridge. Foxglove optional, for deep-dive visualization only and never for control. |
| Source of truth | All robot state lives in the backend. The backend derives it from live topics, with a receipt time and age for every signal. The browser only renders snapshots, so it can't hold its own idea of the state. |
| Process supervision | `systemd-run --user` transient units, one per component. Gives real "alive", restart, exit code and a journal per component, and survives a console restart. |
| Biggest structural fix | **Run the Arduino bridge separately from MoveIt** (`gesture_control.launch.py run_arduino:=false` plus its own bridge unit). Restarting MoveIt then no longer resets the Arduino or base odometry. Restarting the bridge counts as a **re-zero**, and only the guided flow can do it. |
| Executor changes | Small and additive: a structured `executor_state` JSON (latched + 2 Hz), a plan preview JSON, approval scoped to a plan id, a cancel service, and a command request token. All existing topics keep working. |
| E-stop | **The software can't observe it.** The UI never says "E-stopped". It shows *Motor power: verified / unverified / suspect* from observed motion, and operator declarations are labelled as declarations. A hardware sense line is an open question. |

---

## 1. Architecture

### 1.1 Options weighed

| | Custom backend (rclpy + FastAPI) + web UI | rosbridge + browser roslibjs | Foxglove Studio panels | RViz plugin |
|---|---|---|---|---|
| Accuracy | **Best.** One process owns every subscription, receipt time and derived state, and every tab sees the same snapshot. | Each tab derives state on its own. Latched QoS and staleness are handled per browser, and a reconnect loses history. | Good per-topic freshness, but state-machine logic would live in extension panels (TypeScript) on each client. | Same single-display problem as today. |
| Safety gating | Enforced on the server: actions are checked against the state at execution time. | The browser publishes raw topics, so any tab can send anything. | Panels publish raw topics, so there's no server-side gate. | Qt-only, on the VM screen. |
| Process supervision | Yes (backend runs `systemctl --user`). | No | No | No |
| Mac → VM network | One HTTP port. Images are recompressed to JPEG on the server. | rosbridge pushes raw `sensor_msgs/Image` as JSON/CBOR, which is heavy. | foxglove_bridge works well over the network. | Needs X forwarding or VNC. |
| Student dev effort | Python (already known) + small React app. | Low at first, but logic sprawls in JS. | Learning the extension API. Good 3D for free. | High (C++/Qt). |

**Pick: custom backend.** Accuracy and server-side gating are the top requirements, and only the backend option gives one authoritative state machine that can also start and stop processes. Foxglove (`foxglove_bridge`) can optionally run as another supervised unit for 3D/TF inspection. The console links to it, but it is view-only by convention and the console never depends on it.

### 1.2 Process topology

```
Browser (Mac or laptop) ──HTTP/WS :8080──►  homeplus_console (ROS machine)
                                            ├─ rclpy node "operator_console" (MultiThreadedExecutor, own thread)
                                            │    subscriptions → SignalStore (value, recv_mono, header_stamp, rate)
                                            ├─ StateMachine (pure function of SignalStore snapshot + now)
                                            ├─ ActionService (gate → audit → publish/call → verify)
                                            ├─ Supervisor (systemd-run --user units, journalctl readers)
                                            └─ FastAPI: /ws (snapshot 4 Hz + events), /api/actions/*, /img/*
systemd --user units: hp-tf, hp-camera-rx (Mac profile) | hp-camera (laptop), hp-vision,
                      hp-moveit, hp-bridge, [hp-foxglove]
Mac (secondary profile only): mac_agent (phase 3) → mac_sender, AWS tunnel, aws ec2 start/stop
```

- The backend binds to `127.0.0.1:8080` by default. In the Mac profile it binds to the host-only address `192.168.64.2:8080`, which only the Mac can reach. Yibing's laptop uses `localhost`. The robot is never exposed on lab Wi-Fi without an explicit flag.
- **Every snapshot is a full state.** The server sends the full derived state (about 5–10 KB of JSON) every 250 ms instead of diffs. A lost or reordered message can't leave the UI wrong; the next snapshot corrects it. Events (audit entries, timeline lines, action results) are a separate append-only stream with sequence numbers, and the client re-requests anything it missed after a reconnect.

---

## 2. Signals: what the console subscribes to (verified names)

Every signal is stored as `Signal{value, source, recv_mono, header_stamp, count, rate_hz_10s}`. **Age is always measured from receipt time on the console's monotonic clock.** Header stamps are only used for checks like skew.

| Signal | Topic / source | Type / QoS | Fresh if | Notes from code |
|---|---|---|---|---|
| Hardware joints | `/hardware/joint_states` | JointState | age < 0.5 s | Published once per firmware `STATE4` line. Joints: `base_x, base_y, base_theta, joint_grip_L, joint_hand, joint_wrist, joint_elbow, joint_shoulder` (+4 wheel zeros). Executor itself rejects state older than 0.5 s. |
| Bridge ready | `/arduino/ready` | Bool, TRANSIENT_LOCAL | received < 6 s **and** joints fresh | Republished every 5 s by `_report_connection`. Goes false on any `_fail`. |
| Arm still | `/arduino/status` | Bool, TRANSIENT_LOCAL | valid only while joints fresh | Published only on change, so a latched value can outlive the bridge. Never show it unless joints are fresh. |
| Hardware acks | `/arduino/ack` | String | event | `ACK/DONE <id>`, `ERROR <id> <reason>`. Reasons include firmware `TRACKING_ERROR dx_cm=…`, `SETTLE_TIMEOUT`, and bridge-side `HOST_STOP`, `STALE_FEEDBACK`, `HOST_TIMEOUT`, `RX_ACK_TIMEOUT`, `FIRMWARE_RESTART`, `SERIAL …`, `NOT_READY_OR_BUSY`, `INVALID_COMMAND …`. |
| Executor status text | `/gesture_control/executor_status` | String | event | Existing strings: `Starting sequence: …`, `[seq] step i/n: name`, `Plan for X is waiting for approval`, `Sequence X ABORTED at step: reason`, `Command X FAILED: …`, `Busy — ignoring gesture …`. |
| Executor busy | `/gesture_control/executor_busy` | Bool, volatile | — | **Only published on change and not latched**, so the console can't know it at startup. This is why §7.1 is needed. |
| Executor state (new) | `/gesture_control/executor_state` | String JSON, TRANSIENT_LOCAL + 2 Hz | age < 1.5 s | See §7.1. |
| Plan preview (new) | `/gesture_control/plan_preview` | String JSON, TRANSIENT_LOCAL | per plan_id | See §7.2. |
| Planned path (existing) | `/display_planned_path` | moveit_msgs/DisplayTrajectory | per plan | Fallback preview source if §7.2 isn't merged yet. |
| Detector diagnostics | `/molmopoint/diagnostics` | String JSON | see §3.3 | Keys: `stage` (`inputs`, `motion_gate`, `publish_gate`, `geometry`, `done`), `reason`, `detail`, `frame_age_sec`, `rgbd_skew_sec`, `robot_moving`, `backend` (`local`/`remote`), `cycle_sec`, plus per-target keys. |
| Detection pose | `/molmopoint/task_{1,2}/pose`, `/molmopoint_pose_world` | PoseStamped | per sample | Feeds the detection window (§4.2). |
| Debug image | `/molmopoint/debug_image` | Image | age < 3/inference_rate | Already has `stage: reason` drawn on it. |
| Camera color / depth | `/cameras/d405/color/image_raw`, `/cameras/d405/aligned_depth_to_color/image_raw`, `/cameras/d405/color/camera_info` | sensor-data QoS | age < 1 s | In the Mac profile, `vm_receiver.py` **stamps frames with VM receipt time**. Header age therefore can't show latency on the Mac side; only rate and frozen content can (hash of a 32×32 thumbnail, flagged if it stays identical for ≥ 3 s). |
| Detector task | `/molmopoint` parameter `task_id` | GetParameters service, polled every 2 s | — | Same check `task.sh execute` does. |
| MoveIt | `/move_action` action server, `/compute_ik`, `/check_state_validity` | readiness poll every 2 s | — | `ActionClient.server_is_ready()` and `client.service_is_ready()`. |
| TF | `world → d405_color_optical_frame`, `world → base_link` | tf2 | lookup OK and transform age < 1 s | |
| Logs | `/rosout` | rcl_interfaces/Log | — | All node logs with node names. No file tailing is needed for the timeline. |
| Command sources | `get_publishers_info_by_topic('/gesture_control/command')`, same for approve and `/arduino/command_queue` | graph query every 2 s | — | Shows **every other publisher** (e.g. `gesture_behavior` from `gesture_control.launch.py`, or a forgotten `ros2 topic pub` terminal). |
| Remote GPU | `GET {remote_inference_url}/health` | HTTP, every 5 s, 3 s timeout | 200 within 10 s | Mac profile only. |
| Processes | `systemctl --user show hp-* -p ActiveState,SubState,MainPID,ExecMainStartTimestampMonotonic,NRestarts,Result` | every 1 s | — | |

---

## 3. State model

### 3.1 Principles

1. **Derived, not stored.** `derive_state(snapshot, now) -> ConsoleState` is a pure function. The only persistent state is what can't be observed: operator declarations, the odometry origin confirmation, acknowledged faults and the audit log. Each of those is recorded with who, when and why.
2. **Three-valued truth.** Each displayed fact is `true / false / unknown(reason, last_value, age)`. Stale data renders as **UNKNOWN** in grey, with "last seen 12 s ago: 0.78 m" in small type. It never shows the old value as if it were current.
3. **Priority order.** The top-level state is the first rule that matches, so the most dangerous condition always wins the headline.

### 3.2 Robot (hardware + executor) state

Thresholds are config values; the defaults are below.

| # | State | Entered when (signals) | Exits when |
|---|---|---|---|
| 1 | **CONSOLE LINK LOST** (client-side) | No snapshot for 1.5 s. | Snapshot resumes. The UI greys out everything, disables all buttons and shows "Values frozen. Use the physical E-stop." |
| 2 | **BRIDGE DOWN** (sub-reason) | `hp-bridge` not `active`, **or** `/hardware/joint_states` age > 1.0 s. Sub-reasons come from bridge `/rosout`: `Arduino serial port is not connected; retrying.` → *USB not attached*; `No fresh firmware state feedback …` → *port open, no firmware telemetry*; `Arduino serial failure: …` → *serial error*. | Fresh joints for 1 s. |
| 3 | **ODOMETRY ORIGIN LOST** | An Arduino reset was observed without the guided re-zero: bridge unit restarted, `FIRMWARE_RESTART` / `SERIAL` ack, `/rosout` `Connected <port> … waiting for firmware state feedback` (serial reopen resets the Mega), or the new `bridge_info.session_id` changed (§7.4). Shows the **base_x/base_y the robot believed at the moment of reset**. | Guided re-zero flow (§5.3) completes, or the operator explicitly accepts "current spot is the origin", recorded in the audit log with a warning. |
| 4 | **HARDWARE FAULT** (sticky) | `/arduino/ack ERROR … TRACKING_ERROR / SETTLE_TIMEOUT / STALE_FEEDBACK / RX_ACK_TIMEOUT / HOST_TIMEOUT` since the last DONE. | Operator acknowledges **and** a subsequent command reaches DONE with `endpoint verified`. |
| 5 | **MOTOR POWER SUSPECT** | Any of: (a) bridge session started ≥ 8 s ago and the arm never reached upright (`|joint_elbow|`, `|joint_shoulder|` > 3°) — firmware `setup()` commands elbow/shoulder 0 at every boot; (b) elbow ≤ −34° or shoulder ≤ −29° (folded or sensor-clamped limits) while ready; (c) idle drift: with the executor idle, elbow/shoulder > 2° from the last DONE pose, or base > 2 cm / 2° from the held target; (d) fault from row 4 with TRACKING/SETTLE. For (c) on the base: **"Base is X cm from its held target. When motors are powered it will drive back."** | Arm observed upright and still for 2 s, and no drift. |
| 6 | **AWAITING APPROVAL** | `executor_state.phase == "awaiting_approval"` (fresh). | Phase changes. |
| 7 | **RUNNING** `command · step k/n · step_name · phase` | `executor_state.busy` (fresh). Phases: `detecting`, `planning`, `executing`, `verifying`, `gripper`. | Not busy. |
| 8 | **ABORTED** (sticky until acknowledged) | `executor_state.last_result.outcome ∈ {aborted, failed, cancelled}` with `step` and `reason`. | Operator acknowledges. |
| 9 | **EXECUTOR UNKNOWN** | `executor_state` stale > 1.5 s (node dead or MoveIt unit down). | Fresh again. |
| 10 | **IDLE · NOT READY** (list of blockers) | Executor idle but any precondition in §3.4 fails. | — |
| 11 | **IDLE · READY** | All preconditions hold. | — |

**Motor power indicator** (always visible, separate from the state): `VERIFIED` (a joint moved toward a commanded target in the last command, or the arm was observed rising after a bridge start), `UNVERIFIED` (nothing has moved since the bridge started), or `SUSPECT` (row 5). **The console never shows "E-stop released". It can't know that.**

### 3.3 Perception state (shown next to the robot state, gates Start)

| State | Rule |
|---|---|
| CAMERA DOWN | Color or depth age > 1 s, or the receiver/camera unit isn't active. |
| CAMERA FROZEN | Thumbnail hash unchanged for ≥ 3 s while rate > 0. This catches a hung sender that keeps republishing. |
| DETECTOR DOWN | `hp-vision` inactive, or no diagnostics for > max(10 s, 2/inference_rate + remote_timeout_sec + 5). That is 37 s with the remote default `remote_timeout_sec: 30`; it shows "slow" in amber from 10 s. |
| GPU SERVER DOWN | Remote profile, `/health` failing. |
| DETECTOR WAITING | `stage == inputs` (`waiting_for_color_or_camera_info`, `no_depth_frame`, `rgbd_skew`). |
| DETECTOR PAUSED | `stage == motion_gate` (`robot_moving`, `frame_before_motion_done`, `motion_status_never_received`, `motion_status_never_still`) or `publish_gate` (`frame_within_motion_settle`, `frame_stale`, `frame_from_future`). `frame_stale` / `frame_before_motion_done` lasting > 10 s while the arm is still is escalated to **red "detector is looking at old frames"**. This is the 7-minute-stale failure. |
| NO TARGET | `stage == geometry` (e.g. `sam2_returned_no_mask`, `world_transform_failed`) or the model said none. |
| TARGET UNSTABLE | Window gate (§4.2) fails. |
| TARGET OK | Window gate passes. |

### 3.4 Allowed actions by state

The server re-checks these at execution time. The UI also disables the buttons, but that isn't relied on.

| Action | Allowed when | Confirmation |
|---|---|---|
| **STOP** | **Always**, from any client including view-only, even with stale data | None |
| Approve plan `<id>` | AWAITING APPROVAL, `plan_id` matches, plan preview received, controlling client | Hold-to-approve 1 s |
| Reject plan | AWAITING APPROVAL | None |
| Cancel sequence | RUNNING or AWAITING APPROVAL | None |
| Start pickup (task 1) / Press button (task 2) | IDLE·READY + perception TARGET OK + preflight checklist (§5.2) all green + detector `task_id` matches + no foreign command publishers (or acknowledged) | Modal summarising the target and the first unapproved motion ("base will drive ≈ 0.42 m forward without approval") |
| Home | IDLE (ready, joints fresh), not HARDWARE FAULT, not ODOMETRY ORIGIN LOST (home drives the base to (0,0,0), the reset origin) | Modal with "base will move X m" |
| Gripper open/close | IDLE, bridge ready | Single confirm |
| Clear octomap (`/clear_octomap`, std_srvs/Empty) | Executor idle (it clears the map itself during pose plans) | None |
| Start/stop/restart component | See §5.1. `hp-bridge` restart **only** through the re-zero flow. | Per component |
| Acknowledge fault / abort | Sticky state present | Required text "I've inspected the robot" checkbox |

---

## 4. Screens

A single page, laid out for a 1440×900 laptop screen. There are no tabs for safety-critical things: STOP, the state headline and the approval panel are always visible.

```
┌────────────────────────────────────────────────────────────────────────────┐
│ [■ STOP]  ROBOT: AWAITING APPROVAL · pick_up_coffee 6/9 align_refined_grasp │
│           PERCEPTION: TARGET OK   MOTORS: VERIFIED   ORIGIN: confirmed 14:02 │
│           controlling: mac-browser (you)   console link ● 0.2 s              │
├───────────────┬───────────────────────────────┬────────────────────────────┤
│ HEALTH        │ CAMERA + DETECTION            │ SEQUENCE TIMELINE          │
│ hp-tf    ● 3m │ [debug image, age 1.2 s]      │ 1 open_gripper     ✓ 0.9s │
│ hp-vision ●   │ stage: done / pose_published  │ 2 retract_arm      ✓ verif │
│ hp-moveit ●   │ window: 4/4 ok, spread 1.1 cm │ 3 move_to_pregrasp ✓ verif │
│ hp-bridge ●   │ z 0.762 m (band 0.70–0.85)    │ 4 refine_target    ✓ 3 smp │
│ joints 25 Hz  │ history ▁▁▁▁ x/y/z sparkline  │ 5 align_refined    ⏳ APPROVE│
│ ready ✓ still✓│                               │ … ABORT reasons inline     │
├───────────────┴───────────────────────────────┴────────────────────────────┤
│ APPROVAL: align_refined_grasp  plan #17   ⏱ 41 s left   [Hold to APPROVE] [Reject] │
│ base Δ +0.03 m, −0.01 m, +1.2°; shoulder 0→38°; elbow 0→71°; 2.4 s; 31 wp; TCP end (0.81,0.02,0.79) │
│ [top-down base view + joint bars]  [3D preview (M3)]  [open in Foxglove]   │
├────────────────────────────────────────────────────────────────────────────┤
│ MANUAL: Home · Gripper open · Gripper close · Clear octomap   | PREFLIGHT ✓✓✓✗ │
│ LOGS: [executor][bridge][vision][moveit] filter: important ▾               │
└────────────────────────────────────────────────────────────────────────────┘
```

### 4.1 Health dashboard
One row per component, showing:
- unit state, uptime and restarts
- **observed** health: topic rate and age next to process state, where both must be green for green
- last error line from the journal
- start/stop/restart buttons

Liveness and health are kept separate. Process alive with topics stale shows amber "running but silent".

### 4.2 Camera + detection
- Debug image as JPEG over WS, encoded by the backend at up to 2 Hz with **its age stamped beside it**. After 3 s it's greyed out with a STALE watermark.
- Diagnostics: `stage/reason/detail`, `frame_age_sec`, `cycle_sec`, `backend`.
- **Detection window.** The last N = 10 poses of the active task from `/molmopoint/task_{id}/pose` are transformed to `world`. Gate (configurable) = last ≥ 3 consecutive cycles all `pose_published`, all within the last 20 s, max pairwise distance < 3 cm, z inside `[z_min, z_max]` (default 0.70–0.85 m; needs calibration, see §9), all finite, and no cycle between them ended in a non-`done` stage.
- Shows the spread, a z band chart and an "alternating targets" detector: 2 clusters > 10 cm apart within the window means **"two candidate objects"**. This targets the two-red-objects failure.
- **This gate only covers starting.** The executor's own mid-sequence `refine_target` uses its own settings, so §7.5 also tightens the executor config.

### 4.3 Sequence timeline
- Built from `executor_state.steps[]` (status per step: pending / running / awaiting_approval / done / skipped / failed, with timings), so it doesn't depend on log parsing.
- Each step expands to its `/rosout` lines from `gesture_motion_executor` and `arduino_bridge`, filtered by regex to the useful ones: `Step \d+/\d+`, `measured endpoint verified`, `DONE without a settled measured endpoint`, `IK preflight .*: \d+ valid, rejected=`, `ABORTED at`, `Invalid state for .*: contacts=`, `no valid IK goal`, `Cartesian approach incomplete`, `Arduino execution error: ERROR`, `MoveIt failed for`.
- A "show all" toggle shows the unfiltered lines.
- An abort is shown as a red card: **step, reason, the relevant Arduino ERROR code, and a suggested check** from a small lookup table (e.g. `TRACKING_ERROR` → "check E-stop / power switch / battery clip").
- MVP fallback before §7.1 lands: parse `executor_status` strings and mark the timeline "derived from status text".

### 4.4 Approval panel
- Appears, and plays a sound, when AWAITING APPROVAL.
- Shows: step name, `plan_id`, **countdown computed from the executor's `deadline`** (not a browser timer started on arrival), and the summary from §7.2:
  - base Δx/Δy/Δθ, per-joint start→end and max excursion
  - duration, waypoint count
  - end TCP pose, and distance from TCP end to the detected target
  - whether the target came from a fresh detection, and its age
- Renders a top-down 2D view (base footprint start/end, TCP path projected, detected cup) and joint bars. This is cheap and exact. The 3D URDF preview comes in M3.
- Approve is disabled until the preview for **this** `plan_id` has arrived. It's sent as `{plan_id, approve, operator}`.
- **If no controlling client is connected for 5 s while awaiting approval, the backend sends reject.** The executor would time out anyway; this just makes it explicit and logged.
- Steps that run *without* approval still get their preview shown in the timeline as "auto-approved by config", so the operator can see what moved.

### 4.5 Manual commands + preflight
Manual buttons follow §3.4. The preflight checklist (§5.2) sits next to Start, and each failed item links to the panel that fixes it.

### 4.6 Logs
One pane per unit from `journalctl --user -u hp-X -o json -f`, plus a filtered "important" view. Logs can be downloaded per run.

---

## 5. Process supervision and guided flows

### 5.1 Components (profile-driven)

`config/profiles/laptop.yaml` and `config/profiles/mac_vm.yaml` declare each unit's command, env and health checks. Every unit gets `source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash`. The Mac profile also gets `FASTRTPS_DEFAULT_PROFILES_FILE=~/d405_fastdds.xml`. The console sets env explicitly per unit instead of relying on shell rc files.

| Unit | Command | Healthy when | Notes |
|---|---|---|---|
| `hp-tf` | `ros2 launch homeplus_urdf_description state_publisher.launch.py use_gui:=false publish_joint_states:=false` | TF lookup OK | |
| `hp-camera` (laptop) / `hp-camera-rx` (Mac) | laptop: realsense via `vision_pipeline` `enable_d405:=true`; Mac: `python3 ~/d405_vm_receiver.py` | color/depth fresh, not frozen | Mac sender runs on macOS: observed only in M1–M2, controlled by `mac_agent` in M3. |
| `hp-vision` | `ros2 launch homeplus_vision vision_pipeline.launch.py side_mount:=left task_id:=<id> enable_target_detection:=true enable_gesture_detection:=false … enable_d405:=<profile>` | diagnostics fresh, `task_id` param matches | Task switch = set the `/molmopoint task_id` param (executor already does this). Changing it doesn't need a restart. |
| `hp-moveit` | `ros2 launch homeplus_moveit_config gesture_control.launch.py run_arduino:=false launch_obstacle_cloud:=false use_octomap:=true use_rviz:=<profile>` | `/move_action` ready, `executor_state` fresh | **`run_arduino:=false` already exists in the launch file.** Restarting MoveIt is now harmless to odometry. `gesture_behavior` is also in this launch; see §9. |
| `hp-bridge` | new `arduino_bridge.launch.py` (wraps the existing `arduino_reader.py` `--port --baud` node, same node name `arduino_bridge`) | joints fresh, ready | **Restart = Arduino reset = odometry re-zero.** Only the §5.3 flow can restart it. Plain "Stop" is allowed (it publishes STOP first). |
| `hp-foxglove` (optional) | `ros2 launch foxglove_bridge foxglove_bridge_launch.xml` | port open | View only |

Supervisor implementation:
- `systemd-run --user --unit=hp-X --collect --property=KillMode=control-group --property=TimeoutStopSec=10 -E … bash -lc '<cmd>'`.
- Stop uses `systemctl --user stop`; status comes from `systemctl show`.
- The whole process group is killed, so a `ros2 launch` can't leave orphans behind. That was a real source of "cancelled but still running".
- Start is **idempotent**: if the unit is already active, return `already_running` with its PID and start time.
- Ordering is enforced by readiness, not sleeps: start `hp-tf` → wait TF → `hp-camera*` → wait frames → `hp-vision` → `hp-moveit` → wait `/move_action` → bridge (via the flow).
- "Start all" is a guided sequence that shows each step's observed result.

### 5.2 Preflight checklist (gates Start)

Auto-checked items, all live:
- bridge ready, joints fresh
- arm still
- motor power VERIFIED
- origin confirmed
- no HARDWARE FAULT or unacknowledged ABORT
- executor idle and fresh
- MoveIt ready
- TF OK
- camera fresh and not frozen
- detector `task_id` == requested task
- detection window gate passes
- no foreign publishers on `/gesture_control/command` or the approval topics
- GPU `/health` OK (remote profile)
- disk space for trajectory logs

Operator-declared items, each a checkbox recorded in the audit log and reset after every run or abort:
- "one target object in view"
- "area clear"
- "robot at start spot" (unless the origin was just confirmed in the last 10 min)

### 5.3 Guided "re-zero at start spot" flow

The flow is a server-side state machine. Each step **observes** before it advances, and the UI shows what it is waiting to observe.

1. **Preconditions:** executor idle. If not, require STOP + cancel and wait for idle.
2. **"Press the E-stop."** Operator confirms: *declared, unverifiable*.
3. **"Move the robot to the start spot (tape mark)."** Operator confirms.
4. Console publishes STOP, then restarts `hp-bridge`. It observes: unit restarted, new bridge session (new `session_id` or a `Connected …` rosout line), fresh joints, and `base_x, base_y, base_theta` within 1 mm / 0.1° of 0.
5. **Cross-check the declaration.** The firmware drives the arm upright at boot.
   - Elbow/shoulder stay folded for 5 s → "consistent with E-stop pressed ✓".
   - **The arm rises → "Motors are powered. The E-stop was NOT pressed."** The console shows this in red and records it.
6. **"Stand clear. Release the E-stop. The arm will rise to upright."** It observes the arm reach |elbow|, |shoulder| < 3° and still within 10 s, and the base staying within 1 cm of 0.
   - Arm doesn't rise → MOTOR POWER SUSPECT with the checklist "E-stop / power switch / battery clips".
7. Origin is marked confirmed (`origin_confirmed_at`, operator, bridge `session_id`). It invalidates automatically on any new session id.

There's also a smaller **"Reconnect Arduino" flow** for USB drop-outs. Because reopening the port resets the Mega, it routes into this same flow. The UI says why.

---

## 6. Safety and accuracy design

### 6.1 STOP
- A fixed red button on every view, also bound to the `Esc` key, allowed from view-only clients. It never asks for confirmation.
- The backend:
  1. publishes `/arduino/stop` `Bool(true)` three times, 20 ms apart, with reliable QoS
  2. calls `/gesture_control/cancel` (§7.3) if available
  3. writes an audit entry
- **It verifies the effect:**
  - within 1 s joint velocities ≈ 0, or no motion was active
  - the executor goes idle within 3 s, or `last_result.outcome == cancelled`
  - an `ERROR <id> HOST_STOP` ack arrives if a command was active
- The result shows as "STOP confirmed: motion halted, sequence cancelled". **If that doesn't happen in time, it shows "STOP NOT CONFIRMED. Use the physical E-stop."**
- If the bridge is down, the button still works but says "bridge down: STOP could not reach the Arduino. Firmware watchdog halts queued motion after 1.5 s without host heartbeat."
  - This comes from the firmware: `commandCount > 0 && current_millis - lastHostMillis > 1500`.
- The UI shows the hardware E-stop location reminder whenever the console link is lost.

### 6.2 Actions are gated, audited, verified and idempotent
- Every action is `POST /api/actions/<name>` with a client-generated `request_id`.
- The server:
  1. **dedups** by `request_id`
  2. checks the §3.4 table against the *current* derived state
  3. writes `audit.jsonl` `{t_wall, t_mono, operator, client, action, args, state_before, result}` **before** sending
  4. sends
  5. tracks verification
- The UI shows `sent → accepted → confirmed` or `→ not observed within Ns`.

Verification per action:

| Action | Confirmed when |
|---|---|
| Start | `executor_state.request_token` matches within 2 s and `busy=true` |
| Approve | Phase leaves `awaiting_approval` and the step moves to `executing` |
| Home | Executor busy with `command == home` |
| Clear octomap | Service returned |

If the executor reports the command was rejected (`last_rejected.token`), the UI says **"NOT STARTED: executor busy with X"**. This prevents the "cancelled terminal still executed" confusion: the console only reports what it observed.

### 6.3 Single controller
- One browser session holds the **control lease**, renewed by its WS heartbeat every 1 s and expiring after 3 s. Other sessions are view-only plus STOP.
- Taking control needs a confirmation and is audited.

### 6.4 Disconnects mid-sequence
- **Bridge loses joints during motion.** The bridge itself fails with `STALE_FEEDBACK` after 1 s and the firmware watchdog stops. The console shows BRIDGE DOWN plus **sequence outcome UNKNOWN until the executor reports**. It doesn't guess "aborted". It also sends STOP, which is harmless.
- **Camera/detector drops during a sequence.** Shown as a perception warning, with no auto-action. The executor's detect steps time out on their own.
- **Console backend crash.** Units keep running (systemd). The robot keeps the executor's own safety: the approval timeout rejects. On restart the console re-adopts units, re-reads latched topics, and starts with origin **unconfirmed unless the bridge `session_id` matches the persisted one**.
- **Browser drop while awaiting approval.** The backend rejects after 5 s with no controlling client (§4.4).

### 6.5 Accuracy rules for the UI code (enforced in review and tests)
- Every rendered value comes from a `Signal` with `age` and `source`. Hovering shows the topic, the last receipt time and the rate.
- There are no client-side defaults that look like data: no `0.0` placeholders, only "—".
- Colours: green means verified fresh, amber means degraded or slow, red means a bad state, grey means unknown.
- Counters (countdowns) are derived from server-provided deadlines. When the link is lost they show "?".
- Latched values (`/arduino/status`, `/arduino/ready`) are rendered only while their liveness partner (joints) is fresh.

---

## 7. Required changes to existing ROS code

All changes are additive and backward compatible. Each is small and testable on its own.

### 7.1 Executor: structured state (`gesture_motion_executor.py`, ~150 lines)
- Add an `ExecutorReporter` helper that publishes `/gesture_control/executor_state` (String JSON, TRANSIENT_LOCAL depth 1). It republishes on every change and at 2 Hz from a timer.
- Fields:
  ```json
  {"seq": 812, "busy": true, "command": "pick_up_coffee", "request_token": "1734542101.123",
   "sequence": {"name": "pick_up_coffee", "step_index": 5, "step_count": 9, "step_name": "align_refined_grasp",
                "step_type": "pose", "phase": "awaiting_approval"},
   "steps": [{"name": "open_gripper", "status": "done", "t_start": ..., "t_end": ..., "note": "endpoint verified"}, ...],
   "approval": {"plan_id": 17, "step": "align_refined_grasp", "remaining_sec": 41.2, "required": true},
   "last_result": {"outcome": "aborted", "step": "detection", "reason": "no stable detection on /molmopoint/task_1/pose within 30.0s", "t": ...},
   "last_rejected": {"token": "...", "reason": "busy"},
   "hardware_ready": true, "holding_cup": false}
  ```
- Hook points:
  - `_try_set_busy` / `_set_busy`
  - the per-step `_publish_status(f"[{sequence_name}] step …")` at line ~1405
  - `_wait_for_trajectory_approval` (set/clear the approval block; `remaining_sec` from the existing `deadline` loop)
  - `_wait_for_arduino_done` (`executing` / `verifying`; note `measured endpoint verified`)
  - `_abort_sequence`
  - the `except` in `_run_command`
  - the sequence-complete path

### 7.2 Executor: plan preview (~60 lines)
- In `_handle_goal_result` right after `_extract_trajectory_waypoints` (and in `_execute_preplanned`), publish `/gesture_control/plan_preview` with:
  - `plan_id` (the existing `PlanContext.request_id`) and `step`
  - `requires_approval`
  - `joint_names`, start/end positions, max excursion, duration, waypoint count
  - base Δ
  - a down-sampled trajectory (≤ 60 points)
  - end TCP pose via the existing `_fk_pose`
- Publish it for auto-approved steps too.

### 7.3 Executor: scoped approval + cancel (~70 lines)
- **Scoped approval.** New subscription `/gesture_control/approve_plan` (String JSON `{plan_id, approve, operator}`). Accept it only if `plan_id == self._active_plan.request_id and awaiting_approval`. Otherwise log `Ignoring approval for stale plan …` and report it in state.
  - Today the `Bool` approval has no plan identity, so a delayed "true" could approve a different plan. The Bool topic stays for CLI use.
- **Cancel.** New service `/gesture_control/cancel` (`std_srvs/Trigger`). It sets `self._cancel_event`, sets the approval response to `False`, calls `_cancel_plan(self._active_plan, "operator cancel")`, publishes `/arduino/stop`, and the sequence loop checks `_cancel_event` before each step and inside detection/approval/Arduino waits. Outcome: `cancelled`.
- **Approval policy.** Optional parameter `approval_policy: config | all_motion` (default `config`, so behaviour is unchanged). When `all_motion` is set, `require_trajectory_approval: false` on a step is ignored, and gripper steps also require approval. The console profile sets `all_motion` by default. This is Yibing's call; see §9.

### 7.4 Arduino bridge: session identity (`arduino_reader.py`, ~30 lines)
- Publish `/arduino/bridge_info` (String JSON, TRANSIENT_LOCAL), updated on change:
  - `session_id` (uuid4 generated each time the serial port opens, i.e. each Mega reset)
  - `port`, `opened_wall`, `compatible`, `protocol` (`STATE4`)
  - `last_error`, `firmware_restarts`
- This is what lets the console detect a **silent** odometry reset from a USB re-plug. `_read_loop` reconnects automatically every 2 s, and that reset currently goes unnoticed.

### 7.5 Config hardening (`gesture_execution_map.yaml` / params)
- For `pick_up_coffee` (sequence level and `refine_target`), set `detection_min_stable_samples: 3` and `detection_stability_tolerance_m: 0.03`. Today both are 1, so a single sample is accepted. Default params already have `detection_stability_tolerance_m: 0.015`.
- Add an optional executor-side `detection_world_bounds` check (reject z outside a band before using a detection). It goes in `_refresh_detection` next to the existing non-finite check. This stops a z = −6.79 m pose server-side, not just in the UI.

### 7.6 New launch file
`homeplus_moveit_config/launch/arduino_bridge.launch.py`: just the existing `arduino_bridge` Node block from `gesture_control.launch.py` (same args). Old usage still works because `run_arduino` defaults to `true`.

### 7.7 Detector (optional)
If the raw model text isn't already in diagnostics, add `model_text` (e.g. "There are none.") so the UI can show *why* there is no target.

The UI's M1 can run against unmodified code. Without 7.1 and 7.3 it falls back to status-string parsing, the Bool approval and STOP-only cancel, and labels this as **degraded mode** in the header.

---

## 8. Build plan

### 8.1 Layout (new ament_python package, repo root)
```
homeplus_console/
  package.xml  setup.py  setup.cfg  resource/homeplus_console
  homeplus_console/
    main.py              # entry: rclpy thread + uvicorn
    ros_io.py            # subscriptions, graph/param/action-readiness polling → SignalStore
    signals.py           # Signal, SignalStore (thread-safe), freshness helpers, injectable clock
    state_machine.py     # derive_state(snapshot, now) -> ConsoleState  (pure)
    actions.py           # gate table, dedup, verify-trackers
    approvals.py         # plan preview cache, lease-loss auto-reject
    detection_window.py  # pose window, spread/z-band/two-cluster gate
    flows/rezero.py      # guided flow state machine
    supervisor.py        # systemd-run/systemctl/journalctl wrapper
    log_filter.py        # /rosout regexes → timeline lines; status-string fallback parser
    audit.py             # append-only JSONL, one file per day + per run
    images.py            # Image → JPEG (cv_bridge + cv2), thumbnail hash
    api.py               # FastAPI routes + WS
    static/              # built frontend (committed build or built in colcon)
  config/profiles/{laptop,mac_vm}.yaml  config/console.yaml (thresholds)
  web/                   # Vite + React + TS source
  sim/fake_arduino.py  sim/fake_detector.py  sim/sim.launch.py
  test/
mac_agent/ (M3)          # FastAPI on macOS: mac_sender, tunnel, aws ec2 start/stop/status
```

### 8.2 Stack
- **Backend:**
  - Python 3.10 (Humble system python), `rclpy`, `fastapi>=0.110`, `uvicorn[standard]` (websockets), `pydantic>=2`, `PyYAML`, `opencv-python-headless` / `cv_bridge` (already present for vision), `httpx` (GPU health)
  - Installed with `pip install --user` into the ROS python. No venv, so `rclpy` imports.
- **Frontend:**
  - React 18 + TypeScript + Vite, `zustand` for the snapshot store, plain CSS
  - M3: `three` + `urdf-loader` for the 3D preview, with the URDF served from `homeplus_urdf_description` by the backend
  - No component framework is needed
- Runtime needs only the built static files. **Node is needed only to build.**
- Start the console with `ros2 run homeplus_console console --profile mac_vm`. Once stable, optionally install it as a `systemd --user` service.

### 8.3 Milestones

| Milestone | Scope | Exit criterion |
|---|---|---|
| **M0 Foundations** (≈3 days) | §7.1, 7.3 (scoped approval + cancel), 7.4, 7.6; `fake_arduino` + `fake_detector`; sim launch | Sim pickup runs end-to-end with approvals over topics; unit tests green |
| **M1 MVP** (≈1.5 weeks) | Backend signal store + state machine + supervisor (tf, camera-rx/camera, vision, moveit, bridge start/stop); STOP with verification; approve/reject with text summary + countdown; timeline (structured, with rosout detail); health panel; audit log; control lease; Start/Home with gating and request-token verification | Full pickup on hardware from the console alone, **no terminals except the Mac sender + tunnel** |
| **M2 Safety depth** (≈1 week) | Camera/detection panel + detection window gate; preflight checklist; re-zero flow; motor-power-suspect detectors (boot-upright, clamp, idle drift); foreign-publisher warning; log panes; §7.2 preview + 2D top-down view; §7.5 config | Every failure mode in §8.5 produces the expected UI state on hardware |
| **M3 Comfort** (≈1 week) | 3D trajectory preview; `mac_agent` (sender start/restart incl. segfault auto-detect, tunnel, AWS start/stop with **running-cost timer and "stop instance" reminder on shutdown**); `approval_policy: all_motion`; per-run report download | Mac profile needs zero terminals after starting the agent |
| **M4 Hardening** | Rosbag regression suite, run on Yibing's laptop, docs | Two clean sessions by a second operator using only the UI |

### 8.4 Sim / dev without hardware
- **`fake_arduino.py`**
  - Opens a pty pair and passes the slave path to the real `arduino_reader.py` via `--port`. The bridge, its validation and its STOP handling are exercised unchanged.
  - Speaks STATE4 at the firmware cadence, plus `READY4`, `RX4`, `BUF3`, `ACK/DONE/ERROR`. It integrates simple first-order joint motion toward the queue targets, and commands upright at "boot".
  - Reuse the host-compiled firmware harness in `tests/test_firmware.py` where possible, for protocol fidelity.
  - Fault flags:
    - `--no-motor-power`: arm stays at −35°/−30°, commands end in `TRACKING_ERROR` / `SETTLE_TIMEOUT`
    - `--disconnect-after S`
    - `--push-base dx`: idle drift
    - `--reset-at S`: new boot, odometry 0
- **`fake_detector.py`**
  - Publishes `/molmopoint/diagnostics`, `/molmopoint/task_1/pose` and a synthetic `/molmopoint/debug_image`. It serves the `task_id` parameter.
  - Scenarios: `good`, `two_targets` (alternate 40 cm), `none` ("There are none."), `stale_frames` (`frame_before_motion_done` forever), `garbage_z` (z = −6.79), `high_first` (first sample +0.20 m).
- **Bags.** Record a session with `ros2 bag record` of every §2 topic (minus raw images, plus debug image). Recordings of the failure day, if any, become regression fixtures.

### 8.5 Test plan
- **Unit (pytest, no ROS needed):**
  - `state_machine` table tests: snapshot fixtures → expected state + allowed actions
  - a frozen-clock test for each staleness threshold (value at t, then t+threshold+ε → UNKNOWN)
  - `detection_window` (spread, z band, two clusters, non-finite)
  - `log_filter` against real executor/bridge log excerpts saved from `~/runlogs/t3_moveit.log`
  - `actions` (dedup, state gate, verify timeout)
  - re-zero flow transitions, including the "arm rose while E-stop declared" branch
- **Executor changes:** extend `tests/` with launch tests in sim:
  - `executor_state` fields per step
  - stale `plan_id` approval ignored
  - cancel during detection, approval and execution → outcome `cancelled`, and STOP published
- **Integration (sim):** `launch_testing` starting fake_arduino + real bridge + executor (MoveIt with `use_rviz:=false`) + fake_detector + console. Then Playwright (Python) drives the UI:
  - start pickup → approve → done
  - `--no-motor-power` → MOTOR POWER SUSPECT, Start disabled
  - `--reset-at` → ODOMETRY ORIGIN LOST
  - kill fake_detector → DETECTOR DOWN within 37 s, and its values show UNKNOWN
  - browser close during approval → auto-reject logged
  - double-click Start → one command sent (dedup)
- **Accuracy property test:** for every displayed signal, stop its publisher and assert the UI shows UNKNOWN within threshold + 0.5 s. Generate this from the §2 table so new signals can't skip it.
- **Hardware acceptance:** reproduce each real failure on purpose and record the observed UI state:
  - E-stop pressed at bridge start
  - power switch off
  - USB unplug (Mac: UTM detach)
  - kill `mac_sender`
  - two red objects
  - push the robot while e-stopped
  - start from close to the cup
  - reject a plan
  - let an approval time out
  - STOP mid-motion

---

## 9. Risks and open questions (for the user / Yibing)

1. **E-stop and motor power can't be observed.** Can we wire the E-stop auxiliary contact or a motor-rail voltage divider to a spare Mega analog pin and add it to the STATE line (STATE5)? That turns "suspect" into "measured" and is the single biggest accuracy gain. Until then the UI infers motor power from motion.
2. **Splitting the bridge from MoveIt** changes the documented T3 procedure. Is Yibing OK with a separate bridge launch? (It's backward compatible: the default is unchanged.)
3. **Approval policy.** `open_gripper`, `retract_arm_*`, `move_to_pregrasp` (base drive), `move_to_grasp`, `lift` and the button steps currently skip approval. Should the console default to `all_motion`? It's safer, but means more clicks per run.
4. **z band and spread thresholds** for the cup need calibration from good runs. Is 0.70–0.85 m right for the coffee-machine table? Does task 2 need a different band?
5. **Does executor `detection_min_stable_samples: 3` slow runs unacceptably** at `inference_rate: 0.5` (≈6 s plus remote latency)? An alternative is to keep 1 at sequence start and rely on the console's pre-start window. The executor-side change is safer because it covers `refine_target` mid-sequence.
6. **`gesture_behavior`** runs inside `gesture_control.launch.py` and can publish to `/gesture_control/command`. Should it be disabled in console profiles? The console will flag it as a foreign publisher either way.
7. **Time sync (Mac profile).** Frames are stamped at VM receipt, so true capture latency is invisible. Should `mac_sender` send the capture time so the receiver can report it, without changing the stamp?
8. **Mac control of `mac_sender` needs `sudo`.** For `mac_agent` to restart it, we need a sudoers `NOPASSWD` entry for exactly that command, or to run the agent as root under launchd. Which is acceptable?
9. **AWS cost control.** Should the console auto-stop the instance after N idle minutes? It needs AWS credentials on the Mac agent, never on the robot machine.
10. **Exposure.** Default binding is localhost or host-only. If the laptop UI should be reachable from another device in the lab, we need at least a shared token and HTTPS. Is that wanted?
11. **Start spot definition.** Is there a physical mark? The re-zero flow assumes one. Could the console verify it, e.g. a fiducial seen by the D405?
12. **Risk: the console becomes another thing that can crash.** Mitigations: units run under systemd independently of the console, and the executor keeps its own approval timeout. The physical E-stop stays the primary stop; the console STOP is secondary and says so.
13. **Risk: false "suspect" alarms** from the idle-drift rule (e.g. the PWM-estimated wrist and gripper are noisy). Only elbow, shoulder and base, which are measured, feed the drift rule.
