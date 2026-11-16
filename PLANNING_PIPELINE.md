# HomePlus manipulation pipeline

The executor now separates target geometry, goal feasibility, path planning and
hardware completion. A successful motion plan is not evidence of a successful
physical grasp or button activation.

## Perception and state

The bridge is the only hardware joint-state publisher. It publishes received
`STATE2` feedback to `/hardware/joint_states` (executor) and `/joint_states`
(TF/MoveIt). Base values are wheel odometry; shoulder/elbow values are encoder
measurements. Gripper/hand/wrist values remain PWM-based estimates. Wheel slip,
servo backlash and camera/TCP calibration errors still need physical measurement.
Neither a command nor a timeout is published as a measured endpoint. Feedback is
accepted throughout execution. Startup and error recovery require measured
stillness; missing feedback prevents execution.

The detector pairs color with the nearest buffered depth frame (maximum skew
35 ms), captures the image-time TF before inference, and retains the acquisition
timestamp. Motion, settling, stale observations and task changes invalidate
results. The executor sets the detector task and consumes its task-specific pose
topic, rejecting duplicate/old frames. Stability means cluster diameter <=15 mm,
not distance from the median; button refinement currently allows 20 mm. Normals
must also agree within 0.08 rad. These are configurable operational limits, not
measured accuracy claims.

Cup detection uses the former robust masked-depth estimator: retain a near-depth
cluster and take median projected coordinates. Cylinder/taper fitting is no
longer a detection gate. `cup_surface_to_center_m` shifts the detected surface
horizontally away from the camera toward the desired grasp center, preserving
height. Its default is zero until calibrated; restoring detection does not by
itself establish physical goal accuracy. Collision radius/height are explicitly
configured envelopes (65 mm / 150 mm), not measured dimensions. The two saved
rejected observations both produce targets with this estimator.
Camera frames more than 100 ms ahead of ROS time at receipt are rejected with
an explicit clock error; host/camera clock synchronization remains unresolved.
Buttons retain the local plane fit and original model pointing ray.
Other perception-only tasks retain segmented-depth localization.

## Planning stages

1. Detect the target and establish its approach frame. For cups, add measured
   cylinder geometry to the planning scene; allow only finger–cup contact.
2. Open/set the gripper and retract the arm before mobile positioning.
3. Generate and rank collision-checked whole-body IK candidates. Verify their
   fixed-base continuation with IK and a complete Cartesian lookahead.
4. Plan the selected base motion with bounded path constraints and arm fixed.
   Execute it, verify measured completion, then replan the arm standoff from the
   actual reached state. The whole-body path is not planned and then discarded.
5. Reacquire the target. Align the fixed-base arm at the refined standoff before
   imposing orientation constraints on the final approach.
6. Use a complete, collision-checked, joint-jump-checked Cartesian path for
   contact. Partial Cartesian paths never execute. The base remains fixed.
7. For pickup, close the gripper, attach conservative cup geometry, then lift
   and retreat with that geometry. Closure does not prove retention: no gripper
   contact/load sensor is available. An explicit `gripper_open` command detaches
   the carried geometry. A new pickup is rejected while it remains attached.

Approach offsets and tool orientations follow the detected target frame. Base
reach is evaluated in the candidate base frame. Target height comes from detection plus explicit task-relative offsets. There is
no configured tabletop height or implicit TCP height clamp. Observed OctoMap
geometry and robot collision geometry determine whether a path is clear.

RRTConnect handles free-space positioning using validated joint-goal candidates.
Failures identify missing services, invalid starts/collisions, no acceptable IK
candidate, incomplete Cartesian continuation, or path-search failure. Retrying
RRT does not repair an invalid goal. Final serialized trajectories are checked
again, including intermediate states and endpoint/path constraints, before send.

The base still uses two prismatic joints and a continuous yaw joint in the URDF.
The workspace box does not narrow those prismatic bounds; explicit base path
constraints bound mobile stages. Collision-check resolution is configured for
all manipulation groups. Acceleration limits are conservative initial values
and need benchmarking against measured hardware capability.

## Scene lifecycle

Clouds retain image timestamps and require acquisition-time transforms; there is
no latest-TF fallback. Motion invalidates old masks. The target-aware cloud waits
for a valid mask and reports its acquisition stamp. Map preparation waits for a
fresh masked cloud after clearing, rather than assuming a fixed sleep rebuilt
the scene. That acknowledgement confirms cloud publication, not atomic insertion
inside MoveIt; a short insertion allowance remains. Lift/retreat reuse the
prepared world scene instead of clearing unseen obstacles again.

The former 4/8 cm target-neighborhood erasure is disabled. Mask dilation is
limited to two pixels. Nearby table/appliance geometry remains collision-active.
Persistent calibrated table/appliance primitives and dynamic obstacle monitoring
are still useful future work; current collision geometry is depth-based plus the
explicit cup.

## Firmware protocol v2 — required together with the new bridge

The source is `ARDUINO_FIRMWARE_UPDATED.txt`. Save it as an Arduino sketch and
build/upload it for the actual board using the installed hardware libraries.
No firmware was flashed by this change. The bridge rejects legacy firmware.

ROS command envelope:

```
CMD <uint32 id> <x_cm y_cm yaw_deg grip_deg hand_deg wrist_deg elbow_deg shoulder_deg 0 0 time_ms>,...
```

The bridge inserts a CRC-16/CCITT-FALSE field between the ID and queue on the serial
wire. Times start at zero and strictly increase. Firmware interpolates positions
using these times. Large trajectories are split into overlapping bounded queues,
with verified stops at boundaries; they are not truncated. This deliberately
adds pauses and does not reproduce a velocity-controlled trajectory controller.
Both firmware and host enforce queue size, finite numbers, limits and timing.

`STATE2 <id> <eight hardware positions>` reports state at 20 Hz. `DONE <id>`
requires a 500 ms settled dwell; the host also verifies the feedback endpoint.
`ERROR <id> <reason>` aborts the task, except for the executor-side recovery of
TRACKING_ERROR/SETTLE_TIMEOUT described under "Fault recovery" below (a fresh
plan, never a resend). The bridge never converts timeout into
success or automatically retries a possibly executed command. `PING` supplies a
heartbeat; loss for 1.5 seconds during motion stops the queue. `STOP` holds the
measured/estimated state. Large intermediate tracking errors also stop motion.

The tighter encoder settling limits may expose existing arm control/calibration
problems. Correct those from measured residuals rather than relaxing completion
until inaccurate motion appears successful. Button contact force/travel feedback
and grasp retention sensing remain unavailable.

## Fault recovery (executor)

The firmware limits are unchanged: the 3° arm tracking limit, SETTLE_TIMEOUT and
STOP still halt the motion where it is (`stopAtCurrentState`). Recovery only
decides what the sequence does next. It always replans from the measured joint
state and never resends the old trajectory. Decision logic is in
`homeplus_moveit_config/scripts/fault_recovery.py`. The tests are
`tests/test_fault_recovery.py` (pure Python) and
`tests/test_fault_recovery_executor.py` (stubbed executor, needs ROS sourced).

Settings go on a step, or on the sequence for every step that does not set its
own. Each is optional:

| key | default | meaning |
|---|---|---|
| `fault_retries` | 2 | retries per step after TRACKING_ERROR/SETTLE_TIMEOUT (0–5; 0 disables) |
| `fault_settle_timeout_sec` | 10 | how long to wait for the arm to be still and the bridge ready again |
| `start_state_recovery` | true | clear the map and rebuild it once when only map voxels make the start invalid |
| `on_detection_timeout` | `abort` | sequence only: `back_up` runs the `back_up` command once and waits for detection again |

**Motion faults** apply to `pose` (including Cartesian and standoff phases) and
`joint_state` steps inside sequences.
- TRACKING_ERROR or SETTLE_TIMEOUT on a step without `contact_ends_step`: the
  executor publishes `<step>: TRACKING_ERROR — recovering 1/2 at 50% speed`. It
  waits for `/arduino/ready` to come back (the bridge sets ready only after its
  own 0.7 s stillness check) and for 0.5 s of host-measured stillness. Then it
  plans the same step again from the measured state, with velocity and
  acceleration scaling halved on each retry (floor 0.1, never above the configured
  value). Standoff base phases use the same factor.
- The step keeps the same target. A base-relative `joint_state` goal (`back_up`)
  is the absolute goal from the first attempt, so a retry does not add another
  offset. A `joint_state` retry is skipped when the measured state already
  matches the goal.
- Retries go through approval like any other plan. A step that needs approval
  asks again, and an auto-approved step continues by itself.
- After a successful retry the executor publishes `<step>: recovered after N retry`.
  When retries run out, the sequence aborts as before. The reason is the bridge's
  error line followed by `(recovery: TRACKING_ERROR after 2 recovery attempt(s))`.
- These are never retried: `contact_ends_step` steps (a stall there still counts
  as contact), HOST_STOP, SERIAL, STALE_FEEDBACK, HOST_TIMEOUT, FIRMWARE_RESTART,
  NOT_READY_OR_BUSY, RX_ACK_TIMEOUT, DONE without a settled endpoint, and a host
  wait timeout. Recovery is also abandoned, and the sequence aborts, when the
  bridge is not ready again or the arm is not still within
  `fault_settle_timeout_sec`.
- Operator STOP: the executor now serves `/gesture_control/cancel`
  (`std_srvs/Trigger`), which the console already calls with every STOP. The
  service publishes `/arduino/stop`, cancels a plan that is in flight or waiting
  for approval, blocks any recovery retry, and aborts the sequence at the next
  step (`ABORTED at <step>: operator STOP`). The service name is derived from
  `command_topic`, so a dry-run executor gets `/gesture_dry_run/cancel`.
- Gripper steps and single (non-sequence) commands are unchanged.

**Start-state collision**: before planning a `pose` or `joint_state` goal, the
measured start is checked as before. The executor may clear the map only when
all of the following are true: every reported contact involves `<octomap>`,
none involves `detected_table`, there is no self-collision, and no constraint is
violated. It then publishes `<name>: start touches map voxels only (...) —
clearing the map and replanning once`. It clears the map even when the step has
`clear_octomap_before_plan: false`, waits for a fresh masked cloud, re-applies
the active finger and held-cup ACM allowances, and checks the fresh measured
state again. This happens at most once per step attempt. If the start is still
invalid, the step aborts with `start state still in collision after a map clear
and rebuild (<pairs>)`. Any other invalid start aborts with `start state invalid
(<pairs>)` instead of the generic `planning/execution failed`.

**Detection timeout** (opt-in, `on_detection_timeout: back_up` on the
sequence): this covers only the sequence's first detection, before anything has
moved. When it times out, the executor publishes `[<seq>] detection: no stable
detection ... — backing up once and looking again`. It runs `back_up` once,
planned from the measured state with that command's approval setting, and waits
for detection once more. Close-up `detect` steps are not covered, because
backing up would change the view those steps are tuned for.

None of the new status lines match the console's step, approval, abort or
failure patterns (`log_filter.parse_status_text`). The console still records
the firmware fault as a sticky hardware fault for the operator to acknowledge,
even when the retry succeeds.

## Hand the cup to a person (task 9, `hand_cup_to_person`)

Runs only while the cup is attached (the sequence has `held_cup_placement`, like
`place_cup_on_drip_tray`; the console offers task 9 only while holding). Pure
logic is in `homeplus_moveit_config/scripts/handoff_logic.py`, tested by
`tests/test_handoff_logic.py` (no ROS).

Detector tasks (`homeplus_vision/config/task_registry.yaml`):

| task | caption | camera | geometry |
|---|---|---|---|
| 9 | "the person" | side D455 (`camera: d455`) | `person`: median of the masked body's near-depth cluster (depth ≤ `max_depth_m` 4 m), level heading; height 0.1–2.0 m |
| 10 | "the person's open hand, palm up" | hand D405 | `point`: depth at the model's point (palm surface); rejected inside the held-object volume around `grasp_tcp` (`reject_held_object`) and outside 0.5–1.5 m height |

The detector now has an optional second RGB-D input (`secondary_*` parameters,
default the D455 topics). It subscribes to it only when some task sets
`camera: d455`, and the task's camera chooses the input for each cycle. The diagnostics
carry `"camera": "d455"` for those cycles. Task 10 has no command of its own:
`publish_task_pose: true` gives it `/molmopoint/task_10/pose`. If the D455 is not
connected or its mount TF is wrong, change task 9 to the D405 (remove
`camera: d455`, set `max_depth_m: 2.0`) and restart hp-vision.

Steps (`gesture_execution_map.yaml`):

| # | step | type | what it does |
|---|---|---|---|
| 1 | `scan_for_person` | `scan_rotate` | look at the current heading (3 agreeing samples within 10 cm, 10 s), then turn the base in place 30° at a time (left, up to 11 turns = 330°), collision-checked, 20 % speed, no approval; ends when a person 0.5–4 m from the base is stable; aborts after the last heading |
| 2 | `approach_person` | `base_approach` | turn to face them and drive straight until the held cup is 0.65 m from their measured front (base_link 0.65 m + gripper reach); arm unchanged; ≤2.5 m; 20 %; **approval** |
| 3 | `find_hand` | `detect` | switches the detector to task 10; 3 samples within 2 cm; within 1 m of the person reading and 1.1 m of the base |
| 4 | `present_cup_above_hand` | `pose` (base placement) | cup bottom 10 cm above the palm reading; the base may shuffle ≤0.35 m at 20 % to put the hand in the arm plane; arm 20 %; fresh map; **approval** |
| 5 | `lower_cup_into_hand` | `pose` Cartesian, `contact_ends_step` | aim the cup bottom 2 cm below the palm reading at 10 %; a stall counts as contact; held cup and fingers may touch map voxels; **approval** |
| 6 | `wait_for_person_to_take_cup` | `handoff_wait` | hold still; ask the operator to open the gripper (see below) |
| 7 | `release_cup_to_person` | `gripper` 0, `detach_cup` | opens only after step 6 approved it (the validator rejects a gripper opening before a `handoff_wait`) |
| 8 | `withdraw_hand_from_cup` | `pose` Cartesian | open hand straight back 12 cm, 20 % |
| 9 | `back_away_from_person` | `joint_state` base-relative | base 30 cm back, 20 % |
| 10 | `retract_arm_after_handoff` | `joint_state` | arm upright, 30 %, once clear of the person |

**Opening the gripper always asks** (`require_release_approval: true`). The
console shows an approval for `wait_for_person_to_take_cup/open_gripper` with a
zero-motion preview. The button reads "Hold to open the gripper (release the
cup)" and Reject reads "Keep holding". While it waits, the executor measures
evidence and shows it: contact at the guarded lower, and a pull, which means the
shoulder or elbow encoder sits more than 1.5° from its settled value for 0.3 s
(the firmware holds position at idle and raises no fault). The encoder settled
value is the median of the first 0.5 s after the arm is still. With
`require_release_approval: false`, contact (`release_on_contact`) or a pull
opens the gripper without asking. No approval within `timeout_sec` (60 s),
Reject, STOP, bridge not ready or stale joint feedback: the sequence aborts at
step 6, still holding the cup (`... ; still holding the cup`). The operator can
then open the gripper by hand with the console's **Open gripper** (it detaches
the cup) or back away with **Back up**. A dry run skips the wait.

New step types and keys (validated by `_command_config_errors`; using a key on the
wrong step type is an error):

| type | keys (defaults) |
|---|---|
| `scan_rotate` | `step_deg` (30, ≤90), `max_deg` (360), `direction` (left/right), `min_target_range_m`, `max_target_range_m`, `detection_*`, `task_id` + `detection_topic` (switch the detector), speed/approval keys |
| `base_approach` | `standoff_m` (0.65, ≥0.3, held cup to person), `max_drive_m` (2.5), `min_drive_m` (0.05); `velocity_scaling`/`acceleration_scaling` must be ≤0.3 |
| `handoff_wait` | `require_release_approval` (true), `operator_confirm` (true), `contact_from_step`, `release_on_contact` (true), `timeout_sec` (30; the map uses 60), `pull_threshold_deg` (1.5), `pull_min_duration_sec` (0.3), `pull_joints` (shoulder, elbow; encoder joints only), `settle_timeout_sec` (10), `baseline_sec` (0.5) |
| `detect` | may now set `task_id` + `detection_topic` (together) and `min/max_target_range_m` |
| `base_placement` | `max_base_move_m` (drop placements needing more base travel), `base_velocity_scaling` (slow base drive; otherwise standoff base phases run at full scaling as before) |

The first detection before step 1 is skipped when a `detect`/`scan_rotate` step
comes before any `target: detected` step. Existing sequences are unchanged by
this. Base motions in `scan_rotate`/`base_approach` are base-only phases
planned from the measured state with path bounds, sent like any other plan
(approval, validation, endpoint check). TRACKING_ERROR/SETTLE_TIMEOUT recovery
and STOP behave as in the fault-recovery section.

Console: task 9 "Hand the cup to a person" (`TASK_COMMANDS`, `HOLDING_TASKS =
{7, 9}`). Task 9 is also a search task (`state_machine.SEARCH_TASKS`), so the
detection-window preflight and the TARGET_OK start gate do not apply. It may
start whenever the detector is running (TARGET_OK, NO_TARGET, TARGET_UNSTABLE or
DETECTOR_PAUSED). `--sim` has a stub of the sequence. In the stub, the hand-off
wait is an approval.

## Offline validation

```
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover -s tests -v
python3 tests/moveit_smoke.py
```

The regression suite uses ROS message types but starts no nodes or hardware. It
includes a host C++ compilation of the firmware with fake hardware and protocol
fault tests. This does not replace an AVR build, SRAM check, or physical testing.
The integration smoke test uses localhost ROS domain 213, launches MoveIt without
hardware/controllers/RViz, checks all four IK tips and collision validity, and
requests an RRTConnect plan and a fixed-base Cartesian path. It terminates its
own launch process afterward. It is a model/service smoke test, not a benchmark
of real cup/button scenes.

Before live manipulation, confirm firmware v2 telemetry, calibrate camera and
TCP transforms across several arm poses, then measure repeated static-target
localization and endpoint residuals. Tune PID using those recorded measurements.

Validation on this workspace: the ROS packages built and all 27 offline
regressions passed, including the host C++ firmware harness. The isolated MoveIt
functional checks (FK/IK, RRTConnect, Cartesian path and executor startup) passed.
The sandbox blocked local UDP networking; DDS still completed the service tests.
The installed `move_group` process logged a teardown crash after SIGINT/forced
shutdown. Clean MoveIt shutdown remains an unresolved integration issue; the
functional checks do not certify it. Logs for this run are in
`/tmp/homeplus_smoke_result.log` and `/tmp/homeplus_moveit_smoke.log`.

D405 clock configuration: `config/d405_clock.yaml` disables SDK global-time
conversion on `depth_module`; the RealSense ROS driver maps hardware timestamps
to its ROS time base. This is a mitigation for observed future-dated frames on
WSL, not a verified clock calibration. Restart Terminal 2 and verify
`ros2 param get /cameras/d405 depth_module.global_time_enabled` returns false.
Keep future-frame rejection enabled. If skew persists, inspect host clock steps
and stream timestamps rather than increasing TF tolerance or rewriting stamps.
Restart the camera after host time changes or suspend/resume.

Cup collision envelopes are anchored behind the observed front surface, not
centered on the grasp target. Their center offset is applied to the stabilized
sequence observation. Grasp calibration remains independent. This corrects the
reproduced false palm/hand-camera collision caused by centering a 65 mm radius
cylinder on the front surface. `tests/cup_scene_smoke.py` checks the reported
approach with that envelope in an isolated MoveIt scene (no camera OctoMap).
Mobile standoff feasibility now uses continuous collision-checked Cartesian
approach directly, without a redundant isolated endpoint-IK gate. Lift/retreat
are checked during execution in the appropriate attached-object scene.

Measured-base replanning: retain the preflight arm configuration as a preferred
seed, but merge only its arm joints into the measured full state. Accept it only
after checking the original goal tolerances and current collisions. Fixed-base
KDL groups retain full-pose solving with epsilon 0.004; mobile groups retain the
default solver epsilon. Explicit task constraints and final trajectory validation
remain authoritative. `tests/base_settle_smoke.py` exercises millimetre-scale base
settling offsets, FK position error, orientation constraints and Cartesian paths.
Trajectory filenames replace stage separators with underscores.

Buffered hardware streaming (requires the matching STATE4 firmware): each
trajectory is one transaction, with its original MoveIt timestamps. The bridge
validates the full queue, sends 24 initial points, and refills in batches of up to
12 using firmware sequence-numbered capacity reports. The Mega retains one
interpolation anchor and reuses the same 45-point array; packets do not stop or
reset PID correction. CRC covers packet sequence, flags and all positions.
A missed refill stops with BUFFER_UNDERRUN; lost feedback/heartbeats, invalid
packets and tracking errors also stop motion. Only the final packet can settle
and emit DONE. The earlier per-chunk inverse-quintic retiming is removed.

Only retract and coarse cup/button standoff stages opt into endpoint_profile:
transit. Firmware accepts a settled endpoint within 5 mm per base axis, 0.5 degree
yaw and 1 degree shoulder/elbow, then holds the measured configuration. The host
allows telemetry rounding margin (5.1 mm, 1.1 degrees); firmware still enforces
its tighter yaw limit. The next stage plans from actual feedback. Contact and
other motions retain the existing precise firmware bands and host checks.
PWM-only joints remain estimates. No tracking-error limits were relaxed.

Wire format: STR id crc sequence flags points. Sequence begins at zero;
flags bit 0 marks the final packet, bit 1 selects transit acceptance. CRC16-CCITT
covers everything after the CRC field. BUF3 id next_sequence free reports credit.
STATE4 uses the same eight state values as STATE2. Legacy CMD remains for single
gripper commands, but streaming motion is rejected unless STATE4 is observed.
Tests exercise packet order/CRC, credit handling, interpolation across packets,
compaction, final-only completion, underrun and endpoint profiles. These are host
logic tests, not proof of physical tracking or an AVR resource/build check.

Receiver flow control: STATE4 firmware acknowledges each 32 consumed STR bytes
with RX4 <count>. The bridge waits for this credit before writing another block;
heartbeats never enter a partially transmitted packet. Missing credit aborts
rather than retrying motion blindly. CRC still validates the full packet. This
replaces delay-only serial pacing for streamed trajectories. Coarse cup pregrasp
arm speed/acceleration scaling is 0.20 (previously 0.15); contact remains 0.10.
Detection progress reports the actual cluster diameter, orientation spread and
oldest sample age, rather than a cumulative count presented as a completion ratio.

Detection acceptance defaults to one fresh valid observation, including cup/button
refinement. Multi-sample position/orientation consensus is opt-in, not enabled
in the shipped task configuration. The existing masked-depth estimator and
independent goal-point offset are retained.

Refresh acceptance: detection_max_refresh_shift_m defaults to zero (disabled),
including the installed executor parameters. A fresh observation is not rejected
for differing from the earlier view. Positive values explicitly opt into that gate.
Task speed reductions have been removed: MoveIt velocity/acceleration scaling
now defaults to 1.0 for pose, joint and base-phase planning. Shoulder/elbow model
velocity limits are 0.35/0.70 rad/s, matching existing firmware PWM travel rates.
Cartesian timing retains acceleration bounds and uses 0.35 rad/s rather than
its former extra 0.12 rad/s cap. Firmware tracking protection is unchanged.

Base tracking-error tolerance is now 6 cm per X/Y axis during motion (formerly
4 cm). This is distinct from endpoint acceptance. Yaw and arm tracking limits,
endpoint tolerances and trajectory speeds are unchanged by this adjustment.

Coarse transit now uses endpoint-only position-error checking for base, yaw,
shoulder and elbow. Mid-motion lag does not trigger TRACKING_ERROR in that
profile; final error/stillness checks and the settle timeout still apply.
Precise/contact trajectories retain mid-motion tracking checks. CRC, buffer
underrun, host-loss and STOP handling apply to both profiles. This permits greater
deviation from the planned path during transit; it does not improve tracking or
prove collision clearance for the actual path. Existing hardware speed and
acceleration limits remain active.

Removed the support_surface_z_m/minimum_tcp_clearance_m height override and its
sequence inheritance. Old height-clamp configuration keys are rejected at startup.
This removes a target bias; it does not erase observed table collisions.

Collision visualization: invalid states checked with diagnostic logging publish
/planning_debug/rejected_state (DisplayRobotState) and /planning_debug/contacts
(MarkerArray). Red contact spheres and yellow labels identify exact reported
contact locations and body pairs; colliding robot links are highlighted. The
last overlay repeats for late RViz subscribers and clears when a hardware command
starts. These topics do not update measured joints, scene validity, or commands.
The default RViz layout includes blue obstacle points, green segmented target
points, the detection image and mask. These camera displays are live, while the
rejected pose/contact overlay is the last diagnostic snapshot; compare while the
robot/camera remains stationary. OctoMap contacts remain unlabeled environment
voxels, not automatic desk/cup classifications. Disable the overlay displays
when inspecting the actual robot alone.
