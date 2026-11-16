# Prompt: measure and correct the hand camera (D405) mounting error

Paste everything below the line into a new Claude Code session started in
`/mnt/c/users/easha/arl/homeplus_robot_workspace` (fresh terminal, Orchestra on).

---

The hand-mounted Intel RealSense D405 on the HomePlus robot looks rotated: its
roll (rotation about the viewing axis) seems off, and heights it measures are off
by 4-6 cm depending on arm pose (the same cup reads 0.74 vs 0.805 m; the tray
0.787 vs 0.857 m; table fits 0.62-0.73 m). I want to MEASURE the mounting error
(roll, pitch, yaw and position) from the robot's own sensors, then fix the URDF
so heights agree from every pose. No rulers or hand measurements, no hardcoded
offsets — calibrate from data.

## Read first
- `parallel_prompts/HANDOFF_2026-10-29.md` (step A is exactly this), `LAUNCH.md`.
- Camera mount in `homeplus_urdf_description/urdf/` (xacro): find the joint(s)
  from the hand link to `hand_camera_link_1` / `d405_color_optical_frame` and
  their origin xyz/rpy. Arm joints: joint_shoulder, joint_elbow (closed-loop,
  encoders), joint_wrist (roll), joint_hand (pitch) — wrist/hand/gripper are
  OPEN-LOOP servos, so their reported angle is the commanded one, not measured.
- Clouds: `/vision/clouds/d405/full` (camera frame, acquisition stamps),
  `/cameras/d405/aligned_depth_to_color/image_raw`, TF from robot_state_publisher.

## Method (propose and refine)
1. Level-plane check (no target needed): from several arm poses, fit the table
   plane in the camera frame, transform to `world` with the TF at the cloud's
   stamp; the real table is level, so the fitted normal's tilt gives the error.
   Decompose into roll about the optical axis and pitch; report per pose.
2. Separate a fixed mount error from pose-dependent error: if the tilt is the
   same in every pose → camera mount (fix in URDF); if it varies with joint_hand
   or joint_wrist → those servos' zero/scale is off (report which, by how much).
3. Height/position: use the table height consistency across poses (and, if
   useful, an AprilTag/ArUco or checkerboard on the table — the repo has an ArUco
   detector in homeplus_vision) to solve the remaining translation/yaw. A small
   least-squares hand-eye style solve over N poses is fine.
4. Validate: after the correction, the table height and a fixed object's
   position must agree within ~5 mm across all poses used and 2 new ones.

## Constraints
- The robot stack may be running from the main checkout for other work. Before
  ANY arm motion, ask me; I will move the arm (via the console's manual
  commands) to the poses you request, or approve each motion. Never send motion
  commands yourself. Never restart the hp-* systemd units or the console
  without asking; read-only ROS subscriptions/TF lookups are fine.
- Make code/URDF changes on a branch in a git worktree; show me the exact URDF
  diff (old vs new rpy/xyz, in degrees and mm) before I apply it to the live
  robot. Don't push without asking. Don't run the repo's full pytest suites.
- Deliver: measured roll/pitch/yaw/xyz error with uncertainty, per-pose table,
  the root cause (mount vs servo calibration), the URDF fix, and a 2-minute
  re-check procedure I can repeat after bumping the camera.
