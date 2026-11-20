# Prompt: VLA fine-tuning on the HomePlus embodiment with a scanned digital twin

Paste everything below the line into a new Claude Code session started in
`/mnt/c/users/easha/arl/homeplus_robot_workspace` (a fresh terminal, so Orchestra
is active). Research and planning first; no robot motion, no flashing, no
changes to the running stack (systemd --user hp-* units) unless I say so.

---

I want to find out whether, and how, we can (1) fine-tune a vision-language-action
model (VLA) on this robot's embodiment, and (2) build a digital twin of my real
environment from a scan and use it to generate large numbers of simulated
rollouts for training. Treat this as a feasibility study plus a concrete plan;
build only small spikes that de-risk the plan, in a git worktree.

Hard constraints (non-negotiable; reject options that violate them):
- Everything — sim rollouts, training/fine-tuning and inference on the robot —
  must run on THIS laptop's NVIDIA RTX 4090 Laptop GPU (16 GB VRAM) under WSL2.
  No rented/cloud GPUs. Choose models, precision (bf16/LoRA/QLoRA/4-bit),
  batch sizes, image resolution and action-chunk sizes so they fit 16 GB, and
  show the memory budget for each phase. If Isaac Sim/Isaac Lab cannot run
  under WSL2 on this GPU, say so and pick what can (e.g. MuJoCo/MJX, ManiSkill,
  Genesis).
- It must work for THIS task (the coffee sequence on the real robot: pick up
  the red cup from the table, place it on the coffee machine's drip tray, press
  the bottom and top silver buttons), not a generic benchmark.
- It must use THIS embodiment as defined by our URDF/xacro and SRDF (mobile
  base as planar joints + 4-joint arm + gripper, hand camera D405), not a
  stand-in robot. Any change to the URDF for simulation goes on a branch and is
  explained.

## The robot (read the repo; don't guess)
- Repo: ROS 2 Humble, MoveIt 2. URDF/xacro in `homeplus_urdf_description/`
  (mobile base modelled as planar joints base_x/base_y/base_theta; arm joints
  joint_shoulder, joint_elbow, joint_wrist (roll), joint_hand; gripper
  joint_grip_L; links grasp_tcp, press_tcp, hand_camera_link_1). SRDF and limits in
  `homeplus_moveit_config/config/`.
- Firmware `firmware/homeplus_controller/homeplus_controller.ino`: shoulder and
  elbow are closed-loop (AS5600 encoders, PID, 3 deg tracking limit); wrist, hand
  and gripper are open-loop hobby servos; base has x/y/theta PID with a
  stop-go output stage. Commands are streamed joint trajectories (STATE4
  protocol, `homeplus_moveit_config/scripts/arduino_reader.py`).
- Known motion issues: `origin/jerk-diagnosis-prototypes` (firmware hold band,
  speed limiter lag, base timing). Sim-to-real must model these or fix them.
- Sensors: Intel RealSense D405 on the hand (short range, ~7-50 cm; used at
  1-1.5 m it is noisy, and its extrinsic error varies with arm pose by up to
  ~6.5 cm — see `parallel_prompts/HANDOFF_2026-10-29.md`, step A), D455 on the
  side, D435i at the front.
- Hardware: Windows laptop with WSL2 Ubuntu 22.04, RTX 4090 Laptop GPU (16 GB),
  CUDA toolkit not yet installed in WSL.
- Current tasks (classical stack, MolmoPoint + SAM2 + MoveIt): pick up a cup,
  place it on a coffee machine's drip tray, press two buttons. Logs:
  `~/trajectory_logs/*.csv`, journalctl for hp-moveit / hp-bridge.

## Questions to answer, with sources (papers, repos, docs; say what's unverified)
1. VLA choice for a custom, partly open-loop, mobile-base embodiment that must
   train AND run on a 16 GB RTX 4090 Laptop GPU: OpenVLA / OpenVLA-OFT (LoRA/4-bit?),
   π0 / π0-FAST / π0.5 (openpi, LoRA?), SmolVLA, Octo, ACT / Diffusion Policy
   (LeRobot). Which actually fit fine-tuning and real-time inference in 16 GB
   (cite measured numbers where they exist); action-space handling (joint vs
   end-effector, chunking, the planar base); minimum demo counts reported for
   new embodiments.
2. "Does it work with any URDF?" What a URDF gives a simulator (kinematics,
   collision, inertia) and what it does not (actuator dynamics, servo
   dead-bands, backlash, PID behaviour, latency, sensor noise). Check our URDF
   imports into Isaac Sim / Isaac Lab and MuJoCo (via urdf2mjcf or MuJoCo's
   importer); list what needs fixing (meshes, inertias, joint limits,
   mimic joints for the gripper, the planar base).
3. Environment scanning: phone LiDAR/photogrammetry (Polycam, Scaniverse),
   Gaussian splats (and SplatSim, RialTo-style real-to-sim-to-real), mesh
   import into Isaac Sim / MuJoCo; collision meshes vs visual realism;
   articulated parts (the coffee machine's buttons, drip tray).
4. Rollouts at scale: RL in Isaac Lab (GPU-parallel) vs scripted/planner
   demonstrations in sim (using our MoveIt pipeline as the expert) for
   imitation; domain randomization needed for transfer (lighting, textures,
   camera extrinsics ±cm, depth noise like the D405 at range, servo lag,
   base slip); evaluation protocol on the real robot.
5. Real data: what teleop rig fits this robot (gamepad, SpaceMouse, console
   jog), recording format (LeRobot dataset), and how many real demos to mix
   with sim data.
6. Honest risk assessment: which parts are research, which are engineering,
   rough effort per phase, and the smallest end-to-end experiment that proves
   or disproves the approach (e.g. one task — "press the bottom button" — in
   sim, transferred to the robot).

## Spikes worth doing (only in a worktree, nothing on the live stack)
- Convert our URDF (`xacro` → URDF) and load it in MuJoCo (pip, CPU OK); report
  what breaks and fix it on a branch. Check joint limits/inertias are sane.
- Replay a few `~/trajectory_logs` CSVs in that sim and compare planned vs
  simulated motion (tells us how much actuator modelling matters).
- Sketch the data pipeline: ROS bag/topic → LeRobot dataset schema for this
  robot (cameras, joint states, commanded actions, language instruction).

## Deliver
A short written plan (publish it as an artifact page I can share with Yibing):
recommended model and sim stack, phases with effort, the first experiment, what
I need to do physically (scan, teleop rig, calibration), the 16 GB VRAM budget
per phase and expected training times on this laptop, and disk needs. Plus the branch name for any spikes. Do not install system packages
with sudo or touch the robot without asking me first.
