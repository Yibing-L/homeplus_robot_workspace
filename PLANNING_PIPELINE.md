# HomePlus planning pipeline

## Design direction

The HomePlus pickup pipeline follows a staged manipulation pattern:

1. acquire a fresh object pose;
2. open and verify the gripper;
3. plan to a collision-aware pre-grasp pose;
4. reacquire the object from the closer camera viewpoint;
5. plan the final approach;
6. close and verify the gripper;
7. plan a lift while holding the gripper state;
8. plan a retreat with the object.

The horizontal grasp stages account for the actual palm collision envelope:
with world RPY `[0, 0, 0]`, the palm extends roughly 4.7 cm below
`grasp_tcp`. Pre-grasp and grasp therefore target the upper part of a cup on a
support surface instead of placing the palm through the table.

Each motion is planned from the latest reached robot state. Only one command and
one MoveGroup request may be active. A timed-out request is cancelled, and late
results or approval messages cannot execute a later command. Hardware completion
timeouts abort the task by default.

ROS-facing joint states always use SI units. The Arduino boundary converts base
meters to centimeters and revolute-joint radians to degrees. In particular,
gripper values in `gesture_execution_map.yaml` remain Arduino degrees in the
range `[0, 90]`, but `/joint_states` contains gripper radians.

## Why this structure

- [MoveIt Task Constructor](https://doi.org/10.1109/ICRA.2019.8793898) models
  manipulation as interdependent stages, carries planning-scene state between
  stages, and ranks complete solutions rather than treating poses independently.
- [OK-Robot](https://arxiv.org/abs/2401.12202) uses a pre-grasp trajectory with
  progressively smaller approach motions. Its failure analysis recommends
  planning grasp trajectories rather than accepting grasp poses alone, updating
  obstacle maps, and adding explicit failure detection and retry.
- [Visually Grounded Task and Motion Planning](https://arxiv.org/abs/2202.10667)
  evaluates both feasibility and cost across candidate task-motion plans instead
  of selecting a target solely from its semantic detection.
- [Dynamic Planning for Sequential Whole-body Mobile Manipulation](https://arxiv.org/abs/2405.15377)
  separates online task replanning, whole-body motion generation, and reactive
  execution-time obstacle avoidance.

The current implementation adopts the parts that fit the installed ROS 2 Humble
stack and custom Arduino transport. `moveit_task_constructor_core` is not
currently installed, so this is a staged executor rather than a direct MTC task.

## Next recommended phase

1. Add several grasp candidates with confidence, approach vector, and object
   dimensions instead of one `PoseStamped`; reject candidates that are outside
   reach or collide, then rank complete pre-grasp/grasp/lift solutions.
2. Add the detected cup as a collision object and attach it to `grasp_tcp` after
   verified closure so lift and retreat account for the carried geometry.
3. Replace Arduino `DONE` as the only execution signal with measured joint and
   gripper feedback; compare actual versus planned state and stop on divergence.
4. Add local reactive stopping/replanning when the occupancy scene changes
   during execution. OMPL is a global planner and does not by itself make an
   already-streamed trajectory safe in a changing scene.
5. Once MoveIt Task Constructor is available, move the YAML stages into an MTC
   pick task and keep the Arduino bridge behind a standard trajectory controller
   action. This lets MTC plan and score full task solutions and use MoveIt's
   execution monitoring without a custom transport path.

## Safety-sensitive parameters

- `require_trajectory_approval`: require an RViz approval for a planned stage.
- `continue_on_arduino_timeout`: defaults to `false`; use `true` only for bench
  testing without reliable acknowledgements.
- `max_start_state_error`: rejects trajectories whose first waypoint is too far
  from the executor's current state.
- `max_base_goal_translation_m` and `max_base_goal_rotation_rad`: bound a pose
  stage's final mobile-base pose around its current state. This prevents an
  unconstrained whole-body IK goal from trading a distant base motion for an
  extreme folded-arm solution.
- `base_goal_tcp_x_min_m` and `base_goal_tcp_x_max_m`: for an ahead-facing
  whole-body stage, derive the allowed base-X interval from the requested TCP
  target. Unlike a fixed displacement limit, this allows the base to approach
  distant objects while keeping the arm inside a useful forward reach band.
- `clear_octomap_before_pose_plan`: clears stale occupancy immediately before a
  Cartesian request and waits `octomap_rebuild_wait_sec` for one current,
  target-filtered D405 cloud. A missing clear service fails the stage instead
  of silently planning against stale moving-camera voxels.
- `position_tolerance_m` and `orientation_tolerance_rad`: are goal tolerances,
  not path constraints. Tight values can make a valid grasp unnecessarily hard
  to plan.
- `maintain_orientation_during_path`: promotes the pose's orientation constraint
  to a MoveIt path constraint. The pickup uses it after the pre-grasp stage has
  first aligned `grasp_tcp` to world RPY `[0, 0, 0]`.
- `max_attempts`: retries a gripper command only when no `DONE` acknowledgement
  arrives. The close stage permits one retry after a missed command.
