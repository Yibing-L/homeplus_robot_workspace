# Prompt: go straight from the cup pickup to the drip tray (no retreat_with_object)

Paste everything below the line into a new Claude Code session started in
`/mnt/c/users/easha/arl/homeplus_robot_workspace` (fresh terminal, Orchestra on).

---

On the HomePlus robot, `pick_up_coffee` ends with `lift` then
`retreat_with_object` (backs the held cup away), and then
`place_cup_on_drip_tray` starts with `move_to_tray_standoff` (currently
`keep_arm: true`: keeps the arm as pickup left it and drives only the base),
then `move_over_tray`, `slide_to_wall`, a guarded `lower_cup_onto_tray`,
release, retreat, retract. I think `retreat_with_object` is unnecessary: after
the lift the robot should go directly toward the coffee machine's drip tray.
Investigate whether that's right and, if so, how to do it safely. Pickup itself
works well — do not change any pickup step before `lift` (open_gripper,
move_to_pregrasp, refine_target, align_refined_grasp, move_to_grasp,
close_gripper_at_target, lift must behave exactly as now).

## Read first
- `homeplus_moveit_config/config/gesture_execution_map.yaml` (pick_up_coffee,
  place_cup_on_drip_tray), `homeplus_moveit_config/scripts/gesture_motion_executor.py`
  (sequence loop, held-cup attach/ACM rules, `_standoff_keep_arm`,
  `_prepare_mobile_standoff`, measure_ahead/slide_to_wall, guarded
  `contact_ends_step`), `parallel_prompts/HANDOFF_2026-10-29.md`, git log of the
  last two weeks for why each step exists (`git log -p -S retreat_with_object`).
- Logs of real runs: `journalctl --user -u hp-moveit -u hp-executor` and
  `~/trajectory_logs/` (traj_retreat_with_object_*, traj_*standoff*,
  traj_*place*). Compare where the gripper/base are after `lift`, after
  `retreat_with_object`, and at the place standoff.

## Questions
1. What does `retreat_with_object` actually do (direction, distance, speed,
   collision checks, map clears) and what problem was it added for (e.g. the
   lifted cup/arm clearing the table edge before the base turns or drives)?
2. If it's removed (or folded into the place standoff), is the base drive from
   the post-lift pose to the tray standoff still collision-free with the cup and
   arm extended over the table? Use the logged poses and the collision model
   (table box, OctoMap) to check; replay planning offline where possible.
3. Best design: (a) drop the step; (b) keep a minimal lift only and let
   `move_to_tray_standoff` (base-only, keep_arm) carry the cup; (c) a combined
   "pickup → place" sequence so the robot never stops between them. Consider
   the console (it starts tasks separately: task 1 pickup, task 7 place) and
   holding-state handoff.
4. Timing: how many seconds would this save per run?

## Constraints
- Analysis + a proposed change on a branch in a git worktree. Do not touch the
  main checkout (a real robot runs from it), do not restart hp-* systemd units
  or the console, do not send motion or publish to robot topics. If you run ROS
  for an offline check, export ROS_DOMAIN_ID=89 first.
- Do not run the repo's full pytest suites; validate the execution map with the
  executor's own validator (`GestureMotionExecutor._command_config_errors`).
- Deliver: findings with evidence (log lines, poses), the recommended option,
  the exact YAML/code diff, and a short test plan I can run on the robot with
  STOP in hand. Don't push without asking.
