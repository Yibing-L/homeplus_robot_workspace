# Speed investigation: why is HomePlus slow, and what makes it faster

You are working on the HomePlus home-assistant robot (Cornell Architectural Robotics Lab):
a mobile base + 4-joint arm + gripper with RealSense cameras (D405 on the hand, D455 on the
left side) that picks up a red cup, places it on a coffee machine drip tray, presses the
machine's buttons and hands the cup to a person. ROS 2 Humble, MoveIt 2, WSL2 on a laptop
with an RTX 4090 Laptop GPU (16 GB). Workspace: `/mnt/c/users/easha/arl/homeplus_robot_workspace`.
Another session is working on accuracy (button press, place, pickup) at the same time:
**do not change behaviour of the task sequences; your job is speed.**

## Rules (non-negotiable)
- **No robot motion.** Never send motion commands, never start tasks, never publish to
  `/arduino/*` or `/gesture_control/*`. Read-only ROS subscriptions are fine.
- **Do not restart services** (`hp-vision`, `hp-moveit`, `hp-executor`, `hp-bridge`,
  `hp-tf`, the console) without the operator's explicit OK. Work offline where you can.
- The GitHub repo is **public**: never commit secrets, IPs, instance IDs or passwords.
- `homeplus_vision/scripts/grounding_dino_node.py` was just changed by Yibing (commit
  09b46fe: RGB-D pairing, acquisition-time TF). Don't rewrite his logic; propose changes and
  keep them minimal. Work on a new branch (`speed-investigation`), commit there, don't push
  to `main` without the operator's OK.
- Don't run the whole test suite; targeted tests only.

## What we already measured (2026-11-20)
Across today's sequences (424 s total), time split roughly:
| robot moving | detection | OctoMap clear + rebuild | other (gripper waits, post_delay_sec, settle sleeps) | planning |
|---|---|---|---|---|
| 30% | 27% | 19% | 17% | 3% |

Detector (`/molmopoint/diagnostics`): `inference_sec` ≈ 3.0–3.3 s per call (was ~2.2 s a week
ago), `segment_sec` (SAM2) ≈ 0.05 s, GPU utilisation low. Model: `allenai/MolmoPoint-8B`,
4-bit bitsandbytes, `max_new_tokens` 64, Molmo multi-crop preprocessing. Each executor
detection step wants 3 agreeing samples (≈10 s). The detector also pauses while
`/arduino/status` says "moving" — today it was paused 62 of 71 cycles while the robot sat still
(firmware stuck reporting moving with motors unpowered).

## Questions to answer, with numbers
1. **Where do the ~3 s per detection go?** Split `run_model_inference` into: processor /
   multi-crop preprocessing (CPU), host→GPU copy, `generate` (and tokens generated), point
   extraction. Do it offline: load the same model in the `.venv-molmopoint` venv on saved
   frames (see below) — the GPU has room only if vision is stopped, so ask the operator for a
   window, or measure with a minimal timing patch they can load at the next vision restart.
2. **Cheap wins on the current model:** fewer crops / smaller input resolution, lower
   `max_new_tokens` (a point needs ~10–20 tokens), SDPA/flash-attention, bf16 compute dtype,
   8-bit vs 4-bit (4-bit dequant is often slower), `torch.compile`. Measure accuracy too.
3. **Faster models** for open-vocabulary pointing/boxes, benchmarked on OUR frames for speed
   AND accuracy: e.g. smaller Molmo variants (Molmo-7B-O / MolmoE-1B), Grounding DINO
   (tiny/base, there is a `Grounded-SAM-2/` checkout in the repo), OWLv2, YOLO-World,
   Florence-2, Qwen2.5-VL-3B, Moondream2. Prompts that matter: "red cup", "the bottom silver
   button on the coffee machine", "the top silver button on the coffee machine", the drip tray,
   "the person", "the person's open hand, palm up" (`homeplus_vision/config/task_registry.yaml`).
   The hard case is bottom vs top button (two similar silver buttons ~6 cm apart).
4. **Detect-then-track:** detect once with the big model, then track (SAM2 video predictor /
   template tracking) for the refine steps.
5. **Non-detection time:**
   - OctoMap clears — `clear_octomap_before_plan` + `octomap_rebuild_wait_sec` on many steps in
     `homeplus_moveit_config/config/gesture_execution_map.yaml`; could clear only on a
     start-state map collision?
   - Fixed sleeps — `settle_sec`, `post_delay_sec`, gripper waits in
     `homeplus_moveit_config/scripts/gesture_motion_executor.py`.
   - The detector motion gate stuck on "moving".
   - Base-then-arm sequential phases that could be combined.
   Quantify each from the logs (`journalctl --user -u hp-executor` / `-u hp-moveit`, the
   executor logs every step with timestamps).

## Data you can use
- `detection_logs/molmopoint_targets_*.jsonl`: every detection (pixel, model point, 3-D
  camera/world point, label, stamp). No images in there.
- To get frames: with the operator's OK, record a short `ros2 bag` of
  `/cameras/d405/color/image_raw`, `/cameras/d405/aligned_depth_to_color/image_raw`,
  camera_info, and `/tf` `/tf_static` while vision runs (read-only), with the cup, buttons,
  tray and a person in view. Or save PNGs from those topics.
- `tests/test_detector_replay.py` replays the detector on synthetic inputs.

## Deliverable
A short report (also a markdown file under `docs/` on your branch):
1. a table per option: ms/detection, accuracy on our frames (hit/miss per prompt, pixel error
   where checkable), VRAM, effort;
2. the time budget of a full pickup → place → button run and the projected saving per fix;
3. a ranked recommendation (do-now / try-next / skip);
4. ready-to-load patches for the do-now items on `speed-investigation`, each small and
   reversible, with exact load steps (which service to restart).
