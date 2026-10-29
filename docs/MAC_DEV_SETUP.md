# HomePlus cup pickup: run cheatsheet (Mac + VM + AWS)

Yibing's 4-terminal flow, adapted for running everything from a MacBook:

| Piece | Runs on |
|---|---|
| ROS 2 (robot TF, detection node, MoveIt, Arduino bridge) | Ubuntu VM in UTM (`<user>@<vm-ip>`) |
| D405 hand camera | **Mac** (streamed into the VM; UTM USB passthrough can't do RealSense depth) |
| MolmoPoint + SAM2 models | AWS GPU `<INSTANCE_ID>` (g6.xlarge), reached through an SSH tunnel |

Branch: `experiment/closed-loop-control` (Yibing). VM workspace: `~/ros2_ws/src/homeplus_robot_workspace`.
Mac copy of the code: `the repo root`.

---

## 0. Before anything: hardware

- [ ] **Emergency stop released** (if it's pressed, the arm silently does nothing; the Arduino and relay still look fine)
- [ ] Arm power switch **on** (`|` = on, `O` = off)
- [ ] Battery clips connected (red→red / black→black, check `+`/`−` markings)
- [ ] Arduino USB → Mac, then attached to the VM: UTM toolbar **USB icon → Arduino Mega**
- [ ] D405 USB → Mac, **not** attached to the VM (leave it on macOS)
- [ ] Only **one** red cup in view (a second red object makes detection flip between them)

---

## 1. Mac: AWS model server

```bash
# Start the GPU instance (about $0.85/h while running)
aws ec2 start-instances --instance-ids <INSTANCE_ID>
aws ec2 wait instance-running --instance-ids <INSTANCE_ID>
IP=$(aws ec2 describe-instances --instance-ids <INSTANCE_ID> \
  --query 'Reservations[0].Instances[0].PublicIpAddress' --output text); echo $IP

# Start the server (it does NOT auto-start after a reboot). Model load takes 1–15 min.
ssh -i <your-aws-key.pem> ubuntu@$IP \
  'cd ~/homeplus_server && tmux new-session -d -s server "venv/bin/uvicorn server:app --host 127.0.0.1 --port 8765 2>&1 | tee ~/server.log"'

# Wait until healthy
ssh -i <your-aws-key.pem> ubuntu@$IP 'until curl -s 127.0.0.1:8765/health; do sleep 5; done'
```

Keep this **tunnel** open in its own Mac terminal (the VM calls `http://192.168.64.1:8765`):
```bash
ssh -i <your-aws-key.pem> -o ServerAliveInterval=15 -N \
  -L 192.168.64.1:8765:127.0.0.1:8765 ubuntu@$IP
```

## 2. Mac: D405 camera sender (own terminal, keep running)

```bash
cd tools/d405_mac_bridge && sudo .venv/bin/python mac_sender.py --fps 5
```

---

## 3. VM terminals

Open each with `ssh -i <your-vm-key> <user>@<vm-ip>`, or use a terminal on the VM desktop.
**In every VM terminal, run first:**
```bash
source ~/trackc_env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE=~/d405_fastdds.xml   # needed for the large depth images
```

### VM-A: D405 receiver
```bash
python3 ~/d405_vm_receiver.py
```

### T1: Robot TF
```bash
ros2 launch homeplus_urdf_description state_publisher.launch.py use_gui:=false publish_joint_states:=false
```
(`scripts/run_robot_tf.sh` expects `install/` inside the repo; there's a symlink for it, but the line above always works.)

### T2: Vision + MolmoPoint (D405 driver OFF, the Mac bridge supplies frames)
```bash
cd ~/ros2_ws/src/homeplus_robot_workspace
ros2 launch homeplus_vision vision_pipeline.launch.py side_mount:=left task_id:=1 \
  enable_d405:=false enable_d435i:=false enable_d455:=false \
  enable_target_detection:=true enable_gesture_detection:=false \
  enable_d405_octomap:=true enable_d435i_octomap:=false enable_d455_octomap:=false
```
Remote inference is turned on in the VM's `homeplus_vision/config/target_detection_params.yaml`:
`remote_inference_url: "http://192.168.64.1:8765"`.

### T3: MoveIt + Arduino bridge
⚠️ **This resets the Arduino, and the arm rises to upright (all joints 0°) on its own. Stand clear.**
```bash
ros2 launch homeplus_moveit_config gesture_control.launch.py \
  arduino_port:=/dev/ttyACM0 launch_obstacle_cloud:=false use_mapping:=true
```
To show RViz on the VM screen when starting T3 over SSH, prefix it with:
`DISPLAY=:0 XAUTHORITY=$(ls /run/user/1000/.mutter-Xwaylandauth.* | head -1)`

---

## 4. Checks before picking up

```bash
ros2 topic echo --once /arduino/ready            # data: true
ros2 topic echo --once /arduino/status           # data: true (arm still; detection only runs when true)
ros2 topic echo /molmopoint/diagnostics          # stage + reason for every detection cycle
ros2 topic echo /molmopoint_pose_world           # cup position in world; should stay steady
ros2 run rqt_image_view rqt_image_view           # pick /molmopoint/debug_image
```
Good detection looks like this: the debug image shows the red point and green mask **on the cup under the coffee machine**, and the world position doesn't jump between detections.

---

## 5. Run the pickup

```bash
cd ~/ros2_ws/src/homeplus_robot_workspace
bash scripts/task.sh 1 execute
```
⚠️ The first stages (open gripper, retract arm, **drive the base to 20 cm in front of the cup**) run **without** approval.

Approve or reject each planned path shown in RViz (60 s timeout):
```bash
ros2 topic pub --once /gesture_control/approve_planned_path std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /gesture_control/approve_planned_path std_msgs/msg/Bool "{data: false}"
```

Watch progress:
```bash
tail -f ~/runlogs/t3_moveit.log | grep -E "gesture_motion_executor|arduino_bridge"
```

### Other commands
```bash
# Home: all joints to 0 (arm upright, camera forward). Needs approval.
ros2 topic pub --once /gesture_control/command homeplus_interfaces/msg/GestureCommand \
  "{gesture_id: 0, gesture_label: home, confidence: 1.0, command_name: home, command_type: joint_state}"

# Stop motion now
ros2 topic pub --once /arduino/stop std_msgs/msg/Bool "{data: true}"

# Clear the obstacle map
ros2 service call /nvblox/clear std_srvs/srv/Trigger "{}"
```

---

## 6. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Arm doesn't move, Arduino fine, relay clicks | **E-stop pressed**, power switch off, or battery clip loose |
| `TRACKING_ERROR` / `SETTLE_TIMEOUT` right away | Motors have no power (see above) |
| Diagnostics: `motion_status_never_received` | T3 isn't running: detection waits for the Arduino bridge |
| Detection position jumps ~40 cm | Two red objects in view; remove one (or make the task caption more specific) |
| `move_to_pregrasp: no valid IK goal` + contacts at table height | Gripper hits the table in the obstacle map. A local fix tries +2/+4 cm (`safe_z_candidate_offsets_m`). This is the open planning issue |
| D405 "device busy", camera disappears from VM | Something started the RealSense driver in the VM. Keep `enable_d405:=false` |
| Detection node health check fails | AWS server not running or tunnel closed (sections 1–2) |

### Re-flash Yibing's firmware (from the VM)
```bash
export PATH=$HOME/.local/bin:$PATH
cd ~/ros2_ws/src/homeplus_robot_workspace
arduino-cli compile --fqbn arduino:avr:mega firmware/homeplus_controller
arduino-cli upload  --fqbn arduino:avr:mega -p /dev/ttyACM0 firmware/homeplus_controller
```
(Stop T3 first; it holds the serial port. The arm rises on boot.)

---

## 7. Shut down

```bash
# VM: stop all ROS processes started in tmux
tmux kill-session -t orch; tmux kill-session -t trackc

# Mac: stop AWS billing
aws ec2 stop-instances --instance-ids <INSTANCE_ID>
```
- The AWS disk was raised to 500 MB/s / 8000 IOPS for faster model loads (+≈$0.05/h). Turn it back down with
  `aws ec2 modify-volume --volume-id <VOLUME_ID> --throughput 125 --iops 3000`
  (AWS allows one change per 6 hours).
- Local code changes (detection node remote mode + diagnostics, tests, pregrasp z offsets) are **uncommitted**
  in `the repo root`.
