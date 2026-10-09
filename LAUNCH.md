# Launch Instructions

## Build

```bash
cd ~/homeplus_robot_workspace
colcon build --packages-select homeplus_vision homeplus_moveit_config
source install/setup.bash
bash scripts/setup_molmopoint.sh --download-model
```

## Firmware compatibility

This workspace now requires STATE4 streaming firmware from
`ARDUINO_FIRMWARE_UPDATED.txt`. Build and flash the matching sketch separately;
the source changes do not flash the board. The matching Arduino IDE sketch is
`firmware/homeplus_controller/homeplus_controller.ino`. Restart Terminal 3 after
uploading. The bridge refuses streamed motion with older STATE2 firmware.
See `PLANNING_PIPELINE.md` for the protocol, regression tests and remaining
calibration requirements. Hardware acknowledgements must remain enabled.

## Launch (3 terminals)

### Terminal 1 — Robot TF

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
bash scripts/run_robot_tf.sh
```

If you are testing MolmoPoint without Terminal 3, publish default arm joint states so
the hand camera is connected to `base_link` in TF:

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
PUBLISH_JOINT_STATES=true bash scripts/run_robot_tf.sh
```

### Terminal 2 — Three-camera vision

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
bash scripts/run_vision.sh \
  side_mount:=left \
  enable_gesture_detection:=true \
  gesture_checkpoint:=/path/to/checkpoint.pt \
  gesture_label_map:=/path/to/label_map.json
```

This assigns D405 to target detection and the target-filtered OctoMap. D435i
and D455 remain available as cameras but do not feed OctoMap by default because
they do not yet receive a matching target mask; enabling either would insert
the cup back into the collision map. Use `side_mount:=right` when the D455 is
installed on the right side. Omit the gesture arguments when testing without a
gesture checkpoint.

### Terminal 3 — MoveIt + Gesture Control + Arduino

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
ros2 launch homeplus_moveit_config gesture_control.launch.py \
  launch_obstacle_cloud:=false
```

Target-filtered OctoMap collision avoidance is enabled by default. To disable it
for troubleshooting:

```bash
ros2 launch homeplus_moveit_config gesture_control.launch.py use_octomap:=false
```

After both terminals start, verify that exactly one target-aware cloud feeds
MoveIt:

```bash
ros2 topic info /obstacle_cloud -v
ros2 service call /clear_octomap std_srvs/srv/Empty "{}"
```

The first command should show one publisher (`/d405_obstacle_cloud`) and one
subscriber (`/move_group`). The D405 terminal prints `CLOUD_STATS` every five
seconds; `mask=active` is required while planning a detected cup. D405 cloud
publication pauses during Arduino motion, then `/clear_octomap` is called and
the map is rebuilt from the settled hand-camera pose after `DONE`.

## Verify Before Testing

Check Arduino is connected:

```bash
ros2 topic echo /hardware/joint_states
ros2 topic echo /arduino/ready
```

Check detection is working (place coffee cup in camera view):

```bash
ros2 topic echo /molmopoint_pose_world
```

## Test Without Gesture (manual trigger)

```bash
ros2 topic pub --once /gesture_control/command homeplus_interfaces/msg/GestureCommand \
  "{gesture_id: 1, gesture_label: 'test', confidence: 0.9, command_name: 'pick_up_coffee', command_type: 'sequence'}"
```

The configured task sequences execute their planned stages automatically. For
manual review, set `require_trajectory_approval: true` on the relevant steps in
`gesture_execution_map.yaml`; publish a Bool on
`/gesture_control/approve_planned_path` to approve/reject that displayed stage.
The executor validates the current state and path again before sending motion.

## Press a Square Button

The executor selects detector task 2 automatically when pressing. To preview its detections before triggering:

```bash
bash scripts/run_vision.sh task_id:=2
```

Then trigger the pressing sequence directly:

```bash
ros2 topic pub --once /gesture_control/command homeplus_interfaces/msg/GestureCommand \
  "{gesture_id: 2, gesture_label: 'test', confidence: 0.9, command_name: 'press_square_button', command_type: 'sequence'}"
```

The sequence sets the gripper to 70 degrees, retracts the arm before any base
repositioning, approaches the detection, refines the button pose from the
settled close view, places the dedicated 70-degree single-fingertip `press_tcp`
at that refreshed target, pauses for one second, and retreats. Its motion stages
do not require separate RViz approval. Contact uses a fixed-base Cartesian path;
no button activation/force sensor is available to confirm activation.

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `arduino_port` | `auto` | Serial port for Arduino, or auto-detect |
| `arduino_baud` | `115200` | Serial baud; must match firmware `Serial.begin(...)` |
| `run_arduino` | `true` | Launch Arduino serial bridge |
| `wait_for_hardware_ack` | `true` | Wait for Arduino `DONE`; must remain true for this hardware executor |
| `continue_on_arduino_timeout` | `false` | Must remain false; timeouts retain measured state and abort |
| `use_octomap` | `true` | Enable target-filtered OctoMap collision avoidance |
| `enable_moveit_planning` | `true` | Enable MoveIt planning in executor |
| `launch_moveit` | `true` | Launch MoveIt stack |
