# Launch Instructions

## Build

```bash
cd ~/homeplus_robot_workspace
colcon build --packages-select homeplus_moveit_config
source install/setup.bash
```

## Launch (4 terminals)

### Terminal 1 — Robot TF + RealSense

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
bash scripts/run_robot_tf.sh &
sleep 3
bash scripts/run_realsense.sh
```

If you are testing DINO without Terminal 3, publish default arm joint states so
the hand camera is connected to `base_link` in TF:

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
PUBLISH_JOINT_STATES=true bash scripts/run_robot_tf.sh &
sleep 3
bash scripts/run_realsense.sh
```

### Terminal 2 — Grounding DINO (target detection)

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
WORLD_FRAME=base_link bash scripts/run_dino.sh 1
```

### Terminal 3 — MoveIt + Gesture Control + Arduino

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
ros2 launch homeplus_moveit_config gesture_control.launch.py arduino_port:=/dev/ttyACM0
```

To enable OctoMap collision avoidance:

```bash
ros2 launch homeplus_moveit_config gesture_control.launch.py arduino_port:=/dev/ttyACM0 use_octomap:=true
```

### Terminal 4 — Gesture Recognizer

```bash
cd ~/homeplus_robot_workspace
source install/setup.bash
bash scripts/run_gesture.sh /path/to/checkpoint.pt /path/to/label_map.json
```

## Verify Before Testing

Check Arduino is connected:

```bash
ros2 topic echo /arduino/ack
```

Check detection is working (place coffee cup in camera view):

```bash
ros2 topic echo /gdino_pose_world
```

## Test Without Gesture (manual trigger)

```bash
ros2 topic pub --once /gesture_control/command homeplus_interfaces/msg/GestureCommand \
  "{gesture_id: 1, gesture_label: 'test', confidence: 0.9, command_name: 'pick_up_coffee', command_type: 'sequence'}"
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `arduino_port` | `/dev/ttyACM0` | Serial port for Arduino |
| `run_arduino` | `true` | Launch Arduino serial bridge |
| `use_octomap` | `false` | Enable OctoMap collision avoidance |
| `enable_moveit_planning` | `true` | Enable MoveIt planning in executor |
| `launch_moveit` | `true` | Launch MoveIt stack |
