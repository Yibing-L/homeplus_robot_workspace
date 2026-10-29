# HomePlus Vision

ROS 2 perception package for the HomePlus robot's three Intel RealSense
cameras. Camera identity is part of the interface: perception nodes never
subscribe to an anonymous `/camera` topic.

## Camera Roles

| Camera | Mount | Responsibilities |
|---|---|---|
| D405 | End effector | Target detection and obstacle mapping |
| D435i | Front of base | Obstacle mapping |
| D455 | Left or right side of base | Gesture detection and obstacle mapping |

The machine-readable inventory is in `config/camera_roles.yaml`. The D455 side
is selected at launch with `side_mount:=left` or `side_mount:=right`.

## Runtime Architecture

```text
D405  /cameras/d405  -> MolmoPoint-8B ------> /molmopoint_pose_world
                     -> SAM2 point mask ----> /object_mask
                     -> obstacle cloud -----+
                                              |
D435i /cameras/d435i -> obstacle cloud ------+-> /obstacle_cloud -> MoveIt
                                              |
D455  /cameras/d455  -> obstacle cloud ------+
                     -> gesture recognizer -> /gesture_recognition/state
```

All obstacle-cloud nodes transform points into `base_link` before publishing.
The D405 node removes the current target mask from its obstacle cloud so MoveIt
can approach the detected object. D435i and D455 contribute unmasked scene
geometry.

## Launch

Build and source the workspace:

```bash
cd ~/ros2_ws/homeplus_robot_workspace
colcon build --packages-select homeplus_vision homeplus_moveit_config
source install/setup.bash
```

Install the dedicated runtime and download the official checkpoint once:

```bash
bash scripts/setup_molmopoint.sh --download-model
```

This deployment pins official `allenai/MolmoPoint-8B` revision
`188130f961c8e0888a34e11121a1423c461a01ba` with NF4 4-bit weights on the
12 GB RTX 4080 Laptop GPU. The full BF16/F32 checkpoint does not fit alongside
ROS perception and SAM2. The local environment inherits the workspace's ROS 2
and CUDA-enabled PyTorch packages while pinning the Transformers version
required by MolmoPoint.

Review the upstream model card before non-research deployment. Although the
weights are Apache-2.0, Ai2 notes that some training datasets are restricted to
academic or non-commercial research use.

Start robot TF in one terminal:

```bash
PUBLISH_JOINT_STATES=false bash scripts/run_robot_tf.sh
```

Start all cameras, target detection, and obstacle-cloud generation:

```bash
bash scripts/run_vision.sh side_mount:=left
```

Enable gesture recognition by supplying its checkpoint:

```bash
bash scripts/run_vision.sh \
  side_mount:=right \
  enable_gesture_detection:=true \
  gesture_checkpoint:=/absolute/path/model.pt \
  gesture_label_map:=/absolute/path/labels.json
```

The gesture pipeline is disabled by default because a trained checkpoint is a
required runtime input. The D455 camera and its obstacle cloud still start.

## Device Selection

The launch file selects each device by model. Serial numbers are optional when
exactly one D405, D435i, and D455 are attached. Pin devices for repeatable
experiments when needed:

```bash
export D405_SERIAL="_123456789"
export D435I_SERIAL="_234567890"
export D455_SERIAL="_345678901"
export D455_SIDE="left"
bash scripts/run_vision.sh
```

The leading underscore keeps a numeric RealSense serial represented as a
string by ROS parameters.

## Launch Files

- `multi_camera.launch.py`: three RealSense drivers and robot mount TFs.
- `perception.launch.py`: target, gesture, and obstacle-cloud nodes for cameras
  that are already running.
- `vision_pipeline.launch.py`: normal full composition.
- `molmopoint_pipeline.launch.py`: D405-only target-detection entry.
- `molmopoint_test_interface.launch.py`: live MolmoPoint prompt tester.
- `dino_pipeline.launch.py` and `dino_test_interface.launch.py`: legacy aliases.
- `gesture_pipeline.launch.py`: D455-only gesture compatibility entry.
- `camera_only.launch.py`: D405-only camera compatibility entry.

## Interactive MolmoPoint Test

Start the D405, the production detector node, and the live test window:

```bash
source .venv-molmopoint/bin/activate
ros2 launch homeplus_vision molmopoint_test_interface.launch.py
```

Type a natural-language target and click **Apply live** (or press Enter). The
model stays loaded while the prompt changes. The image and point table show
`/molmopoint/debug_image` and `/molmopoint/detections` output. MolmoPoint emits
constrained point tokens rather than calibrated detector confidence values.

If the D405 driver is already running, avoid launching a second camera:

```bash
ros2 launch homeplus_vision molmopoint_test_interface.launch.py launch_camera:=false
```

To attach only the window to a detector that is already running:

```bash
ros2 run homeplus_vision molmopoint_test_interface.py
```

Pass an initial target or enable the normal SAM2/depth pose stage when needed:

```bash
ros2 launch homeplus_vision molmopoint_test_interface.launch.py \
  caption:='the red mug on the left' publish_poses:=true
```

## Topic Contract

Camera inputs:

```text
/cameras/d405/{color,aligned_depth_to_color}
/cameras/d435i/{color,aligned_depth_to_color}
/cameras/d455/{color,aligned_depth_to_color}
```

Perception outputs retained for compatibility with the motion stack:

```text
/molmopoint_pose_world
/object_mask
/gesture_recognition/state
/obstacle_cloud
```

Per-camera debug clouds are available under `/vision/clouds/<camera>/`.

## MoveIt Integration

`homeplus_moveit_config` subscribes to `/obstacle_cloud`. When the full vision
pipeline is running, launch gesture control without its legacy D405 cloud node:

```bash
ros2 launch homeplus_moveit_config gesture_control.launch.py \
  use_mapping:=true \
  launch_obstacle_cloud:=false
```

## Verification

Confirm each physical camera is publishing on its assigned namespace:

```bash
ros2 topic hz /cameras/d405/color/image_raw
ros2 topic hz /cameras/d435i/aligned_depth_to_color/image_raw
ros2 topic hz /cameras/d455/color/image_raw
ros2 topic hz /obstacle_cloud
```

Check mount transforms:

```bash
ros2 run tf2_ros tf2_echo hand_camera_link d405_link
ros2 run tf2_ros tf2_echo front_camera_link d435i_link
ros2 run tf2_ros tf2_echo base_link d455_link
```

RealSense publishes factory camera intrinsics on each `camera_info` topic. Mount
extrinsics belong in the robot URDF or the configurable D455 mount launch
arguments, not in a generic sample calibration file.
