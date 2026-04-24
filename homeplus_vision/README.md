# homeplus_vision

A ROS2 package for ArUco marker detection and pose estimation using Intel RealSense cameras.

## Features

- Real-time ArUco marker detection
- 6-DOF pose estimation (XYZ position + RPY orientation)
- TF2 transforms for coordinate frame management
- Integration with RealSense D400 series cameras
- Configurable marker sizes and dictionary types
- Debug visualization
- Real-time gesture recognition from aligned RGB-D streams

## Dependencies

Make sure you have the following packages installed:

```bash
# ROS2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-realsense2-camera

# Python dependencies
pip install opencv-python numpy scipy
pip install mediapipe torch
```

For the ROS Python environment used in this workspace, the combinations below were
validated while bringing up the gesture and Grounding DINO nodes in WSL:

```bash
pip --python /usr/bin/python3 install --user \
  "numpy==1.26.4" \
  "opencv-contrib-python==4.10.0.84" \
  "mediapipe==0.10.14" \
  "torch==2.11.0"
```

Important notes:
- `cv_bridge` in ROS Humble breaks if `numpy 2.x` is installed in `/usr/bin/python3`.
- The `grounding_dino` stack in this repo was only compatible after downgrading
  `transformers` to `4.33.2`.
- Use `pip --python /usr/bin/python3 ...` so packages land in the same Python
  interpreter that `ros2` uses.

## Package Structure

```
homeplus_vision/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── aruco_params.yaml          # ArUco detection parameters
│   └── camera_calibration.yaml    # Camera calibration template
├── launch/
│   ├── vision_pipeline.launch.py  # Complete vision pipeline
│   ├── camera_only.launch.py      # RealSense camera only
│   └── aruco_only.launch.py       # ArUco detection only
└── scripts/
    ├── aruco_detector.py           # Main ArUco detection node
    └── aruco_pose_publisher.py     # Pose transformation node
    └── gesture_recognizer.py       # Real-time gesture classifier node
```

## Usage

### 1. Complete Vision Pipeline (Recommended)

Launch the complete pipeline with RealSense camera and ArUco detection:

```bash
ros2 launch homeplus_vision vision_pipeline.launch.py
```

Optional parameters:
```bash
ros2 launch homeplus_vision vision_pipeline.launch.py \
    marker_size:=0.05 \
    aruco_dict_type:=0 \
    target_marker_id:=0
```

### 2. Camera Only

Launch just the RealSense camera:

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_accel:=false \
    enable_gyro:=false \
    enable_motion:=false \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 \
    enable_sync:=true \
    align_depth.enable:=true
```

The older `camera_only.launch.py` in this package still uses legacy RealSense
argument names and may warn or fail on newer `realsense2_camera` versions.

### 3. ArUco Detection Only

If you have another camera source running:

```bash
ros2 launch homeplus_vision aruco_only.launch.py
```

### 4. Gesture Recognition Pipeline

Launch RealSense plus the gesture recognizer:

```bash
ros2 launch homeplus_vision gesture_pipeline.launch.py \
    checkpoint_path:=/absolute/path/to/model_7.pt
```

Optional parameters:

```bash
ros2 launch homeplus_vision gesture_pipeline.launch.py \
    checkpoint_path:=/absolute/path/to/model_7.pt \
    label_map_path:=/absolute/path/to/labels.json
```

The gesture node consumes aligned depth from `/camera/camera/aligned_depth_to_color/image_raw`
and camera intrinsics from `/camera/camera/color/camera_info`, mirroring the `landmark_with_xyz.py`
feature layout and the `online_recognizer_xyz.py` runtime logic.

### 5. Grounding DINO

This node was brought up against a local checkout of `Grounded-SAM-2` placed at:

```text
/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2
```

The current `grounding_dino_node.py` supports this local layout through the
`grounded_sam_root` parameter.

#### Local setup used successfully

1. Add the vendor repo to `PYTHONPATH`:

```bash
export PYTHONPATH="/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2:/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2/grounding_dino:${PYTHONPATH}"
```

2. Install the compatible Hugging Face version:

```bash
pip --python /usr/bin/python3 install --user --force-reinstall "transformers==4.33.2"
```

3. Download the Grounding DINO checkpoint:

```bash
cd /mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2/gdino_checkpoints
bash download_ckpts.sh
```

The required file is:

```text
Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth
```

`publish_poses:=false` is the easiest mode to start with because it does not
require SAM2 segmentation checkpoints.

#### Run with a RealSense camera

Terminal 1:

```bash
ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    enable_accel:=false \
    enable_gyro:=false \
    enable_motion:=false \
    rgb_camera.color_profile:=640x480x30 \
    depth_module.depth_profile:=640x480x30 \
    enable_sync:=true \
    align_depth.enable:=true
```

Terminal 2:

```bash
cd /mnt/c/users/easha/arl/homeplus_robot_workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2:/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2/grounding_dino:${PYTHONPATH}"
ros2 run homeplus_vision grounding_dino_node.py --ros-args \
    -p device:=cpu \
    -p publish_poses:=false \
    -p grounded_sam_root:=/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2 \
    -p image_topic:=/camera/camera/color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info
```

To enable 3D pose estimation from aligned depth after the 2D path is working:

```bash
ros2 run homeplus_vision grounding_dino_node.py --ros-args \
    -p device:=cpu \
    -p publish_poses:=true \
    -p grounded_sam_root:=/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2 \
    -p image_topic:=/camera/camera/color/image_raw \
    -p depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    -p camera_info_topic:=/camera/camera/color/camera_info
```

#### Run on a standalone image

Terminal 1:

```bash
cd /mnt/c/users/easha/arl/homeplus_robot_workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2:/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2/grounding_dino:${PYTHONPATH}"
ros2 run homeplus_vision grounding_dino_node.py --ros-args \
    -p device:=cpu \
    -p publish_poses:=false \
    -p grounded_sam_root:=/mnt/c/users/easha/arl/homeplus_robot_workspace/Grounded-SAM-2
```

Terminal 2:

```bash
cd /mnt/c/users/easha/arl/homeplus_robot_workspace
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run homeplus_vision test_grounding_dino_static.py \
    --image /mnt/c/users/easha/arl/homeplus_robot_workspace/pic.JPG \
    --rate 1 \
    --count 0
```

#### Inspect Grounding DINO output

```bash
ros2 topic echo /gdino/detections
ros2 topic echo /gdino/mask_center
ros2 topic hz /gdino_debug_image
```

Visualize the overlay:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select:

```text
/gdino_debug_image
```

(Run octomap:
```bash
ros2 launch homeplus_moveit_config cloud_to_octomap.launch.py \
    octomap_input_topic:=/full_cloud \
    depth_topic:=/camera/camera/depth/image_rect_raw \
    camera_info_topic:=/camera/camera/depth/camera_info
```
)
## Configuration

### Parameter Hierarchy

The system uses a clean parameter hierarchy to avoid confusion:

1. **Primary source**: `config/aruco_params.yaml` - Edit this file for your setup
2. **Runtime overrides**: Launch arguments - Use for testing different values
3. **No hardcoded defaults** - All values must be explicitly set

### ArUco Parameters

**IMPORTANT**: Edit `config/aruco_params.yaml` for your physical setup:

```yaml
aruco_detector:
  ros__parameters:
    aruco_dict_type: 0        # Match your printed markers
    marker_size: 0.05         # MEASURE your actual marker size!
    camera_frame: "camera_color_optical_frame"
    base_frame: "base_link"
```

**Runtime override example:**
```bash
ros2 launch homeplus_vision vision_pipeline.launch.py \
    marker_size:=0.08 \
    aruco_dict_type:=4
```

### Camera Calibration

The package uses camera_info from the RealSense driver by default. For custom calibration:

1. Run camera calibration:
```bash
ros2 run camera_calibration cameracalibrator.py \
    --size 8x6 --square 0.025 \
    image:=/camera/color/image_raw \
    camera:=/camera/color
```

2. Update `config/camera_calibration.yaml` with your results.

### Transform Setup

The package assumes the following TF tree:
```
base_link → camera_link → camera_color_optical_frame → aruco_X
```

Adjust the static transforms in `vision_pipeline.launch.py` based on your camera mounting.

## Topics

### Published Topics

- `/aruco_pose` (geometry_msgs/PoseStamped): Raw ArUco pose in camera frame
- `/aruco_pose_base_link` (geometry_msgs/PoseStamped): Transformed pose in robot frame
- `/aruco_debug_image` (sensor_msgs/Image): Debug visualization
- `/gesture_recognition/label` (std_msgs/String): Stable gesture label or `idle`
- `/gesture_recognition/class_id` (std_msgs/Int32): Predicted class index, `-1` when idle
- `/gesture_recognition/confidence` (std_msgs/Float32): EMA confidence of the current label
- `/gesture_recognition/active` (std_msgs/Bool): Whether the hand is currently active
- `/gesture_recognition/debug_image` (sensor_msgs/Image): Overlay with landmarks and current state
- `/gdino/detections` (std_msgs/String): JSON list of 2D detections
- `/gdino_debug_image` (sensor_msgs/Image): Grounding DINO debug overlay
- `/gdino/mask_center` (std_msgs/String): JSON target center payload
- `/gdino_pose_camera` (geometry_msgs/PoseStamped): 3D target pose in the camera frame when `publish_poses:=true`
- `/object_mask` (sensor_msgs/Image): Segmentation mask when SAM2 is enabled

### Subscribed Topics

- `/camera/color/image_raw` (sensor_msgs/Image): RGB camera stream
- `/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `/camera/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image): Depth aligned to the color image for gesture recognition
- `/camera/camera/color/image_raw` (sensor_msgs/Image): RGB stream used by gesture recognition and Grounding DINO
- `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics used by gesture recognition and Grounding DINO
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image): Default Grounding DINO depth topic
- `/camera/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image): Preferred Grounding DINO depth topic for target poses

## TF Frames

The package publishes transforms for detected markers:
- `aruco_0`, `aruco_1`, etc. (one for each detected marker)

## Coordinate Systems

- **Camera Frame**: Standard ROS camera frame (Z-forward, Y-down, X-right)
- **ArUco Frame**: Z-axis pointing up from marker, X-axis along marker edge
- **Robot Frame**: Your robot's base_link frame

## Example Output

When a marker is detected, you'll see:
```
[aruco_detector]: Marker 0: Position [x=0.234, y=-0.067, z=0.456], Rotation [r=2.3°, p=-1.2°, y=45.6°]
```

## Troubleshooting

### No markers detected
- Check marker size parameter matches physical markers
- Ensure good lighting conditions
- Verify camera is publishing images: `ros2 topic echo /camera/color/image_raw`

### Gesture node does not start
- Pass a valid `checkpoint_path`; the node will fail fast if the model file is missing.
- Install runtime Python dependencies in the same environment as ROS: `mediapipe`, `torch`, `opencv-python`, `numpy`.

### Gesture predictions are unstable
- Confirm the checkpoint was trained from `landmark_with_xyz.py` / `train_with_angles.py` or `train_xyz.py`.
- Make sure RealSense depth is aligned to color. `gesture_pipeline.launch.py` enables `enable_sync` and `align_depth.enable`.

### Grounding DINO import or startup failures
- Use the ROS interpreter explicitly when installing packages:
  `pip --python /usr/bin/python3 ...`
- Pin `transformers==4.33.2`; newer versions removed APIs used by this Grounding DINO code.
- Ensure `PYTHONPATH` includes both:
  - `Grounded-SAM-2`
  - `Grounded-SAM-2/grounding_dino`
- Confirm the checkpoint exists at:
  `Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth`

### Grounding DINO crashes with `_ARRAY_API not found` or `cv_bridge` errors
- This means `numpy 2.x` was installed into `/usr/bin/python3`.
- Reinstall compatible versions:

```bash
pip --python /usr/bin/python3 uninstall -y numpy opencv-python opencv-contrib-python
pip --python /usr/bin/python3 install --user --force-reinstall \
    "numpy==1.26.4" \
    "opencv-contrib-python==4.10.0.84"
```

### Grounding DINO warns `Failed to load custom C++ ops. Running on CPU mode Only!`
- This is expected in the current WSL setup.
- The patched local repo falls back to a pure PyTorch implementation.
- It is slower, but it is sufficient for functional testing.

### TF transform errors
- Check that camera transforms are correctly configured
- Verify all required frames exist: `ros2 run tf2_tools view_frames`

### Camera not found
- Check RealSense connection: `rs-enumerate-devices`
- Install RealSense SDK if needed

## Integration with Motion Planning

To use detected poses for motion planning:

```python
# In your motion planning node
from geometry_msgs.msg import PoseStamped

def aruco_pose_callback(self, msg):
    # msg contains the ArUco pose in base_link frame
    target_pose = msg.pose
    # Use this pose for IK or motion planning
    self.move_to_pose(target_pose)

# Subscribe to transformed poses
self.pose_sub = self.create_subscription(
    PoseStamped,
    '/aruco_pose_base_link',
    self.aruco_pose_callback,
    10
)
```
