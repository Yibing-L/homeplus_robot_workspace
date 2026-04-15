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
ros2 launch homeplus_vision camera_only.launch.py
```

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

### Subscribed Topics

- `/camera/color/image_raw` (sensor_msgs/Image): RGB camera stream
- `/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- `/camera/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image): Depth aligned to the color image for gesture recognition

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
