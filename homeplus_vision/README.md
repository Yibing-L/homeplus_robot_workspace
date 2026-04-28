# homeplus_vision

ROS 2 vision tools for the HomePlus robot. The current package supports Intel
RealSense capture, gesture recognition, and Grounding DINO target detection.

## Features

- RealSense RGB-D camera launch files
- Gesture recognition from aligned RGB-D streams
- Grounding DINO target detection
- Optional 3D target pose output through TF2
- Debug image and JSON detection topics

## Dependencies

Install the ROS dependencies:

```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-realsense2-camera
```

Use the ROS Python interpreter for Python packages:

```bash
pip --python /usr/bin/python3 install --user \
  "numpy==1.26.4" \
  "opencv-contrib-python==4.10.0.84" \
  "mediapipe==0.10.14" \
  "torch==2.11.0" \
  "transformers==4.33.2"
```

Important:

- `cv_bridge` in ROS Humble can break with `numpy 2.x`.
- The Grounding DINO stack used here expects `transformers==4.33.2`.
- Keep Python packages in `/usr/bin/python3`, the same interpreter used by `ros2`.

## Package Structure

```text
homeplus_vision/
├── CMakeLists.txt
├── package.xml
├── config/
│   ├── camera_calibration.yaml
│   └── gesture_params.yaml
├── launch/
│   ├── camera_only.launch.py
│   ├── dino_pipeline.launch.py
│   └── gesture_pipeline.launch.py
└── scripts/
    ├── gesture_recognizer.py
    ├── grounding_dino_node.py
    └── test_grounding_dino_static.py
```

## Camera Only

```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_sync:=true \
  align_depth.enable:=true
```

The package also includes `camera_only.launch.py`, but direct RealSense launch
arguments are usually clearer when debugging camera topics.

## Gesture Recognition

```bash
ros2 launch homeplus_vision gesture_pipeline.launch.py \
  checkpoint_path:=/absolute/path/to/model.pt \
  label_map_path:=/absolute/path/to/labels.json
```

The gesture node consumes:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

## Grounding DINO

The DINO node expects a local `Grounded-SAM-2` checkout with Grounding DINO
available on `PYTHONPATH`, plus this checkpoint:

```text
Grounded-SAM-2/gdino_checkpoints/groundingdino_swint_ogc.pth
```

The normal project entrypoint is:

```bash
cd /home/yibing/ros2_ws/homeplus_robot_workspace-occ-map
source install/setup.bash
bash scripts/run_system.sh --with-dino --no-occ-map --task-id 1
```

Useful debug terminals:

```bash
ros2 topic echo /gdino/detections
ros2 topic echo /gdino/mask_center
ros2 topic hz /gdino_debug_image
ros2 run rqt_image_view rqt_image_view /gdino_debug_image
```

The DINO node publishes:

```text
/gdino/detections
/gdino/mask_center
/gdino_debug_image
/gdino_pose_world
/object_mask
```

## Camera Frames

Camera frames are named explicitly to avoid mixing the hand, front, and rear
cameras:

```text
hand_camera_link_1
  hand_camera_link
    hand_camera_color_optical_frame
    hand_camera_depth_optical_frame

front_camera_link_1
  front_camera_link
    front_camera_color_optical_frame
    front_camera_depth_optical_frame

rear_camera_link_1
  rear_camera_link
    rear_camera_color_optical_frame
    rear_camera_depth_optical_frame
```

For DINO 3D pose output, use `hand_camera_color_optical_frame` as the camera TF
frame.

## Troubleshooting

If DINO starts but no detections appear:

```bash
ros2 topic list
ros2 topic hz /camera/camera/color/image_rect_raw
ros2 topic hz /gdino_debug_image
```

If target poses are shifted, inspect the hand camera TF:

```bash
ros2 run tf2_ros tf2_echo base_link hand_camera_color_optical_frame
```

If `cv_bridge` or OpenCV fails after pip installs, restore compatible versions:

```bash
pip --python /usr/bin/python3 uninstall -y numpy opencv-python opencv-contrib-python
pip --python /usr/bin/python3 install --user --force-reinstall \
  "numpy==1.26.4" \
  "opencv-contrib-python==4.10.0.84"
```
