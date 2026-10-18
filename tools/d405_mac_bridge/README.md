# D405 Mac bridge

UTM USB passthrough cannot carry RealSense depth (frames stall within seconds, then the
camera drops out of the VM). The D405 therefore stays on the Mac; this bridge streams
color + depth aligned to color + intrinsics into ROS in the VM on the same topics/frames
realsense2_camera uses (/cameras/d405/color/image_raw, .../aligned_depth_to_color/image_raw,
both camera_info, static TF hand_camera_link -> d405_link -> d405_color_optical_frame).
Stamps = VM ROS time at receipt (never ahead of the clock); color and depth share a stamp.

1. UTM USB menu: D405 must be UNCHECKED (Mac owns it). Arduino stays checked.
2. Mac terminal (root needed by librealsense on macOS):
       cd tools/d405_mac_bridge && sudo .venv/bin/python mac_sender.py --fps 5
3. VM (in place of multi_camera.launch.py / the camera part of run_vision.sh):
       FASTRTPS_DEFAULT_PROFILES_FILE=~/d405_fastdds.xml python3 ~/d405_vm_receiver.py
   The XML enlarges Fast DDS shared memory; without it ~75% of 1 MB depth frames are dropped.

Vision launches must use enable_d405:=false (else realsense2_camera fights for the topic names).
Detector runs at 0.5 Hz, so 5 fps is plenty.
