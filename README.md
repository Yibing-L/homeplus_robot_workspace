# HomePlus Robot Workspace

This repository contains ROS 2 packages for the HomePlus robot, including URDF description, MoveIt 2 configuration, and custom IK/planning scripts.

## Packages
- `homeplus_urdf`: Robot description (URDF, meshes, RViz config)
- `homeplus_moveit_config`: MoveIt 2 configuration, planning, and IK scripts

## Build and Launch Instructions

### 1. Build the workspace
```bash
colcon build
```

### 2. Source the workspace
```bash
source install/setup.bash
```

### 3. Visualize robot in RViz
```bash
ros2 launch homeplus_urdf display.launch.py
```

### 4. Build with symlink install (for development)
```bash
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source ~/install/setup.bash
```

### 5. Launch MoveIt 2
```bash
ros2 launch homeplus_moveit_config moveit.launch.py
```

### 6. Run IK node as ROS 2 executable
```bash
source /opt/ros/humble/setup.bash
source ~/install/setup.bash
ros2 run homeplus_moveit_config ik.py
```

### 7. Clean build artifacts
```bash
rm -rf build/ install/ log/
```


