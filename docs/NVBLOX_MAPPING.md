# Nvblox collision mapping

The default manipulation launch now uses NVIDIA nvblox v0.0.10 to integrate a
TSDF on CUDA and sends its reconstructed surface to MoveIt as the world collision
object `nvblox_environment`. OMPL, robot kinematics and STATE4 hardware execution
remain in use. No OctoMap updater or standalone OctoMap server is needed.

## Build

Prerequisites are the existing ROS 2 Humble/MoveIt development environment, CMake
3.22+, a C++17 compiler and a working CUDA toolkit/GPU. This installation uses
Ubuntu 22.04, GCC 11, CUDA 13 and an RTX 4080 Laptop (compute capability 8.9).

```bash
bash scripts/setup_nvblox.sh
source /opt/ros/humble/setup.bash
colcon build --packages-up-to homeplus_moveit_config --symlink-install
source install/setup.bash
```

The setup script pins upstream commit
`c457c3fc01003bec6eba3ec1c61e6bf84bc3f51f`, disables its renderer/PyTorch wrapper,
and installs under `.deps/nvblox`. It does not install system packages or replace
the detector's Python/CUDA environment. On other GPUs, set
`NVBLOX_CUDA_ARCHITECTURES` before running it. Dependency source/build directories
are under `.deps` and excluded from Git and colcon discovery.

The CPU-only ROS/VNC Docker image and a Mac VM without CUDA cannot run this
mapper. Remote detector inference does not offload reconstruction: mapping must
run on a CUDA-capable ROS host.

## Run

Use the three-terminal workflow in `LAUNCH.md`. The existing vision-side
`enable_d405_octomap` switches retain their names for compatibility but control
the generic obstacle cloud; they do not select a reconstruction backend.

```bash
ros2 launch homeplus_moveit_config gesture_control.launch.py
```

Mapping starts alongside MoveIt. A missing CUDA device or a mapper process exit
stops that launch. For hardware-free planning tests, explicitly use
`use_mapping:=false`; this disables depth-based environment collision checking.
The old `use_octomap` argument is accepted as a deprecated enable/disable alias;
it still selects nvblox. Leave `run_rebuild_map:=false`.

To view mapping separately, with vision, robot TF and the hardware state bridge
already running, use `bash scripts/run_occ_map.sh`. This starts its own MoveIt;
do not run it alongside the manipulation launch. It starts no motion executor.
The old `cloud_to_octomap.launch.py` remains only as a deprecated forwarding entry.

```bash
ros2 service call /nvblox/clear std_srvs/srv/Trigger '{}'
ros2 topic echo /vision/planning_map_ready
```

Reset returns `success` only after MoveIt accepts removal of the prior mesh; its
message contains the new map generation. Planning waits for that generation,
at least three fresh integrated frames, a nonempty mesh, and successful
`/apply_planning_scene`. The old cloud-publication notification is not sufficient.
Clearing or scene-update failures stop the stage instead of planning optimistically.

RViz includes `Nvblox collision surface` on `/nvblox/mesh`. MoveIt's planning scene
also displays the actual collision mesh. The mapper never removes unrelated
collision objects or the attached cup.

## Input and filtering

`/obstacle_cloud` is the existing D405 cloud after target masking, depth consistency
and height filtering. It must retain its calibrated optical frame. The mapper
reprojects it to depth at the original pixel sampling stride, using matching
camera intrinsics. The default `depth_downsample: 4` must match the cloud node's
`pixel_stride: 4`. Change both together.

Robot collision bodies, including their URDF origins and 1 cm self-filter padding,
are removed using image-time TF before integration. Missing transforms skip that
observation. Clouds older than two seconds, duplicate timestamps and pre-settling
observations cannot contribute. No mapping occurs before settled hardware status.

The TSDF uses 2 cm voxels, constant observation weights capped at 20, a three-voxel
truncation band and a minimum surface weight of two. The three-frame startup
requirement adds temporal integration compared with the former first-cloud path.
Mapping remains at the cloud's configured 1 Hz; it is not a higher camera rate.
The adapter imposes mesh and allocated-block budgets and stops readiness on scene
rejection, timeout or budget exhaustion. A reset is required to recover.

## Scope and validation

This is static scene reconstruction for staged manipulation. Missing observations
remain unknown; a triangle surface does not classify unseen space as occupied.
There is no dynamic-object decay, reactive obstacle stop, ESDF planning or cuMotion
integration. Incorrect depth, segmentation, odometry or camera calibration can
still produce incorrect surfaces. The supplied Mac camera bridge uses receipt
timestamps, so acquisition-time accuracy there still depends on transport delay.

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
python3 tests/nvblox_smoke.py
python3 tests/moveit_smoke.py
```

The GPU smoke test uses ROS domain 216 and localhost networking. It starts only
MoveIt, robot TF and the mapper, publishes a synthetic plane, verifies accumulation
and reset generations, and proves that MoveIt's collision checker collides with
the reconstructed mesh. It also checks motion gating. No Arduino or cameras are
opened. Passing it does not establish live-camera reconstruction quality or
physical pickup success.

Local validation: all five ROS packages built, 67 offline regressions passed,
and the GPU test reconstructed 1,674 triangles with verified MoveIt collisions,
reset generations, three-frame accumulation and motion gating. The existing
MoveIt smoke test passed FK/IK for four groups, RRTConnect planning, time
parameterization, Cartesian continuation and executor startup. These checks used
synthetic geometry and no physical hardware.

The installed MoveIt still logs a segmentation fault on shutdown after SIGINT,
which was already documented on the closed-loop branch. Its functional checks
complete before that teardown failure. MoveIt's unused occupancy monitor also
logs that no OctoMap sensor plugin is defined; no OctoMap updater is loaded.
