# solar_hunter_cpp

C++ port of the [solar-hunter](../solar-hunter) pipeline: detect a bright
circular light blob (a white LED or a patch of yellow sunlight) in a camera
frame, and compute the pan/tilt angles for a parabolic reflector mounted on a
MeArm gripper so it redirects that light toward a chosen target direction.

This is a ROS 2 (`ament_cmake`) node — unlike the standalone Python original,
it publishes the resulting pan/tilt (radians, patched into the arm's current
joint vector) on `/arm_joint_commands` (`std_msgs/Float64MultiArray`) and
subscribes to `/joint_states` to track the arm's known joints.

The package lives **standalone** (outside any ROS 2 workspace) at
`/home/ikbal/solar_hunter_cpp/`. It still needs a ROS 2 installation to
compile against (rclcpp, sensor_msgs, OpenCV), but it no longer needs to live
inside `ros2_ws/src/`.

## Files

| File | Purpose |
|------|---------|
| `build_standalone.sh` | Build the package with an isolated colcon workspace |
| `run_solar_hunter.sh` | Run the C++ solar-hunter pipeline (like solar-hunter/main.py) |
| `scripts/01_serial_bridge.sh` | Serial bridge to STM32 (S-curve enabled) |
| `scripts/02_description_rviz.sh` | Robot description + RViz |
| `scripts/03_object_detection.sh` | C++ object detection / aiming node (camera index 4 by default) |
| `scripts/04_visual_servoing.sh` | C++ visual servoing (S-curve enabled) |
| `scripts/run_all.sh` | Launch all 4 stages in separate xterm windows |
| `scripts/stop_all.sh` | Stop all 4 stages |
| `include/solar_hunter_cpp/targets.hpp` | Named aim directions (`locusts`, `meteors`, `space_debris`, `moon_south_darkside`, `solar_sail`) |
| `include/solar_hunter_cpp/reflector_angle.hpp` | Aiming math: direction vector → pan/tilt servo angles, focus-centering correction |
| `include/solar_hunter_cpp/detector.hpp` | HSV + contour/circularity light-blob detector |
| `src/solar_hunter_node.cpp` | ROS 2 node: camera loop, detection, aiming, joint-command publishing |
| `config/solar_hunter_params.yaml` | Default parameters |
| `launch/solar_hunter.launch.py` | ROS 2 launch file |

## Building (standalone)

```bash
cd /home/ikbal/solar_hunter_cpp
./build_standalone.sh          # builds + installs into ./install/
./build_standalone.sh --clean  # wipe and rebuild
```

This sources your ROS 2 install, builds with colcon into an isolated workspace,
and installs into `./install/`.

## Running

```bash
# Aim-only mode (no camera) — computes and publishes pan/tilt for 'meteors'
./run_solar_hunter.sh --no-camera

# With live focus-centering camera loop (index 4 = Integrated Camera)
./run_solar_hunter.sh --camera 4 --target meteors

# Different target, no debug windows
./run_solar_hunter.sh --target locusts --camera 0 --no-debug

# Build first if the binary isn't present yet
./run_solar_hunter.sh --build --target meteors --no-camera

# Or run the ROS 2 node directly after sourcing:
source install/setup.bash
ros2 run solar_hunter_cpp solar_hunter_cpp_node \
  --ros-args -p camera_index:=-1 -p target:=meteors

# Via launch file
source install/setup.bash
ros2 launch solar_hunter_cpp solar_hunter.launch.py \
  target:=meteors camera_index:=4
```

## Running the full rig

Each stage has its own script under `scripts/`, matching the 4 terminals
used to run the rig by hand:

```bash
scripts/01_serial_bridge.sh       # Serial bridge to STM32 (S-curve enabled)
scripts/02_description_rviz.sh    # Robot state publisher + RViz
scripts/03_object_detection.sh    # C++ object detection / aiming (camera 4)
scripts/04_visual_servoing.sh     # C++ visual servoing (S-curve enabled)
```

Run them in 4 separate terminals, or start all 4 at once (each in its own
xterm window) with:

```bash
scripts/run_all.sh
```

Override the serial port or camera index via environment variables:

```bash
SERIAL_PORT=/dev/ttyACM1 CAMERA_INDEX=0 scripts/run_all.sh
```

Stop every stage with:

```bash
scripts/stop_all.sh
```

## Command-line options (run_solar_hunter.sh)

| Option | Meaning |
|--------|---------|
| `--target NAME` | Aim target: locusts, meteors, space_debris, moon_south_darkside, solar_sail (default: meteors) |
| `--camera IDX` | V4L2 camera index for the focus-centering loop (default: 4) |
| `--no-camera` | Aim-only mode, no camera loop (camera_index=-1) |
| `--publish-rate HZ` | Joint-command publish rate (default: 10) |
| `--no-debug` | Disable OpenCV debug windows |
| `--build` | Run build_standalone.sh before starting |

Press `ESC` in the debug window to shut the node down.

## Camera index

On this machine index 4 (`/dev/video4`, "Integrated Camera") is the working
RGB camera; index 5 exists but OpenCV cannot capture from it. See
`v4l2-ctl --list-devices` for your hardware.

## Porting notes

The C++ version mirrors the Python original module-for-module:

* `targets.py` → `targets.{hpp,cpp}` — same 5 named targets, az/el → unit vector
* `reflector_angle.py` → `reflector_angle.{hpp,cpp}` — same servo-angle math and focus-centering correction
* `detector.py` → `detector.{hpp,cpp}` — same HSV white/yellow masking, morphology, contour circularity ranking
* `main.py` → `solar_hunter_node.cpp` + `run_solar_hunter.sh` — same pipeline, but as a long-lived ROS 2 node with a timer-driven camera loop instead of a blocking `while` loop
