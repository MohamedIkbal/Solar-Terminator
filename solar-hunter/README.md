# solar-hunter

Standalone OpenCV + aiming-math project: detect a bright circular light
blob (a white LED or a patch of yellow sunlight) in a camera frame, and
compute the pan/tilt angle for a parabolic reflector mounted on a MeArm
gripper so it redirects that light toward a chosen target direction.

This folder also carries the full launch setup for the mEArm rig it
targets — serial bridge, robot description/RViz, camera-based object
detection, and the C++ visual servoing node — so the whole stack can be
started from here.

## Running the full rig

Each stage has its own script under `scripts/`, matching the 4 terminals
used to run the rig by hand:

```bash
scripts/01_serial_bridge.sh       # Serial bridge to STM32 (S-curve enabled)
scripts/02_description_rviz.sh    # robot_state_publisher + RViz
scripts/03_object_detection.sh    # camera-based object detection (camera 5)
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

Once the rig is up, run the solar-hunter aiming pipeline (`main.py`,
below) to detect the light and publish the reflector's pan/tilt.

## Files

- `detector.py` — HSV + contour/circularity based blob detector for a
  white-LED or yellow-sunlight circular highlight. Run directly
  (`python3 detector.py`) for a live webcam preview with the detected
  blob outlined.
- `targets.py` — named aim directions (`locusts`, `meteors`,
  `space_debris`, `moon_south_darkside`, `solar_sail`), each stored as
  an azimuth/elevation pair and converted to a 3D unit vector. These
  five are symbolic placeholder directions, not live-tracked positions
  — swap in a real tracking feed (ephemeris, radar, lidar) to replace
  any of them.
- `reflector_angle.py` — the aiming math. The reflector's focal length
  is 1 m (`FOCUS_DISTANCE_M`); with the light source held at the focus,
  a parabolic mirror's reflected rays leave parallel to the mirror's
  own axis, so aiming reduces to pointing that axis at the target's
  direction vector. Also includes a small-angle correction term that
  uses the detected blob's pixel offset from frame-center to keep the
  light source centered on the focal point.
- `main.py` — ties it together: pick a target, compute pan/tilt,
  optionally run a live camera loop that keeps the blob centered on the
  focus while holding the target aim. If ROS 2 (`rclpy`) is available
  it publishes the resulting joint angles (radians) on
  `/arm_joint_commands` (`std_msgs/Float64MultiArray`), patching only
  the pan/tilt entries of the arm's current joint vector; without ROS 2
  it just prints the computed angles.
- `requirements.txt` — `opencv-python`, `numpy`.

## Usage

```bash
python3 main.py --list-targets
python3 main.py --target meteors
python3 main.py --target meteors --camera 0   # also run the live focus-centering loop
```

## Adjusting to the real rig

- `PAN_JOINT_INDEX` / `TILT_JOINT_INDEX` in `main.py` — set these to
  match the actual MeArm joint order (check `ros2 topic echo
  /joint_states` for the `name[]` order).
- `PAN_SERVO_RANGE_DEG` / `TILT_SERVO_RANGE_DEG` in
  `reflector_angle.py` — set to the physical servo limits.
- `WHITE_LOWER/UPPER`, `YELLOW_LOWER/UPPER` in `detector.py` — tune to
  the actual camera's exposure and the LED/sunlight brightness.
- `CAMERA_HFOV_DEG` / `CAMERA_VFOV_DEG` in `reflector_angle.py` — set
  to the real camera/lens field of view for accurate focus-centering.
