# Touch tool with RealSense D405

`basic_camera:=true` now attaches `toucher_with_realsense.stl` to J6,
replacing the placeholder camera box and its visual direction cylinder.
The dual launch enables this assembly on the left arm by default.

```bash
colcon build --packages-select denso_robot_descriptions denso_robot_moveit_config denso_robot_bringup
source install/setup.bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py model:=vs050 sim:=true rviz:=true left_basic_camera:=true right_cellphone_holder:=true use_servo:=false
```

For one robot, use `denso_robot_bringup.launch.py ... basic_camera:=true`.
VS050 and VS060 use this tool macro. Setting both `basic_camera` and
`calib_tool` attaches only the new assembly; `calib_tool` alone still selects
the standalone calibration tool. The `calib_xyz` / `calib_mesh_xyz` options
apply only to that standalone tool.

## Geometry and TF

The supplied STL is in millimeters and is loaded with scale `0.001`.
Its mounting face is Z=0, with the touch tip at (0, 0, 50) mm. The mesh
is mounted with a +90° rotation about J6 Z and no added mounting gap.

Frame names below acquire the robot prefix (`left_`, `right_`, or the single
robot's `namespace` argument).

| Frame | Purpose / nominal transform |
| --- | --- |
| `toucher_link` | Complete assembly, J6 origin with +90° rotation about Z |
| `calib_link` | Contact point (TCP), retaining the existing name: J6 → TCP = XYZ (0, 0, 0.050) m, RPY (0, 0, π/2) |
| `basic_camera_link` | Gazebo camera convention: +X forward along J6 +Z |
| `camera_depth_optical_frame` | ROS optical convention: +X right, +Y down, +Z forward |
| `camera_color_optical_frame` | Coincident with the depth optical frame (D405 left imager RGB) |

For example, inspect the TCP with:

```bash
ros2 run tf2_ros tf2_echo left_J6 left_calib_link
```

The CAD camera envelope is 42 × 42 × 23 mm: X ±21 mm,
Y 17.876915–59.876915 mm, Z 4–27 mm. The sensor looks out of the Z=27 mm
front plate. Nominal left-imager origin is (-9, 38.876915, 23.2) mm
relative to `toucher_link`: half of the 18 mm stereo baseline and 3.8 mm behind the
front plate, following the [official D405 ROS description](https://github.com/realsenseai/realsense-ros/blob/ros2-master/realsense2_description/urdf/_d405.urdf.xacro).
Image right/down follow tool -X/-Y, with forward along tool +Z. This 180°
sensor-roll correction applies to both RGB and depth and their optical TFs.
The mesh mounting and camera position stay fixed; alignment targets the
corrected `basic_camera_link` frame.
These camera extrinsics are nominal, not a hand–eye calibration. Verify
image roll and calibrate the physical camera before using measured poses.

The complete STL is used for visual and collision geometry. Assembly mass
(0.16 kg), center of mass and box inertia are estimates; the STL has no
material/mass metadata. Only the mounting pair J6 ↔ toucher_link is excluded
from collision checking; the rest of the assembly remains collision checked.
The TCP is a fixed TF link, usable as a MoveIt pose target; existing arm
solver tips and Servo command frames remain J6.

## Simulated camera

The [D400 datasheet, August 2025](https://realsenseai.com/wp-content/uploads/dlm_uploads/2025/08/Intel-RealSense-D400-Series-Datasheet-August-2025.pdf)
specifies **84° horizontal × 58° vertical** for D405 HD (16:9), for both
color and depth. This mode-specific specification is used here; the
[product page](https://www.realsenseai.com/products/stereo-depth-camera-d405/)
quotes a general 87° × 58° FOV.

- RGB and depth: 1280 × 720 at 30 Hz simulation time.
- Separate pinhole focal lengths preserve both FOVs:
  fx = 710.7920 px, fy = 649.4572 px, cx = 640, cy = 360.
- The supplied world uses Ogre2. Ogre1 ignores the depth camera's custom
  vertical projection on the installed Gazebo version, narrowing its VFOV.
- Depth clip: 0.07–0.50 m, modeling the advertised ideal working range.
  The real camera's minimum depth depends on resolution/preset; this clip
  is a simulation operating window, not a guarantee of hardware accuracy.
- RGB render clip: 0.005–100 m. Color remains visible beyond the depth
  working range. The near clip clears the CAD front cover.
- Gazebo renders ideal color/depth; it does not emulate stereo matching,
  factory distortion, texture-dependent failures, or submillimeter accuracy.

| ROS topic (add robot prefix after `/`) | Message |
| --- | --- |
| `/basic_camera` | RGB `sensor_msgs/msg/Image` (existing topic) |
| `/basic_camera/camera_info` | RGB `sensor_msgs/msg/CameraInfo` |
| `/basic_camera/depth/image_raw` | Depth `sensor_msgs/msg/Image`, float meters |
| `/basic_camera/depth/camera_info` | Depth `sensor_msgs/msg/CameraInfo` |

The launches bridge both images and camera information from Gazebo to ROS
only when simulation and that arm's camera are enabled. To obtain a ROS
optical-frame point cloud, reconstruct it from the depth image and its
CameraInfo (for example with `depth_image_proc`). Gazebo's native depth
cloud is not bridged: this installed Gazebo version emits body-axis XYZ
with an optical-frame header.

## Validation

Checked all 16 single/dual, VS050/VS060, camera/calibration-tool combinations
for valid URDF/SRDF link references, unique links and the 50 mm TCP.
An isolated Gazebo rendering test using the launch's ROS bridge nodes
received both images and CameraInfo with the expected resolution and TF
headers. A 200 × 120 mm board at 200 mm depth measured 0.200 m and occupied
710 × 390 pixels, matching the configured 84° × 58° projection.
