# Hall touch tool with D405 camera

Select this tool with `hall_touch_camera:=true` for a single robot, or
`left_hall_touch_camera:=true` / `right_hall_touch_camera:=true` for a dual
robot. The new tool takes precedence over `basic_camera`, `calib_tool`, and
`cellphone_holder` on the same arm. The dual launch defaults to the old
`left_basic_camera` tool,
so setting `left_hall_touch_camera:=true` is sufficient to select this one.

From the root of a built ROS 2 workspace, spawn both VS050 robots in Gazebo
with the Hall touch tool and D405 on the left arm and the cellphone holder
with AprilTags on the right arm:

```bash
source install/setup.bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=true rviz:=true use_servo:=false \
  left_basic_camera:=false left_hall_touch_camera:=true \
  right_cellphone_holder:=true
```

Add `right_virtual_phone:=true` to the launch command to show the optional
phone model in the holder. The phone is disabled by default. For the Docker
command, see `denso_docker/README.md`.

The three tool links (with `left_` or `right_` prefixes for a dual robot) are:

| Link | J6 translation (m) | Meaning |
| --- | --- | --- |
| `hall_camera_support_link` | `(0, -0.050, 0.035500)` | Measured point on the camera support plane, Fusion orientation |
| `calib_link` | `(0, 0, 0.105987)` | Hall finger touch tip, preserving the existing TCP name |
| `hall_camera_optical_frame` | `(-0.0090235, -0.050, 0.054700)` | Nominal left imager optical center; ROS optical axes |

The supplied STL is in Fusion millimetres. Its mounting origin is placed at
J6 with no rotation or gap, and the STL is scaled by 0.001. The mesh reaches
Z = 105.9867 mm, matching the measured tip. The support link is offset from
J6, while its visual and collision mesh are shifted back to the Fusion
origin. The optical frame is rotated 180° about tool Z: optical +X points
along tool -X, +Y along tool -Y, and +Z along tool +Z.

`config/d405_calibration.json` is a copy of the supplied calibration data.
Its `rectified.0` entry supplies the simulated 1920 × 1080 camera model:
`fx = fy = 954.334`, `cx = 940.535`, `cy = 540.252` pixels. The baseline
is 18.047 mm. These are **image intrinsics and stereo calibration**, not a
camera-to-tool measurement. The optical center above uses the existing
`basic_camera` tool's nominal D405 geometry: half the measured stereo
baseline toward tool -X, and 3.8 mm behind the front of a 23 mm camera
mounted on the Z = 35.5 mm support plane. Its translation and image roll
should be measured on the assembled hardware before precision camera-to-touch
work. The 940.535 px principal point is an image coordinate; it is not an
extra millimetre offset of the optical frame.

The Gazebo camera publishes `hall_camera`, `hall_camera/camera_info`,
`hall_camera/depth/image_raw`, and `hall_camera/depth/camera_info` under the
robot namespace. Gazebo models a rectified pinhole camera; the raw stereo
distortion and factory rectification matrices are retained in the calibration
JSON but are not applied to the simulator.

The STL has an opaque camera front near J6 Z = 58.43 mm. Gazebo renders the
RGB and depth images from J6 Z = 59.5 mm, about 1 mm beyond that face, to
avoid showing the tool housing in its own images. The nominal optical TF
remains at J6 Z = 54.7 mm, so image parallax in the simulation includes
a 4.8 mm forward render offset. Measure the real lens pose before using
these frames for precision vision-guided contact.
