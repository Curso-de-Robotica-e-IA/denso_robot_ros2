# DENSO cellphone-holder vision

`cellphone_holder_detector` detects AprilTag 36h10 IDs 1, 2, 3 and 4,
uses their 16 corners to estimate a pixel-to-plane homography, and publishes
the requested image point as a metric coordinate on the holder plane.

The measured layout is treated as an isosceles trapezoid:

```text
ID 1 -------- 95.2 mm -------- ID 2
  \                                /
   207.5 mm                  207.5 mm
     \                            /
ID 4 -------- 99.8 mm -------- ID 3
```

The resulting row separation is 207.4872534 mm. Each detected tag square is
21.5 mm, so each tag corner is 10.75 mm from its centre along X and Y. The
horizontal clear corner-to-corner gaps are therefore 73.7 mm on the top row
and 78.3 mm on the bottom row; the corresponding side gap is approximately
186.0 mm. The node uses the exact 16 corner coordinates rather than these
rounded gap values. The
holder-plane origin is at the centre of the layout, +X points right, +Y points
toward IDs 1/2, and +Z is normal to the front of the holder. All ROS positions
are published in metres.

## Build and run

OpenCV must include the `aruco` module (`DICT_APRILTAG_36h10`). On Ubuntu/ROS
install the package dependencies with rosdep, then build:

```bash
rosdep install --from-paths denso_vision --ignore-src -r -y
colcon build --packages-select denso_vision
source install/setup.bash
ros2 launch denso_vision cellphone_holder_vision.launch.py
```

By default (`image_source:=realsense`) the node reads a connected D405 directly
through `pyrealsense2` at 1280×720 and 30 FPS, the D405's maximum supported
color mode. Install it with
`pip install pyrealsense2`. For simulation or rosbag playback, subscribe to a
ROS image instead:

```bash
ros2 launch denso_vision cellphone_holder_vision.launch.py \
  image_source:=topic image_topic:=/basic_camera
```

For a namespaced camera, override the image topic. For example:

```bash
ros2 launch denso_vision cellphone_holder_vision.launch.py \
  image_source:=topic image_topic:=/left_basic_camera
```

For offline homography validation, replay an AVI directly. The supplied video
is looped by default and uses its recorded FPS; `video_loop:=false` stops after
the final frame.

```bash
ros2 launch denso_vision cellphone_holder_vision.launch.py \
  image_source:=video \
  video_path:='/videos/Realsense 14-09-2026/20260914_165123_compact.avi'
```

The default target is the centre of each incoming image. Set
`target_pixel_x` and `target_pixel_y` in the YAML file, or publish a pixel at
runtime:

```bash
ros2 topic pub --once /target_pixel geometry_msgs/msg/PointStamped \
  "{point: {x: 640.0, y: 360.0, z: 0.0}}"
```

## Topics

| Topic | Type | Meaning |
| --- | --- | --- |
| `tag_corners` | `geometry_msgs/msg/PolygonStamped` | 16 image pixels, grouped by IDs 1-4; each group is TL, TR, BR, BL |
| `homography` | `std_msgs/msg/Float64MultiArray` | Row-major 3x3 transform from image pixels to holder-plane metres |
| `homography_rmse_mm` | `std_msgs/msg/Float64` | Fit error over all 16 corners, in millimetres |
| `target_point` | `geometry_msgs/msg/PointStamped` | Selected pixel expressed as `(x, y, 0)` on the holder plane |
| `debug_image` | `sensor_msgs/msg/Image` | Detected tags and selected pixel overlay |
| `target_pixel` | `geometry_msgs/msg/PointStamped` | Runtime input selecting an image `(u, v)` pixel |

Homography alone recovers coordinates only on the known plane: its third
coordinate is therefore exactly zero in `cellphone_holder_tags_frame`. It
does not recover the depth of an arbitrary point away from that plane.

## Simulation-size note

The tag PNG files include a white margin: the detectable black tag occupies
80% of the textured square. To render a 21.5 mm detectable marker, launch the
robot description with `cellphone_holder_tag_size:=0.026875`. The current
description default of 0.031 renders a 24.8 mm detectable black square, which
does not match the physical measurement used by this node.
