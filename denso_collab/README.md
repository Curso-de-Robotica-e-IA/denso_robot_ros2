# DENSO cellphone collaboration

For the concise real-hardware procedure, see [REAL_TESTS.md](REAL_TESTS.md).

Start the dual-robot bringup in a separate terminal. Then run the screen
approach in simulation:
```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=true rviz:=true use_servo:=true \
  left_hall_touch_camera:=true right_cellphone_holder:=true right_virtual_phone:=true
```

For the real dual VS050 installation, `sim:=false` automatically uses
`192.168.160.228` for the left robot and `192.168.160.227` for the right
robot. Start the first hardware check without Servo:

```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=false rviz:=true use_servo:=false \
  left_basic_camera:=false left_hall_touch_camera:=true \
  right_cellphone_holder:=true right_virtual_phone:=false
```

Pass `left_ip_address:=...` or `right_ip_address:=...` only to override those
installed defaults.

The left Hall tool TCP defaults to a `-2 mm` local-X offset and a `-1 mm`
local-Y offset. These values can be overridden from the bringup command
without editing the Xacro. For example, to restore the original CAD position:

```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py \
  model:=vs050 sim:=false rviz:=true use_servo:=false \
  left_basic_camera:=false left_hall_touch_camera:=true \
  right_cellphone_holder:=true right_virtual_phone:=false \
  left_hall_touch_calib_offset_x_mm:=0.0 \
  left_hall_touch_calib_offset_y_mm:=0.0
```

The negative X/Y values are the calibrated correction selected for the left
tool. The right Hall tool keeps `0 mm` defaults for both axes. Restart bringup
after changing either value because the offsets are part of
`robot_description`.

## Real robot + D405 validation

Run the bringup above in terminal A. In terminal B, use the same Humble
environment and confirm MoveIt, controllers, and fresh joint states before
starting any screen test:

```bash
source /opt/ros/humble/setup.bash
source ~/denso_ws/install/setup.bash

ros2 param get /move_group robot_description >/dev/null
ros2 control list_controllers
ros2 topic echo --once /joint_states
```

The joint-state broadcaster and both trajectory controllers must be `active`.
Test the D405 by itself in terminal C:

```bash
python3 - <<'PY'
import pyrealsense2 as rs
for device in rs.context().query_devices():
    print(device.get_info(rs.camera_info.name),
          device.get_info(rs.camera_info.serial_number))
PY

ros2 launch denso_vision cellphone_holder_vision.launch.py \
  image_source:=realsense image_topic:=/left_basic_camera
```

The D405 test publishes `/left_basic_camera` and `/debug_image`. Verify the
streams with `ros2 topic hz /left_basic_camera` and `ros2 topic hz /debug_image`.
Stop this standalone detector with `Ctrl-C` before the next command, because
the screen launch opens the D405 itself.

Finally, run the motion-free integration test:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  image_source:=realsense image_topic:=/left_basic_camera \
  use_random_holder_pose:=false approach_plan_only:=true \
  touch_enabled:=false sim:=false
```

This plan-only run validates MoveIt and the camera alignment without executing
robot motion. It waits for AprilTags 1–4 and one simultaneous red, green, and
blue target trio.

```bash
ros2 launch denso_collab screen_approach.launch.py \
  image_source:=topic image_topic:=/left_basic_camera
```

Add `use_random_holder_pose:=true` to move both robots to one validated random
pose **before** detection and approach. If random planning or execution fails,
the launch does not start the screen stage. Without that option, it uses the
robots' current poses. For a connected RealSense, omit the image arguments;
`image_source:=realsense` is the default. Check camera access separately first:
random motion happens before the detector opens the camera.

With `image_source:=realsense`, the detector opens the D405 itself and
publishes the rectified image on `image_topic` (for example `/left_basic_camera`) and
the annotated detection view on `/debug_image`. In RViz, add an `Image` display
for either topic. Before starting `screen_approach`, keep the bringup running
in another terminal and verify MoveIt responds:

```bash
ros2 node list | grep -Fx /move_group
ros2 service list | grep -Fx /move_group/get_parameters
ros2 param get /move_group robot_description >/dev/null
```

All three commands must succeed. If they do not, restart the dual bringup; a
screen-approach launch cannot start MoveIt itself.

Use `test_runs:=N` (1–100, default 1) for N finite approach tests. With
`use_random_holder_pose:=true`, each test gets a new random holder pose; without
it, each test uses the current pose. A failed random move, alignment, detection,
approach, or return stops the launch. Detection times out after 45 seconds by
default; edit `detection_timeout_sec` in the YAML if needed.

`approach_plan_only:=true` checks both the optional random stage and the screen
stage without executing either plan.

Before accepting a random holder pose, `random_collab_pose` verifies that the
Hall tool can reach a 3 x 3 grid covering the usable phone screen. It samples
the complete normal path from standoff to contact at every grid point and
rejects IK failures, joint-limit violations, and Jacobian condition numbers
above the configured limit. This prevents a camera-centred pose from being
accepted when the tool can only reach the middle of the screen. The grid size,
bezel margin, path samples, and condition-number limit are under
`random_collab_pose` in `cellphone_collab.yaml`.

The camera aligns with the holder, detects simultaneous RGB trios, then
approaches their centers in screen order. The right robot stays still during
the screen stage. The Android test app keeps every target of the batch visible
at once, which is compatible with this trio detector.

Touch is experimental and **off by default**. For simulation, bring up the
virtual phone with `sim:=true right_virtual_phone:=true use_servo:=true`, then
run this launch with `sim:=true touch_enabled:=true`. The launch publishes
`/touch_detected` (`std_msgs/Bool`) from a geometric tip-to-screen check; it
fails closed if the screen TF is missing. For real hardware, use `sim:=false
touch_enabled:=true` and have the ESP32 bridge publish the same boolean topic;
the sensor's hardware protocol is not implemented here. Touch approaches at
the configured speed, stops on contact, then retracts at the configured speed;
`touch_dwell_sec: 0.0` adds no wait after the controlled stop. These values,
the 35 mm approach travel bound, and the 30 s timeout are configured in
`cellphone_collab.yaml`. The real mode accepts at most 200 mm/s approach and
400 mm/s retraction.
In simulation with touch enabled, the standoff is 15 mm from the virtual
screen surface; on the real setup it is 30 mm from the physical screen. The
simulated phone is another robot link,
so the bringup selects a 1 mm Servo **self-collision proximity** margin only in
this configuration; collision checking stays enabled and the real-robot margin
remains 10 mm. Do not treat the 2 mm geometric signal as a validated
force/contact sensor or an Android touch event. Confirm a single touch under
supervision before using `test_runs` with touch enabled.

Edit [cellphone_collab.yaml](config/cellphone_collab.yaml) for observation and
approach distances, motion limits, planning, and calibration. Edit
[right_holder_poses.yaml](config/right_holder_poses.yaml) for validated random
anchors. Restart the launch after changing configuration. The detector's image
and target settings live in
[denso_vision/config/cellphone_holder.yaml](../denso_vision/config/cellphone_holder.yaml).
`screen_plane_offset_m` is the measured +Z distance from the ArUco plane to the
physical screen; its default is 4.5 mm. Consequently, `approach_distance_m` is
now measured from the screen surface.

Standalone tools remain available: `random_collab_pose`, `align_tool`,
`record_right_pose`, `move_right_to_holder_pose`, and `move_to_collab_start`.
The latter is only a simulation test and is not part of the launch.
