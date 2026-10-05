# DENSO cellphone collaboration

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
publishes the raw image on `image_topic` (for example `/left_basic_camera`) and
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

`approach_plan_only:=true` checks the screen stage without executing its plans.
It does **not** stop the optional random stage from moving the robots.

The camera aligns with the holder, detects simultaneous RGB trios, then
approaches their centers in screen order. The right robot stays still during
the screen stage. The current Android app source displays one target at a time,
so its current test screen is not compatible with this trio detector.

Touch is experimental and **off by default**. For simulation, bring up the
virtual phone with `sim:=true right_virtual_phone:=true use_servo:=true`, then
run this launch with `sim:=true touch_enabled:=true`. The launch publishes
`/touch_detected` (`std_msgs/Bool`) from a geometric tip-to-screen check; it
fails closed if the screen TF is missing. For real hardware, use `sim:=false
touch_enabled:=true` and have the ESP32 bridge publish the same boolean topic;
the sensor's hardware protocol is not implemented here. The physical probe
approaches at 5 mm/s, stops on contact, then retracts 30 mm at 30 mm/s;
`touch_dwell_sec: 0.0` adds no wait after the controlled stop. These values,
the 35 mm approach travel bound, and the 30 s timeout are configured in
`cellphone_collab.yaml`. The real mode accepts at most 20 mm/s approach and
30 mm/s retraction.
The holder mesh represents the backing behind the physical phone. Its ordinary
collision margin prevents the Hall tip from reaching the glass. For a
supervised physical touch test, set `touch_disable_collision_check:=true`:

```bash
ros2 launch denso_collab screen_approach.launch.py \
  image_source:=realsense image_topic:=/left_basic_camera \
  use_random_holder_pose:=false approach_plan_only:=false \
  touch_enabled:=true touch_disable_collision_check:=true \
  max_targets:=1 sim:=false
```

That flag sets `moveit_servo.check_collisions` to `false` only while the
straight, bounded Hall probe is advancing, then pauses Servo and restores
collision checking before the launch continues or exits. Singularity, joint
limit, 35 mm travel, 30 s timeout, and Hall contact checks stay active. Keep
hands clear and run one target under supervision.
`max_targets:=1` makes this initial run stop after the first ordered target
(red in the current layout); the default `max_targets:=0` runs every detected
target.
In simulation with touch enabled, the standoff is 15 mm from the holder plane;
the normal approach remains 30 mm. The simulated phone is another robot link,
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

Standalone tools remain available: `random_collab_pose`, `align_tool`,
`record_right_pose`, `move_right_to_holder_pose`, and `move_to_collab_start`.
The latter is only a simulation test and is not part of the launch.
