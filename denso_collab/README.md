# DENSO cellphone collaboration

Start the dual-robot bringup in a separate terminal. Then run the screen
approach in simulation:
```bash
ros2 launch denso_robot_bringup dual_denso_robot_bringup.launch.py
model:=vs050 sim:=true rviz:=true use_servo:=true
left_hall_touch_camera:=true right_cellphone_holder:=true right_virtual_phone:=true
```

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
the sensor's hardware protocol is not implemented here. Touch approaches at
20 mm/s, stops on contact, then retracts 30 mm at 100 mm/s in simulation;
`touch_dwell_sec: 0.0` adds no wait after the controlled stop. These values,
the 35 mm approach travel bound, and the 30 s timeout are configured in
`cellphone_collab.yaml`. Simulation accepts up to 50 mm/s approach and
100 mm/s retraction; real mode retains 20 mm/s and 30 mm/s limits.
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
