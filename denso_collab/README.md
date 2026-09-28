# DENSO cellphone collaboration

Start the dual-robot bringup in a separate terminal. Then run the screen
approach in simulation:

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

`approach_plan_only:=true` checks the screen stage without executing its plans.
It does **not** stop the optional random stage from moving the robots.

The camera aligns with the holder, detects the colored targets, then approaches
their centers in screen order. The right robot stays still during the screen
stage. `touch_screen()` is not implemented yet; this is an approach test, not a
touch test.

Edit [cellphone_collab.yaml](config/cellphone_collab.yaml) for observation and
approach distances, motion limits, planning, and calibration. Edit
[right_holder_poses.yaml](config/right_holder_poses.yaml) for validated random
anchors. Restart the launch after changing configuration. The detector's image
and target settings live in
[denso_vision/config/cellphone_holder.yaml](../denso_vision/config/cellphone_holder.yaml).

Standalone tools remain available: `random_collab_pose`, `align_tool`,
`record_right_pose`, `move_right_to_holder_pose`, and `move_to_collab_start`.
The latter is only a simulation test and is not part of the launch.
