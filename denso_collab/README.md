# DENSO cellphone collaboration

`move_to_collab_start` remains a simulation test with saved joint poses. It is
not part of the normal collaboration launch:

```bash
ros2 run denso_collab move_to_collab_start

ros2 run denso_collab move_to_collab_start --ros-args -p plan_only:=true
```

Align the D405 to the holder with:

```bash
ros2 run denso_collab align_tool
```

To collect a candidate holder pose, move only the right robot in RViz (or on
the real cell) and capture its current six joints:

```bash
ros2 run denso_collab record_right_pose --ros-args -p name:=upper_left
```

It prints one YAML line. A candidate is retained only after `align_tool` can
plan to the holder. We will select randomly from that validated set rather
than generate arbitrary joint values.

The current Gazebo candidates are in `config/right_holder_poses.yaml`.

Move the right robot to one captured pose, then align the left camera:

```bash
ros2 run denso_collab move_right_to_holder_pose --ros-args \
  -p target:="[-2.64757900107895e-05, 0.55618170421763813, 1.9349063120042773, -7.6274262268908639e-05, -0.92117201453096742, 9.0075032430075844e-05]"
ros2 run denso_collab align_tool
```

Run one random collaboration test:

```bash
ros2 run denso_collab random_collab_pose
```

It perturbs one validated right-arm anchor, uses IKFast and FK to build a
combined 12-joint candidate, then collision-checks and executes one `dual_arm`
plan. Set `planning_mode` to `sequential` in `config/cellphone_collab.yaml` to
plan and execute the right arm followed by the left alignment. The bounded
offsets, tilts, attempts, validation timeout, and camera distance are also
configured there.

The normal launch uses the current joint state of both robots. Before each
red, green, and blue target, it aligns the D405 to the holder's current TF,
then moves `left_calib_link` to the target at a fixed screen-normal standoff.
It ignores dot detections until the first alignment has completed.
The holder stays still. Alignment and standoff parameters are in
`config/cellphone_collab.yaml`. `touch_screen()` is deliberately only a
placeholder between moves.

```bash
ros2 launch denso_collab cellphone_collab.launch.py \
  image_source:=topic image_topic:=/left_basic_camera standoff_m:=0.1
```
