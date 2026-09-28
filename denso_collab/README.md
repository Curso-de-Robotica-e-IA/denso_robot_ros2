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

The normal launch uses the current joint state of both robots. It aligns the
D405, waits for a stable complete set of RGB circles, then approaches every
detected centre in screen order (top to bottom, left to right). The holder
stays still. The target list is captured once before moving; later camera
frames do not change that list. `touch_screen()` is still a placeholder, so
the Android test will not complete from these approaches alone.
The camera aligns once before detection; set `align_before_each_target: true`
to realign between targets if needed.
`config/cellphone_collab.yaml` controls standoff planning time, planning
attempts, velocity scaling, and acceleration scaling. The launch loads these
values when the node starts; restart it after changing the YAML.

```bash
ros2 launch denso_collab cellphone_collab.launch.py \
  image_source:=topic image_topic:=/left_basic_camera \
  standoff_m:=0.1 plan_only:=true
```

Use Android Settings to vary fixed/random positions, fixed/random radii, and
the number of RGB trios (try 1–4). The robot discovers the count, positions,
sizes, and colors from the camera; it requires no matching trio count in ROS.

Use `plan_only:=true` first to check MoveIt plans without moving. The active
screen must fit all circles; if the app reports that targets cannot fit, lower
the count or radius in Settings. The detector saves a PNG and target JSON under
`/tmp/denso_target_diagnostics` when a stable layout is found; it also saves
diagnostic PNGs when detection fails. Tune `denso_vision/config/cellphone_holder.yaml`
for camera resolution, HSV thresholds, radius limits, stable frames, and
holder-plane ROI. Calibration offsets in `config/cellphone_collab.yaml` are in
metres and limited to ±20 mm.
