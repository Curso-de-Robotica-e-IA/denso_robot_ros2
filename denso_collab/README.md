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
