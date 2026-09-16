# DENSO cellphone collaboration

Move both robots to the saved collaboration start pose:

```bash
ros2 run denso_collab move_to_collab_start
```

Runs vision, then moves `left_calib_link` to red, green, and blue at a fixed
screen-normal standoff. Each pose uses the homography X/Y directly and keeps
the calibrated tool orientation. It returns to the joint state at launch after
blue. `touch_screen()` is deliberately only a placeholder between moves.

```bash
ros2 launch denso_collab cellphone_collab.launch.py \
  image_source:=topic image_topic:=/left_basic_camera standoff_m:=0.1
```
