# Refactored calibration → MoveIt JSON pipeline

This refactor keeps the same calibration workflow but simplifies the JSON formats and the MoveIt sender. The output JSON now uses a single `points` array that contains both the original Robot1 point and its transformed Robot2 point.

## Stage 1: Collect raw points (Robot1 + Robot2)

Collect matching physical points for both robots (same as before):

```bash
ros2 run denso_robot_bringup save_calib_tool_tf.py --output calib_tool_tf-P0.json
ros2 run denso_robot_bringup save_calib_tool_tf.py --output calib_tool_tf-P0-R2.json
```

Repeat for P1..PN.

## Stage 2: Compute Robot1 → Robot2 transform

```bash
ros2 run denso_robot_bringup compute_robot1_to_robot2_tf.py \
  --robot1-dir <robot1_points_dir> \
  --robot2-dir <robot2_points_dir> \
  --output calib_tool_tf-robot1_to_robot2.json
```

## Stage 3: Transform Robot1 motion points into Robot2 frame

### Input JSON (Robot1 motion path)

Use a simple `points` array. The script accepts either `position_robot1` or `position`:

```json
{
  "points": [
    { "label": "P0", "position_robot1": { "x": 0.10, "y": -0.05, "z": 0.20 } },
    { "label": "P1", "position_robot1": { "x": 0.12, "y": -0.03, "z": 0.22 } }
  ]
}
```

### Transform into Robot2 frame

```bash
ros2 run denso_robot_bringup transform_robot1_points_to_robot2.py \
  --transform-json calib_tool_tf-robot1_to_robot2.json \
  --input-json robot1_motion_points.json \
  --output robot1_points_in_robot2_frame.json
```

### Output JSON (Robot1 + Robot2 points)

```json
{
  "transform_json": "calib_tool_tf-robot1_to_robot2.json",
  "num_points": 2,
  "include_rotation": false,
  "points": [
    {
      "label": "P0",
      "position_robot1": { "x": 0.10, "y": -0.05, "z": 0.20 },
      "position_robot2_transformed": { "x": 0.31, "y": 0.12, "z": 0.18 }
    }
  ]
}
```

If you pass `--include-rotation`, each point will also include
`orientation_robot2_transformed_quaternion`.

## Stage 4: Run MoveIt using the new JSON

Use the new node `send_moveit_points_from_json`, which reads the `points` array and sends
`position_robot2_transformed` to MoveIt. If you set `use_orientation:=true`, it will also
read `orientation_robot2_transformed_quaternion` from each point.

```bash
ros2 run denso_robot_moveit_demo send_moveit_points_from_json --ros-args \
  -p input_json:=robot1_points_in_robot2_frame.json \
  -p group_name:=arm \
  -p velocity_scaling:=0.6 \
  -p rotation_x_deg:=180.0
```

**Note:** ROS parameter names use underscores, not dashes (e.g. `group_name`, `velocity_scaling`).

### Fixed orientation debugging

Set `rotation_x_deg`, `rotation_y_deg`, and `rotation_z_deg` to apply a fixed roll/pitch/yaw
orientation (degrees) to every target. When all three are zero, the node sends position-only
targets without constraining orientation.

If `use_orientation:=true` and a fixed rotation is also set, the fixed rotation is applied
on top of the JSON orientation (quaternion multiplied as `fixed * json`).
