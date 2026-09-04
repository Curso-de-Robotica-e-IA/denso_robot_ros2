# MoveIt Servo

**MoveIt Servo** allows for real-time control of the robotic arm by sending continuous velocity commands. Unlike point-to-point trajectory planning, Servo is ideal for teleoperation, visual servoing, and motions where reactivity is required.

Commands can be sent in Joint Space or Cartesian Space (Twist).

Useful references:

- [MoveIt Servo Humble Docs](https://moveit.picknik.ai/humble/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [MoveIt Servo Humble GitHub](https://github.com/moveit/moveit2/tree/humble/moveit_ros/moveit_servo)

> **NOTE**: MoveIt Servo does not behave like a typical ROS2 node when it comes to configuration — `ros2 param set` does **not** work with the `moveit_servo` node. Parameters must be changed directly in `moveit_servo.yaml`, followed by relaunching the node. See [Configuration Parameters](#configuration-parameters-moveit_servoyaml) below for more observations on how these parameters behave in practice.

## Safety Warning: Collision Thresholds

> **NOTE**: **High Speed Risk** — moving at high velocities may increase the risk of collisions due to latency or stopping distances. **Before using MoveIt Servo**, you must evaluate and tune the collision proximity thresholds to ensure safety.

To modify these values, edit the following file:
`denso_robot_moveit_config/config/moveit_servo.yaml`

Adjust the following parameters:

* `self_collision_proximity_threshold`: distance to trigger a stop when near self-collision.
* `scene_collision_proximity_threshold`: distance to trigger a stop when near environment objects.

---

## Usage

**NOTE**: for dual-arm setups, prefix the service name with `left_` or `right_` (e.g., `/left_servo_node/start_servo`).

### 1. Service Commands (Start / Pause / Stop)

The servo node must be activated or managed via ROS2 service calls. Use the following commands to handle the servo state.

* **Start Servo:** activates the servo control.
```bash
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger {}
```

* **Pause Servo:** pauses motion while keeping the servo active.
```bash
ros2 service call /servo_node/pause_servo std_srvs/srv/Trigger {}
```

* **Unpause Servo:** resumes motion after a pause.
```bash
ros2 service call /servo_node/unpause_servo std_srvs/srv/Trigger {}
```

* **Stop Servo:** completely stops the servo control.
```bash
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger {}
```

### 2. Motion Commands

To move the robot, publish messages to the appropriate topics.

**NOTE**: for dual-arm setups, you must prefix the **topic** name, **frame_id**, and **joint_names**:

* **Topic:** `/left_servo_node/delta_joint_cmds`
* **Frame:** `left_base_link`
* **Joints:** `left_joint_1`

#### 2.1 Joint Jog (Joint Space Control)

Sends angular velocities to specific joints.

* **Unit**: radians per second (rad/s)
```bash
ros2 topic pub /servo_node/delta_joint_cmds control_msgs/msg/JointJog "{
  header: {frame_id: 'base_link', stamp: 'now'},
  joint_names: ['joint_1', 'joint_2', ...],
  velocities: [<velocity_joint_1>, <velocity_joint_1>, ...]
}"
```

#### 2.2 Twist Stamped (Cartesian Control)

Sends linear and angular velocities to the End Effector.

* **Linear Unit**: meters per second (m/s)
* **Angular Unit**: radians per second (rad/s)

The `frame_id` defines the reference frame the velocity is expressed in:

* `base_link` — motion is expressed relative to the robot's base (fixed frame).
* `tool0` — motion is expressed relative to the end-effector frame (see [Frame Conventions](frame_conventions.md)). Useful for motions relative to the tool's own orientation, e.g. moving "forward" from the tool's point of view.

```bash
ros2 topic pub /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{
  header: {frame_id: 'base_link', stamp: 'now'},
  twist: {
    linear: {x: <velocity_x>, y: <velocity_y>, z: <velocity_z>},
    angular: {x: <velocity_roll>, y: <velocity_pitch>, z: <velocity_yaw>}
  }
}"
```

---

## Configuration Parameters (`moveit_servo.yaml`)

> **NOTE**: MoveIt Servo's official documentation does not cover these parameters in detail. The notes below are based on hands-on testing and observed behavior, not official documentation — if you find something inaccurate, please update this section.

### `publish_period`

Acts as a multiplier on the final velocity of the robot: the smaller the period (i.e., the higher the frequency), the smaller the resulting velocity. For example, a `publish_period` of `0.01` (100 Hz) makes the final velocity **100x smaller**.

### `low_latency_mode`

Makes the servo work event-driven instead of cycle-driven.

- When `low_latency_mode: false`, the servo runs on fixed cycles and the rate at which you publish to the topic (the `-r` flag) does **not** affect the final velocity.
- When `low_latency_mode: true`, the servo becomes event-driven, and in this mode the publishing rate **does** act as a velocity multiplier, as described in `publish_period` above — publishing at a higher rate (e.g. `-r 100`) decreases the resulting velocity.

### `lower_singularity_threshold` / `hard_stop_singularity_threshold`

These are threshold values based on the **condition number of the Jacobian matrix**.

### `robot_link_command_frame`

Must be set to a **valid frame** (e.g., `base_link` or `tool0`). However, this does **not** restrict the `frame_id` you can send in a `TwistStamped` command — you can send a twist with a different `frame_id` than the one set in `robot_link_command_frame` without issues.

Recommended command frames: `base_link` or `tool0`. See [Frame Conventions](frame_conventions.md) for details on frame orientation.

### `open_loop_control` (ros2_control parameter)

Should be set to `true`. MoveIt Servo only behaves well with `open_loop_control: true` — with `open_loop_control: false`, the robot's motion is not smooth.

## Related Documentation

- [Frame Conventions](frame_conventions.md)