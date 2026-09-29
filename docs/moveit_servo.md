# MoveIt Servo

**MoveIt Servo** allows for real-time control of the robotic arm by sending continuous velocity commands. Unlike point-to-point trajectory planning, Servo is ideal for teleoperation, visual servoing, and motions where reactivity is required.

Commands can be sent in Joint Space or Cartesian Space (Twist and Pose).

Useful references:

- [MoveIt Servo Jazzy Docs](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)
- [All parameters of MoveIt Servo Jazzy](https://github.com/moveit/moveit2/blob/jazzy/moveit_ros/moveit_servo/config/servo_parameters.yaml)


> **NOTE**: Some applications need lower jitter. For this, you will need a real-time system. For a complete guide on setting up real-time Linux with NVIDIA drivers and Docker containers, see: [Linux Real-Time Guide by Tobit Flatscher](https://github.com/2b-t/linux-realtime#linux-and-docker-real-time-guide-for-ubuntu-realtime-kernel-and-preempt_rt)

## Safety Warning: Collision Thresholds

> **NOTE**: **High Speed Risk** — moving at high velocities may increase the risk of collisions due to latency or stopping distances. **Before using MoveIt Servo**, you must evaluate and tune the collision proximity thresholds to ensure safety.

To modify these values, edit the following file: [`denso_robot_moveit_config/config/moveit_servo.yaml`](../denso_robot_moveit_config/config/moveit_servo.yaml) or by using the `ros2 param` command

Adjust the following parameters:

* `self_collision_proximity_threshold`: distance to trigger a stop when near self-collision.
* `scene_collision_proximity_threshold`: distance to trigger a stop when near environment objects.

## Signal Smoothing

MoveIt Servo supports command smoothing via `smoothing_filter_plugin_name` in [`denso_robot_moveit_config/config/moveit_servo.yaml`](../denso_robot_moveit_config/config/moveit_servo.yaml)
 
[Signal Smoothing](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html#signal-smoothing) status:

* **Butterworth Filter:** Supported.
* **Ruckig Filter:** Not supported yet.
* **Acceleration Limited:** Supported, but **requires** passing `update_period` and `planning_group_name` parameters in your launch file:

```python
# Example launch configuration snippet
acceleration_filter_update_period = {'update_period': 0.01}
planning_group_name = {'planning_group_name': 'arm'}

...

servo_node = Node(
    package='moveit_servo',
    executable='servo_node',
    ...
    parameters=[
        ...
        acceleration_filter_update_period,
        planning_group_name,
    ],
    ...
)
```

## Usage

**NOTE**: for dual-arm setups, prefix the service name with `left_` or `right_` (e.g., `/left_servo_node/start_servo`).

### 1. Service Commands (Start / Pause)

The servo node must be activated or managed via ROS 2 service calls. Use the following commands to handle the servo state.

* **Select Command Type:** activates the servo control for a specific command type.
```bash
ros2 service call /servo_node/switch_command_type moveit_msgs/srv/ServoCommandType "{command_type: <value>}"
```

where `<value>` corresponds to:
* 0 = JOINT_JOG
* 1 = TWIST
* 2 = POSE

* **Pause Servo:** pauses motion while keeping the servo active.
```bash
ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: true}"
```

* **Unpause Servo:** resumes motion after a pause.
```bash
ros2 service call /servo_node/pause_servo std_srvs/srv/SetBool "{data: false}"
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
ros2 topic pub -r <rate in Hz> /servo_node/delta_joint_cmds control_msgs/msg/JointJog "{
  header: {frame_id: '<frame_id>', stamp: 'now'},
  joint_names: ['joint_1', 'joint_2', ...],
  velocities: [<velocity_joint_1>, <velocity_joint_2>, ...]
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
ros2 topic pub -r <rate in Hz> /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{
  header: {frame_id: '<frame_id>', stamp: 'now'},
  twist: {
    linear: {x: <velocity_x>, y: <velocity_y>, z: <velocity_z>},
    angular: {x: <velocity_roll>, y: <velocity_pitch>, z: <velocity_yaw>}
  }
}"
```

#### 2.3 Pose (Cartesian Goal Control)

Sends a target position and orientation for the End Effector. Unlike Twist commands which send direct velocities, Pose commands send spatial targets. MoveIt Servo uses Inverse Kinematics (IK) to dynamically compute the continuous joint velocities required to smoothly track and reach this target pose in real-time.

* **Position Unit**: meters (m)
* **Orientation Unit**: Quaternions (x, y, z, w)

The `frame_id` defines the reference frame the target pose is expressed in:

* `base_link` — the target pose is relative to the robot's base (fixed frame).
* `tool0` — the target pose is relative to the current end-effector frame (see Frame Conventions). Useful for incremental motions relative to the tool's own position and orientation.

```bash
ros2 topic pub -r <rate in Hz> /servo_node/pose_target_cmds geometry_msgs/msg/PoseStamped "{
  header: {frame_id: '<frame_id>', stamp: 'now'},
  pose: {
    position: {x: <x>, y: <y>, z: <z>},
    orientation: {x: <qx>, y: <qy>, z: <qz>, w: <qw>}
  }
}"
```

## Related Documentation

- [Frame Conventions](frame_conventions.md)