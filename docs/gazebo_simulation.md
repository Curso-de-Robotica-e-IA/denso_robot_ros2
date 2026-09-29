# Gazebo Simulation

This document describes how to customize the simulated environment used by the DENSO robot stack in Gazebo — adding objects to the world and configuring sensors.

World files are located in `denso_robot_gazebo/worlds/`.

## Adding Objects to the World

There are two ways to add an object (model) to a world:

### 1. Insert a Model from Gazebo Fuel (GUI)

[Gazebo Fuel](https://app.gazebosim.org/fuel/models) hosts a large collection of ready-to-use models.

1. Browse or search for a model on [Fuel](https://app.gazebosim.org/fuel/models).
2. Copy the SDF snippet from the model's page (`<>` button).
3. Paste it into your world's `.sdf` file, inside the `<world>` tag.

See the official [Model Insertion from Fuel](https://gazebosim.org/docs/harmonic/fuel_insert/) tutorial for details.

### 2. Download and Add a Model Permanently

To keep a local, editable copy of a model instead of only referencing it remotely:

1. On the model's Fuel page, click the download icon to get the model files.
2. Add the downloaded model to `denso_robot_gazebo/worlds/`.
3. Reference it in your world's `.sdf` file. Once local, the model files can be freely edited.

### Repositioning an Object

Use the `<pose>` tag to set an object's position and orientation in the world:

```xml
<pose>x y z roll pitch yaw</pose>
```

- `x y z` — position, in meters
- `roll pitch yaw` — orientation, in radians

Example:
```xml
<include>
  <uri>model://your_model</uri>
  <pose>1.0 0.5 0.0 0 0 1.57</pose>
</include>
```

## Sensors

Sensors are defined using the `<sensor>` tag inside a `<gazebo>` block, referencing the link they should be attached to. The full list of available sensor types and their parameters is defined by the [SDFormat sensor specification](https://sdformat.org/spec/1.11/sensor/).

- `<sensor type="...">` — sensor type (e.g. `camera`, `depth_camera`, `gpu_lidar`, `imu`). See the [full list of sensor types](https://sdformat.org/spec/1.11/sensor/) for parameters specific to each type.
- `reference="..."` — the link the sensor is rigidly attached to.
- `<topic>` — the ROS 2 topic name the sensor data will be published to.
- `<update_rate>` — sensor update rate, in Hz.

For a practical example, see `denso_robot_gazebo/sensors/gz_cam/urdf/gz_cam.xacro`, which defines a camera sensor attached to the `flange` link.

> **NOTE**: since `gz_cam` is attached via the robot's `flange` link, it follows the same namespacing (`left_`/`right_`) described in [Adding Custom Tools](adding_custom_tools.md#2-include-the-tool-in-the-robots-urdf) for dual-arm setups. As with tools, the `flange` frame is currently only available on the **VS-050** robot.

### Attaching a Sensor to a New Tool or Robot Link

To add a sensor to a custom tool or another robot link, follow the same [Adding Custom Tools](adding_custom_tools.md) process, adding a `<gazebo><sensor>` block referencing your tool's link — as shown in the [Xacro File Template](adding_custom_tools.md#xacro-file-template).

## Related Documentation

- [Frame Conventions](frame_conventions.md)