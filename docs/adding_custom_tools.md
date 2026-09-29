# Custom Tools for DENSO Robots

This document explains how to add custom tools (end-effectors) to a DENSO robot, using packages from the [`denso_tools`](https://github.com/Curso-de-Robotica-e-IA/denso_tools) repository.

> **NOTE**: for now, this documentation only works with the **VS-050** robot.

## Adding an Existing Tool to Your Environment

### 1. Copy the Tool Package

Copy the directory of the desired tool from the [`denso_tools`](https://github.com/Curso-de-Robotica-e-IA/denso_tools) repository into the `src/` folder of your `denso_robot_ros2` workspace.

### 2. Include the Tool in the Robot's URDF

Open the file `denso_robot_descriptions/robots/vs050/urdf/denso_robot_macro.xacro` and:

**a)** Add the `include` tag at the top of the file, alongside the other imports:
```xml
<xacro:include filename="$(find your_tool_package)/urdf/your_tool.xacro" />
```

**b)** Add the macro call inside the robot's main macro:
```xml
<xacro:your_tool namespace="${namespace}" />
```

> **NOTE on `namespace`**: when using two robotic arms, each one has a different namespace. This allows different tools to be attached to each arm independently. Use `left_` or `right_` as the namespace, depending on which arm the tool should be attached to. (This only needs to be done in `denso_robot_descriptions/robots/vs050/urdf/denso_robot_macro.xacro`)

### 3. Update the MoveIt2 SRDF

Open the file `denso_robot_moveit_config/robots/vs050/srdf/denso_robot_macro.srdf.xacro`.

Find the `<group>` definition containing a `<chain>` and replace the `tip_link` with your tool's link:

```xml
<chain base_link="..." tip_link="${namespace}your_tool_link" />
```

### 4. Build the Workspace

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Creating a New Tool

### Package Structure

Create a new ROS2 package with the following structure:

```
your_tool/
├── meshes/
└── urdf/
    └── tour_tool.xacro
```
The `meshes/` folder is optional — only needed if the tool has custom 3D geometry.

### Xacro File Template

Copy the template below and replace `your_tool` with your tool's name:

> **NOTE**: when exporting a tool mesh, orient it with **+X forward** and **+Z upward**. See [Frame Conventions](frame_conventions.md) for details.

```xml
<?xml version='1.0' encoding='UTF-8'?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- Define here the parameters your xacro file will receive. -->
    <xacro:macro name="your_tool" params="namespace">

        <!-- If your tool has a 3D mesh, set its path in the geometry tag. -->
        <link name="${namespace}your_tool_link">
            <!-- mass and inertia are mandatory, but can have generic values. For more informations: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/Adding-Physical-and-Collision-Properties-to-a-URDF-Model.html#id2 -->
            <inertial>
                <mass value="1"/>
                <inertia ixx="1e-3" ixy="0" ixz="0" iyy="1e-3" iyz="0" izz="1e-3"/>
            </inertial>
            <collision>
                <geometry>
                    <!-- <mesh filename="file:///$(find your_tool)/meshes/your_tool.dae" scale="1 1 1"/> -->
                </geometry>
            </collision>
            <visual>
                <geometry>
                    <!-- <mesh filename="file:///$(find your_tool)/meshes/your_tool.dae" scale="1 1 1"/> -->
                </geometry>
            </visual>
        </link>

        <joint name="${namespace}your_tool_joint" type="fixed">
            <!-- The tool must always be attached to the robot's flange link. -->
            <parent link="${namespace}flange"/>
            <child link="${namespace}your_tool_link"/>
            <!-- Avoid xyz="0 0 0", as it may cause collisions in the simulated environment. -->
            <!-- Unfortunately, rpy depends on how your mesh was exported, so you'll need to test different values to find the correct one. -->
            <origin rpy="0 0 0" xyz="0 0 0.01"/>
        </joint>

        <!-- The block below is only needed if your tool requires a Gazebo sensor. -->
        <!-- Sensor examples at: https://gazebosim.org/docs/harmonic/sensors/ -->
		<!-- For details of sensor tag: https://sdformat.org/spec/1.11/sensor/ -->
        <gazebo reference="${namespace}your_tool_link">
            <sensor name="your_tool" type="sensor_type">
            </sensor>
        </gazebo>

    </xacro:macro>
</robot>
```

After creating the package, follow the steps in the previous section to integrate it with the robot.

### CMakeLists.txt Template

```cmake
cmake_minimum_required(VERSION 3.8)
project(your_tool)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>your_tool</name>
  <version>0.0.1</version>
  <description>Description package for your tool.</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>denso_robot_descriptions</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```