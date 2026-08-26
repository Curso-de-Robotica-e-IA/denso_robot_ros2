# Adding Custom Tools to DENSO Robot Descriptions

This document outlines the procedure for adding new tools (end-effectors) to the DENSO robot description package.

## Package Structure

The package is organized to support multiple robots and tools. New tools should be added to the `/tools` directory, following the existing structure.

```
├── CHANGELOG.rst
├── CMakeLists.txt
├── README.md
├── package.xml
├── robots
│   ├── cobotta
│   │   ├── meshes
│   │   └── urdf
│   ├── hsr065a1_n32
│   │   ├── meshes
│   │   └── urdf
│   ├── vs050
│   │   ├── meshes
│   │   └── urdf
│   └── vs060
│       ├── meshes
│       └── urdf
├── tools
│   ├── basic_camera
│   │   └── urdf
│   └── your_tool
│       ├── meshes
│       └── urdf
├── urdf
│   └── denso_robot.urdf.xacro
└── worlds
    └── empty_with_camera_support.sdf

```

### 1. Create a Tool Xacro Template

To create a new custom tool, first create a new directory for it under `/tools/` (e.g., `tools/your_tool/urdf/`). Inside that directory, create a `.xacro` file (e.g., `your_tool.xacro`) using the template below.

This file defines your tool's links, joints, and (if necessary) Gazebo sensors

```xml
<?xml version='1.0' encoding='UTF-8'?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <!-- here you define the parameters you want your xacro file to receive. -->
    <xacro:macro name="your_tool" params="namespace">

    <!-- if your tool has meshes, define the path to it in geometry tag. -->
    <link name="${namespace}your_tool_link">
        <!-- mass and inertia are mandatory tags, but they can have generic values -->
		<inertial>
			<mass value="1"/>
			<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
		</inertial>
		<collision>
			<geometry>
				<!-- <mesh filename="file:///$(find denso_robot_descriptions)/tools/your_tool/meshes/your_tool.dae" scale="1 1 1"/> -->
			</geometry>
		</collision>
		<visual>
			<geometry>
				<!-- <mesh filename="file:///$(find denso_robot_descriptions)/tools/your_tool/meshes/your_tool.dae" scale="1 1 1"/> -->
			</geometry>
		</visual>
	</link>

	<joint name="${namespace}your_tool_joint" type="fixed">
        <!-- here you define where your tool will connect. In this case, it's at joint 6. -->
		<parent link="${namespace}J6"/> 
		<child link="${namespace}your_tool_link"/>
        <!-- xyz="0 0 0" is not recommended, as it may cause collisions in the simulated environment.-->
		<origin rpy="0 0 0" xyz="0 0 0.01"/> 
	</joint>

    <!-- if you want to add the basic camera sensor to the end of your tool -->
    <!-- change the parent link of the basic camera joint to that of your tool -->


    <!--  the block below is only necessary if your tool needs to implement a sensor to be used in the Gazebo. -->
    <gazebo reference="${namespace}your_tool_link">
      <sensor name="your_tool" type="type_of_sensor">
        <!-- At https://gazebosim.org/docs/fortress/sensors/, you can find examples of some sensors. -->    
      </sensor>
    </gazebo>
    </xacro:macro>
</robot>
```

### 2. Integrate the Tool with the Robot

To attach your new tool, you must modify the `denso_robot_macro.xacro` file for the respective robot you are using. This file is located inside the robot's `urdf` folder (e.g., `robots/vs050/urdf/denso_robot_macro.xacro`).

Add the following lines to that file:


1. Add the `include` tag at the top of the file, with the other includes:
```xml
<xacro:include filename="$(find denso_robot_descriptions)/tools/your_tool/urdf/your_tool.xacro" />
```

2. Add the tool macro call inside the main robot macro, near the end-effector (J6) definition:
```xml
<xacro:your_tool namespace="${namespace}" />
```