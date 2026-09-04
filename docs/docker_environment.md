# Docker Environment

## Overview

This Docker environment allows you to run the DENSO robot ROS 2 stack with ROS 2 Humble, without needing to install ROS 2 and its dependencies directly on the host machine. It includes support for RViz2 and Gazebo Fortress.

## Requirements

- [Docker Engine and Docker Compose](https://docs.docker.com/engine/install/)
- For NVIDIA GPU support: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) and an NVIDIA GPU with up-to-date drivers

## Building and Starting the Container

```bash
cd .docker/
```

The `--build` flag builds the image before starting the container. Choose the command according to your hardware:

**CPU only:**
```bash
ROS_DOMAIN_ID=<your_id> docker compose run --name denso_ros_humble_cpu --build cpu
```

**With NVIDIA GPU:**
```bash
ROS_DOMAIN_ID=<your_id> docker compose run --name denso_ros_humble_gpu --build gpu
```

> **NOTE**: `ROS_DOMAIN_ID` isolates ROS 2 communication over the network using DDS. To avoid interference between different computers running ROS 2 on the same network, a different domain ID should be set for each computer. On Linux, safe values are **0–101** and **215–232**. For more details, see the [ROS 2 documentation](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html).

## Subsequent Executions

Once the container has been created, use the following commands to restart it without rebuilding:

**CPU:**
```bash
docker start -ai denso_ros_humble_cpu
```

**NVIDIA GPU:**
```bash
docker start -ai denso_ros_humble_gpu
```

To exit the container, run:
```bash
exit
```

### Available Tools

* **Terminator**: terminal with tab and split-screen support.
```bash
terminator
```