FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive

ENV COLCON_WS=/root/denso_ws

RUN mkdir -p $COLCON_WS
WORKDIR $COLCON_WS

COPY . src/

RUN apt-get update -q && apt-get install -q -y --no-install-recommends \
    ros-$ROS_DISTRO-moveit \
    ros-$ROS_DISTRO-moveit-servo \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    terminator \
  && echo 'APT::Install-Recommends "false";' > /etc/apt/apt.conf.d/00norecommends \
  && rosdep install --ignore-src --from-paths src -y -r --rosdistro $ROS_DISTRO \
  && rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
  && echo "source /root/denso_ws/install/setup.bash" >> /root/.bashrc

RUN rm -rf src/*

CMD ["bash"]