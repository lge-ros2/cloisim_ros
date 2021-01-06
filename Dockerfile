FROM ros:foxy-ros-base

ENV HOSTNAME sim_device

WORKDIR /opt/lge-ros2/src

RUN git clone https://github.com/lge-ros2/sim_device.git -b ${ROS_DISTRO}
RUN git clone https://github.com/lge-ros2/cloi_common_interfaces.git -b ${ROS_DISTRO}

WORKDIR /opt/lge-ros2

RUN apt update && apt upgrade -y && rosdep update && \
	rosdep install -y -r -q --from-paths src --ignore-src --rosdistro $ROS_DISTRO

RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"]

COPY ./.entrypoint.sh /opt/

ENTRYPOINT ["/opt/.entrypoint.sh"]