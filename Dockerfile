FROM ros:humble-ros-base

ENV HOSTNAME cloisim_ros
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp
ENV DEBIAN_FRONTEND noninteractive
ENV DEBCONF_NOWARNINGS yes

WORKDIR /opt/lge-ros2/src

RUN git clone --recursive https://github.com/lge-ros2/cloisim_ros.git -b ${ROS_DISTRO}

WORKDIR /opt/lge-ros2

RUN apt update && apt upgrade -y && \
    apt install -y -q ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rosdep update && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}

RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"]

COPY ./.entrypoint.sh /opt/

ENTRYPOINT ["/opt/.entrypoint.sh"]
