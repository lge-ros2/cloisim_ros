FROM ubuntu:18.04

ENV HOSTNAME sim-device
ENV ROS_DISTRO=dashing

RUN apt update && apt upgrade -q -y && \
	apt install -q -y locales curl gnupg2 lsb-release && \
	apt autoclean && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US en_US.UTF-8 && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	export LANG=en_US.UTF-8

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

RUN export DEBIAN_FRONTEND=noninteractive && \
	apt update && \
	apt install -y ros-${ROS_DISTRO}-ros-base git python3-colcon-common-extensions && \
	apt install -y python3-websocket libzmq3-dev libprotobuf-dev protobuf-compiler ros-${ROS_DISTRO}-camera-info-manager && \
	apt autoclean && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /opt/lge-ros2/src

WORKDIR /opt/lge-ros2/src

RUN git clone https://github.com/lge-ros2/sim-device.git -b ${ROS_DISTRO}

WORKDIR /opt/lge-ros2

RUN ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install"]

COPY ./.entrypoint.sh /opt/

ENTRYPOINT ["/opt/.entrypoint.sh"]