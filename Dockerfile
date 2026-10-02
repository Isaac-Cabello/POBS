FROM osrf/ros:lyrical-desktop

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=lyrical

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    nano \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    ros-lyrical-turtlesim \
    ros-lyrical-rqt-common-plugins \
    ros-lyrical-ros-gz \
    ros-lyrical-ros-gz-sim \
    ros-lyrical-simulation-interfaces \
    ros-lyrical-unique-identifier-msgs \
    ros-lyrical-rosidl-typesupport-fastrtps-c \
    ros-lyrical-rosidl-typesupport-fastrtps-cpp \
    sudo \
    gosu \
    && apt-get dist-upgrade -y \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN useradd --create-home --shell /bin/bash ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ros \
    && chmod 0440 /etc/sudoers.d/ros

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ros/.bashrc \
    && echo '[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash' \
        >> /home/ros/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /home/ros/ros2_ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]