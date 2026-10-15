FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    vim \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /workspace
CMD ["bash"]