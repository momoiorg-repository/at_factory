# Base image: Isaac-Lab 2.3.0 (based on Ubuntu 24.04 "Noble")
FROM nvcr.io/nvidia/isaac-lab:2.3.0

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Set non-interactive frontend
ENV DEBIAN_FRONTEND=noninteractive

# 1. Set locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# 2. Enable required repositories (Universe)
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# 3. Install ROS 2 apt source (the new method from your docs)
RUN apt-get update && apt-get install -y curl \
    && export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

# 4. Install ROS 2 Jazzy, dev tools, and upgrade system
ENV ROS_DISTRO=jazzy
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# 5. Add ROS 2 setup to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> ~/.bash

# Default command
CMD ["/bin/bash"]