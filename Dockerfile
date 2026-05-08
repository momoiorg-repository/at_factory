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

# 3. Add ROS 2 apt repository
RUN apt-get update && apt-get install -y curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
         -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
         http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
         | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && rm -rf /var/lib/apt/lists/*

# 4. Install ROS 2 Jazzy, dev tools, and upgrade system
ENV ROS_DISTRO=jazzy
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop \
    && rm -rf /var/lib/apt/lists/*

# 5. Add ROS 2 setup to bashrc (sources from mounted IsaacSim-ros_workspaces)
RUN echo "source /IsaacSim-ros_workspaces/build_ws/jazzy/jazzy_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /IsaacSim-ros_workspaces/build_ws/jazzy/isaac_sim_ros_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> ~/.bashrc

# 6. Isaac Sim ROS2 bridge — ships its own internal Jazzy libs.
#    LD_LIBRARY_PATH must include this path before Isaac Sim starts,
#    otherwise the isaacsim.ros2.bridge extension fails to load.
ENV LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:${LD_LIBRARY_PATH}
RUN echo "export LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib:\$LD_LIBRARY_PATH" >> ~/.bashrc

# Default command
CMD ["/bin/bash"]