# Isaac@Factory Dockerfile: Isaac Sim 5.0.0 + ROS 2 Humble + Factory Environment

# Base image: Isaac Sim 5.0.0 official container
FROM nvcr.io/nvidia/isaac-sim:5.0.0

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# Non-interactive apt and locale
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble
ENV TERM=xterm-256color

# GUI Performance Environment Variables
ENV QT_X11_NO_MITSHM=1
ENV QT_GRAPHICSSYSTEM=native
ENV QT_QPA_PLATFORM=xcb
ENV QT_AUTO_SCREEN_SCALE_FACTOR=1
ENV QT_SCALE_FACTOR=1
ENV QT_FONT_DPI=96
ENV DISPLAY=:0
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# OpenGL and Graphics Performance
ENV LIBGL_ALWAYS_SOFTWARE=0
ENV MESA_GL_VERSION_OVERRIDE=4.5
ENV MESA_GLSL_VERSION_OVERRIDE=450
ENV __GL_SYNC_TO_VBLANK=0
ENV __GL_THREADED_OPTIMIZATIONS=1

# ROS 2 Performance Settings
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV ROS_DOMAIN_ID=31
ENV CYCLONEDDS_URI=file:///cyclonedds.xml

# Timezone setup
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get install -qq -y --no-install-recommends tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install basic tools, ROS apt key, and curl
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
       dirmngr gnupg2 curl \
    && rm -rf /var/lib/apt/lists/*

# Configure ROS 2 apt repository key
RUN set -eux; \
    key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
    export GNUPGHOME="$(mktemp -d)"; \
    gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
    mkdir -p /usr/share/keyrings; \
    gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
    gpgconf --kill all; \
    rm -rf "$GNUPGHOME"

# Add ROS 2 repository
RUN echo "deb [signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2-latest.list

# Fix libbrotli dependency conflict
RUN apt-get update \
    && apt-get install -y --allow-downgrades libbrotli1=1.0.9-2build6 \
    && apt-mark hold libbrotli1 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 core, Gazebo, GUI tools, RMW implementations, TurtleBot3 Lime deps, and build tools
RUN apt-get update \
    && apt-get install -y --allow-downgrades --no-install-recommends \
       # ROS 2 core & base
       ros-humble-ros-core \
       ros-humble-ros-base \
       # ROS 2 dev tools
       ros-dev-tools \
       # RMW implementations (FastRTPS for better GUI performance)
       ros-humble-rmw-fastrtps-cpp \
       ros-humble-rmw-cyclonedds-cpp \
       # GUI tools with performance packages
       ros-humble-rqt* ros-humble-rviz2 \
       # Additional GUI performance packages
       libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
       # Factory environment dependencies
       ros-humble-dynamixel-sdk \
       ros-humble-ros2-control \
       ros-humble-cartographer ros-humble-cartographer-ros \
       ros-humble-navigation2 ros-humble-nav2-bringup \
       ros-humble-ros2-controllers ros-humble-gripper-controllers \
       ros-humble-moveit ros-humble-moveit-servo \
       ros-humble-realsense2-camera-msgs ros-humble-realsense2-description \
       # Build & Python tools
       build-essential git python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Configure colcon mixins & metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

# Create CycloneDDS configuration for better performance
RUN echo '<?xml version="1.0" encoding="UTF-8" ?>' > /cyclonedds.xml \
    && echo '<CycloneDDS xmlns="https://cdds.io/config">' >> /cyclonedds.xml \
    && echo '  <Domain>' >> /cyclonedds.xml \
    && echo '    <General>' >> /cyclonedds.xml \
    && echo '      <AllowMulticast>true</AllowMulticast>' >> /cyclonedds.xml \
    && echo '      <MaxMessageSize>65500B</MaxMessageSize>' >> /cyclonedds.xml \
    && echo '    </General>' >> /cyclonedds.xml \
    && echo '    <Internal>' >> /cyclonedds.xml \
    && echo '      <Watermarks>' >> /cyclonedds.xml \
    && echo '        <WhcHigh>500kB</WhcHigh>' >> /cyclonedds.xml \
    && echo '      </Watermarks>' >> /cyclonedds.xml \
    && echo '    </Internal>' >> /cyclonedds.xml \
    && echo '    <Tracing>' >> /cyclonedds.xml \
    && echo '      <Verbosity>info</Verbosity>' >> /cyclonedds.xml \
    && echo '      <OutputFile>stdout</OutputFile>' >> /cyclonedds.xml \
    && echo '    </Tracing>' >> /cyclonedds.xml \
    && echo '  </Domain>' >> /cyclonedds.xml \
    && echo '</CycloneDDS>' >> /cyclonedds.xml

# Install Isaac lab for factory environment
RUN apt-get update \
    && git clone https://github.com/umegan/IsaacLab_lime.git /IsaacLab \
    && cd /IsaacLab \
    && ln -s /isaac-sim _isaac_sim \
    && TERM=xterm-256color ./isaaclab.sh --install

# Environment setup for runtime with performance optimizations
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=31' >> ~/.bashrc \
    && echo 'source /opt/ros/$ROS_DISTRO/setup.bash' >> ~/.bashrc \
    && echo 'export XDG_RUNTIME_DIR=/tmp/runtime-root' >> ~/.bashrc \
    && echo 'export QT_X11_NO_MITSHM=1' >> ~/.bashrc \
    && echo 'export QT_GRAPHICSSYSTEM=native' >> ~/.bashrc \
    && echo 'export QT_QPA_PLATFORM=xcb' >> ~/.bashrc \
    && echo 'export QT_AUTO_SCREEN_SCALE_FACTOR=1' >> ~/.bashrc \
    && echo 'export QT_SCALE_FACTOR=1' >> ~/.bashrc \
    && echo 'export QT_FONT_DPI=96' >> ~/.bashrc \
    && echo 'export LIBGL_ALWAYS_SOFTWARE=0' >> ~/.bashrc \
    && echo 'export MESA_GL_VERSION_OVERRIDE=4.5' >> ~/.bashrc \
    && echo 'export MESA_GLSL_VERSION_OVERRIDE=450' >> ~/.bashrc \
    && echo 'export __GL_SYNC_TO_VBLANK=0' >> ~/.bashrc \
    && echo 'export __GL_THREADED_OPTIMIZATIONS=1' >> ~/.bashrc \
    && echo 'export CYCLONEDDS_URI=file:///cyclonedds.xml' >> ~/.bashrc

# Create runtime directory
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
    
# Default command
CMD ["/bin/bash"]
