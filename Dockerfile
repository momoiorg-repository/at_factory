# Merged Dockerfile: Isaac Sim 5.0.0 + ROS 2 Humble workspace

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

# Install basic tools, ROS apt key, curl, and Python 3.11
RUN apt-get update \
    && apt-get install -qq -y --no-install-recommends \
       dirmngr gnupg2 curl software-properties-common \
    && add-apt-repository ppa:deadsnakes/ppa \
    && apt-get update \
    && apt-get install -qq -y --no-install-recommends \
       python3.11 python3.11-dev python3.11-venv python3.11-distutils \
       python3-pip \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1 \
    && update-alternatives --install /usr/bin/python python /usr/bin/python3.11 1 \
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

# Install official ROS 2 Humble packages (for packages that fail to build from source)
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
       ros-humble-desktop \
       ros-humble-navigation2 \
       ros-humble-nav2-bringup \
       ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

# Install build dependencies and tools for building ROS 2 from source with Python 3.11
RUN apt-get update \
    && apt-get install -y --allow-downgrades --no-install-recommends \
       # Build tools and dependencies
       build-essential git cmake python3-colcon-common-extensions python3-colcon-mixin python3-rosdep python3-vcstool \
       # Python 3.11 development packages
       python3.11-dev python3.11-venv python3.11-distutils \
       # Additional dependencies for ROS 2 build
       libssl-dev libasio-dev libtinyxml2-dev \
       libcunit1-dev libeigen3-dev libgtest-dev \
       # GUI and graphics dependencies
       libgl1-mesa-glx libgl1-mesa-dri mesa-utils \
       # Additional system dependencies
       curl wget gnupg2 lsb-release \
       # ROS 2 specific dependencies
       python3-pip python3-pytest-cov python3-rosinstall-generator ros-dev-tools \
       libbullet-dev libacl1-dev python3-empy libpython3-dev \
       # Boost libraries for OMPL
       libboost-all-dev libboost-dev libboost-filesystem-dev \
       libboost-program-options-dev libboost-system-dev libboost-thread-dev \
       libboost-serialization-dev libboost-date-time-dev libboost-regex-dev \
       libboost-python-dev libfmt-dev \
       # Geometric shapes dependencies
       libqhull-dev libassimp-dev liboctomap-dev libconsole-bridge-dev libfcl-dev \
       # X11 and graphics for RViz
       libx11-dev libxaw7-dev libxrandr-dev libgl1-mesa-dev libglu1-mesa-dev \
       libglew-dev libgles2-mesa-dev libopengl-dev libfreetype-dev libfreetype6-dev \
       libfontconfig1-dev libxcursor-dev libxinerama-dev libxi-dev libyaml-cpp-dev \
       libzzip-dev freeglut3-dev libogre-1.9-dev libpng-dev libjpeg-dev \
       python3-pyqt5.qtwebengine \
       # Qt5 dependencies
       qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools libqt5core5a \
       libqt5gui5 libqt5opengl5 libqt5widgets5 \
       # Additional dependencies that rosdep needs
       python3-flake8 python3-pytest-timeout uncrustify python3-lark \
       libbenchmark-dev pybind11-dev python3-psutil python3-numpy \
       python3-pycodestyle python3-netifaces libyaml-dev cppcheck \
       googletest libspdlog-dev libxml2-utils pydocstyle acl \
       # Additional packages for compatibility
       python3-pytest python3-pytest-cov python3-pytest-mock \
       python3-mock python3-coverage python3-cov-core \
    && rm -rf /var/lib/apt/lists/*


# Set default Python3 to Python3.11
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

# Install pip for Python 3.11
RUN curl -s https://bootstrap.pypa.io/get-pip.py -o get-pip.py \
    && python3.11 get-pip.py --force-reinstall \
    && rm get-pip.py

# Install Python packages needed for ROS 2 build
RUN python3.11 -m pip install setuptools==70.0.0 \
    && python3.11 -m pip uninstall -y em empy || true \
    && python3.11 -m pip install empy==3.3.4 \
    && python3.11 -m pip install -U argcomplete flake8-blind-except flake8-builtins \
       flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings \
       flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest lark \
    && python3.11 -m pip uninstall numpy -y \
    && python3.11 -m pip install "numpy<1.24" pybind11 PyYAML \
    && python3.11 -m pip install "pybind11[global]"

# Create symlinks for Python3.11 headers where CMake can find them
RUN ln -sf /usr/include/python3.11 /usr/include/python3

# Initialize rosdep
RUN rosdep init && rosdep update --rosdistro $ROS_DISTRO

# Build minimal ROS 2 Humble from source with Python 3.11 (Stage 1)
RUN apt-get update && mkdir -p /opt/ros2_humble_ws/src \
    && cd /opt/ros2_humble_ws \
    && rosinstall_generator --deps --rosdistro $ROS_DISTRO \
       rosidl_runtime_c rcutils rcl rmw tf2 tf2_msgs common_interfaces \
       geometry_msgs nav_msgs std_msgs rosgraph_msgs sensor_msgs vision_msgs \
       rclpy ros2topic ros2pkg ros2doctor ros2run ros2node ros_environment \
       ackermann_msgs example_interfaces --exclude rmw_connextdds_common \
       --exclude rmw_connextdds --exclude rmw_connextddsmicro > ros2.$ROS_DISTRO.rosinstall \
    && vcs import src < ros2.$ROS_DISTRO.rosinstall

# Patch packages to ensure they build with Python 3.11
RUN find /opt/ros2_humble_ws/src -name "CMakeLists.txt" -exec grep -l "PYTHON_INCLUDE_DIRS\|PYTHON_LIBRARY" {} \; | xargs -I{} /bin/bash -c 'if [ -f {} ]; then \
    echo "Patching {}"; \
    sed -i "s/include_directories(\${PYTHON_INCLUDE_DIRS})/include_directories(\/usr\/include\/python3.11)/" {}; \
    sed -i "s/\${PYTHON_LIBRARY}/python3.11/" {}; \
    sed -i "s/find_package(Python3 REQUIRED COMPONENTS Interpreter Development)/find_package(Python3 3.11 REQUIRED COMPONENTS Interpreter Development)/" {}; \
    fi'

# Patch Python Qt bindings for Python 3.11 compatibility
RUN find /opt/ros2_humble_ws/src -name "setup.py" -exec grep -l "python_qt_binding" {} \; | xargs -I{} /bin/bash -c 'if [ -f {} ]; then \
    echo "Patching Python Qt setup.py: {}"; \
    sed -i "s/python_requires.*/python_requires=\"\>=3.6\"/" {}; \
    fi'

# Set up environment variables for Python 3.11
ENV PYTHONPATH=/usr/local/lib/python3.11/dist-packages
ENV PYTHON_EXECUTABLE=/usr/bin/python3.11
ENV Python3_EXECUTABLE=/usr/bin/python3.11
ENV PYTHON_INCLUDE_DIR=/usr/include/python3.11
ENV PYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.11.so

# Build ROS 2 core with Python 3.11
RUN cd /opt/ros2_humble_ws \
    && rosdep install --from-paths src --ignore-src -r -y || true \
    && colcon build --cmake-args \
       "-DPython3_EXECUTABLE=/usr/bin/python3.11" \
       "-DPYTHON_EXECUTABLE=/usr/bin/python3.11" \
       "-DPYTHON_INCLUDE_DIR=/usr/include/python3.11" \
       "-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.11.so" \
       --merge-install

# Fix library compatibility issues
RUN cp /usr/lib/x86_64-linux-gnu/libtinyxml2.so* /opt/ros2_humble_ws/install/lib/ || true \
    && cp /usr/lib/x86_64-linux-gnu/libssl.so* /opt/ros2_humble_ws/install/lib/ || true \
    && cp /usr/lib/x86_64-linux-gnu/libcrypto.so* /opt/ros2_humble_ws/install/lib/ || true

# Copy the existing Isaac Sim ROS workspace (Stage 2)
COPY IsaacSim-ros_workspaces/humble_ws /opt/isaac_sim_ros_ws

# Remove problematic packages that require additional dependencies not in our minimal build
# RUN cd /opt/isaac_sim_ros_ws/src \
#     && rm -rf humanoid_locomotion_policy_example \
#     && rm -rf carter_navigation \
#     && rm -rf isaac_ros_navigation_goal

# Build the Isaac Sim ROS workspace with Python 3.11 (Stage 2)
RUN apt-get update && cd /opt/isaac_sim_ros_ws \
    && rosdep install --from-paths src --ignore-src -r -y || true \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && source /opt/ros2_humble_ws/install/setup.bash && colcon build --cmake-args \
       '-DPython3_EXECUTABLE=/usr/bin/python3.11' \
       '-DPYTHON_EXECUTABLE=/usr/bin/python3.11' \
       '-DPYTHON_INCLUDE_DIR=/usr/include/python3.11' \
       '-DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.11.so' \
       --merge-install"

# Add ROS 2 setup to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /opt/ros2_humble_ws/install/setup.bash" >> ~/.bashrc \
    && echo "source /opt/isaac_sim_ros_ws/install/setup.bash" >> ~/.bashrc

# Configure colcon mixins & metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
    && colcon mixin update \
    && colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml \
    && colcon metadata update

# Copy DDS configurations for better performance
COPY dds-config/cyclonedds.xml /cyclonedds.xml
COPY dds-config/fastdds.xml /fastdds.xml

# Install Isaac lab
RUN apt-get update \
    && git clone https://github.com/umegan/IsaacLab_lime.git /IsaacLab \
    && cd /IsaacLab \
    && ln -s /isaac-sim _isaac_sim \
    && TERM=xterm-256color ./isaaclab.sh --install

# Install PyTorch with CUDA support and Hugging Face libraries
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install --ignore-installed sympy>=1.13.3 \
    && python3 -m pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118 \
    && python3 -m pip install transformers datasets accelerate evaluate \
    && python3 -m pip install tokenizers safetensors huggingface_hub \
    && python3 -m pip install scikit-learn "numpy<1.24" pandas matplotlib seaborn \
    && python3 -m pip install jupyter ipykernel

# Environment setup for runtime with performance optimizations
RUN echo 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' >> ~/.bashrc \
    && echo 'export ROS_DOMAIN_ID=31' >> ~/.bashrc \
    && echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc \
    && echo 'source /opt/ros2_humble_ws/install/setup.bash' >> ~/.bashrc \
    && echo 'source /opt/isaac_sim_ros_ws/install/setup.bash' >> ~/.bashrc \
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