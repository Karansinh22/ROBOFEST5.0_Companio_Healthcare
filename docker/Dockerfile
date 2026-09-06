# Companio healthcare robot - ROS Noetic runtime image for Raspberry Pi 5 (arm64)
# Built and run by scripts/setup_container.sh

# Use ARM64 ROS Noetic base image for Raspberry Pi 5
# If this doesn't work, try: FROM ros:noetic
FROM arm64v8/ros:noetic-ros-core

# Set shell to bash
SHELL ["/bin/bash", "-c"]

# Set working directory
WORKDIR /root

# Update package lists and install basic tools
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    vim \
    nano \
    build-essential \
    cmake \
    python3-pip \
    python3-dev \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-libgpiod \
    libgpiod2 \
    python3-smbus \
    i2c-tools \
    liborocos-bfl-dev \
    flac \
    python3-opencv \
    libgtk2.0-dev \
    pkg-config \
    libxcb-util1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-render-util0 \
    libxcb-randr0 \
    libxcb-xinerama0 \
    libx11-xcb1 \
    libxkbcommon-x11-0 \
    libglu1-mesa-dev \
    freeglut3-dev \
    mesa-common-dev \
    libzbar0 \
    alsa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir minimalmodbus pyserial SpeechRecognition flask pyzbar pyyaml

# Install Raspberry Pi GPG key and repository for libcamera-apps (rpicam-vid)
RUN wget -qO - https://archive.raspberrypi.org/debian/raspberrypi.gpg.key | apt-key add - \
    && echo "deb http://archive.raspberrypi.org/debian/ bullseye main" > /etc/apt/sources.list.d/raspberrypi.list \
    && apt-get update && apt-get install -y \
    libcamera-apps-lite \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && apt-get install -y \
  ros-noetic-xacro \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-tf \
  ros-noetic-tf2 \
  ros-noetic-tf2-ros \
  ros-noetic-tf-conversions \
  ros-noetic-robot-state-publisher \
  ros-noetic-joint-state-publisher \
  ros-noetic-rviz \
  ros-noetic-rplidar-ros \
  ros-noetic-cv-bridge \
  ros-noetic-image-transport \
  ros-noetic-gazebo-dev \
  ros-noetic-gazebo-msgs \
  ros-noetic-gazebo-plugins \
  ros-noetic-gazebo-ros \
  ros-noetic-gazebo-ros-control \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-navigation \
  ros-noetic-move-base \
  ros-noetic-amcl \
  ros-noetic-dwa-local-planner \
  ros-noetic-global-planner \
  ros-noetic-rosbridge-server \
  ros-noetic-interactive-markers \
  ros-noetic-compressed-image-transport \
  ros-noetic-realsense2-camera \
  ros-noetic-realsense2-description \
  && rm -rf /var/lib/apt/lists/*

# Install Intel RealSense SDK (librealsense2) and modern Mesa drivers for Pi 5
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository ppa:kisak/turtle -y \
    && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u \
    && apt-get update && apt-get install -y \
    librealsense2-utils \
    librealsense2-dev \
    mesa-vdpau-drivers \
    mesa-va-drivers \
    libgles2-mesa-dev \
    libegl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*


# Install HailoRT SDK for AI HAT (safe extraction to avoid overwriting system binaries)
RUN apt-get update && apt-get install -y wget \
    && wget https://downloads.mixtile.com/doc-files/hailo/hailort_4.19.0_arm64.deb -O /tmp/hailort.deb \
    && mkdir -p /tmp/hailo_extract \
    && dpkg-deb -x /tmp/hailort.deb /tmp/hailo_extract \
    && if [ -d /tmp/hailo_extract/usr/lib ]; then cp -r /tmp/hailo_extract/usr/lib/* /usr/lib/ ; fi \
    && if [ -d /tmp/hailo_extract/usr/bin ]; then find /tmp/hailo_extract/usr/bin -name "hailo*" -exec cp {} /usr/bin/ \; ; fi \
    && rm -rf /tmp/hailo_extract /tmp/hailort.deb \
    && rm -rf /var/lib/apt/lists/*

# Install additional AI packages (Prioritize system OpenCV for GUI support)
RUN pip3 uninstall -y opencv-python opencv-python-headless opencv-contrib-python || true
RUN pip3 install --no-cache-dir numpy Pillow

# Fix RealSense library naming for ROS Noetic (built against v2.50)
RUN ln -s /usr/lib/aarch64-linux-gnu/librealsense2.so.2.56 /usr/lib/aarch64-linux-gnu/librealsense2.so.2.50


# Source ROS setup in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc \
    && echo "[ -f /root/companio_ws/devel/setup.bash ] && source /root/companio_ws/devel/setup.bash" >> /root/.bashrc \
    && echo "cd /root/companio_ws 2>/dev/null" >> /root/.bashrc

# Keep container running
CMD ["/bin/bash"]

