FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV MAKEFLAGS="-j1"

# Update package list and install tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    curl \
    wget \
    zsh \
    neovim \
    nano \
    libeigen3-dev \
    ros-humble-tf-transformations \
    ros-humble-cv-bridge \
    ros-humble-foxglove-bridge \
    tmux \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ros-gz* \
    ros-humble-*-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-localization \
    ros-humble-joy-teleop \
    && rm -rf /var/lib/apt/lists/*

# Install specific Python packages
RUN pip install --no-cache-dir \
    "numpy<1.25.0" \
    scipy \
    pyyaml \
    opencv-python \
    opencv-contrib-python-headless==4.6.0.66 \
    transforms3d

# Set default working directory
WORKDIR /root

RUN echo "ALL DONE"