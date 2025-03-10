#!/bin/bash

set -e  # Exit on error

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Navigate to the base directory and create uros_ws
cd /shared-folder/ManchesterChallenge
mkdir uros_ws && cd uros_ws

# Clone micro-ROS setup
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# System updates and cleanup
sudo apt update -y
sudo apt upgrade -y
sudo apt full-upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y
sudo apt purge -y

# Install and update dependencies
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update && rosdep install --from-paths src --ignore-src -y

# Install Python pip
sudo apt install python3-pip -y

# Build the workspace
colcon build
source install/local_setup.sh

# Create and build the agent workspace
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

echo "micro-ROS setup completed successfully."
