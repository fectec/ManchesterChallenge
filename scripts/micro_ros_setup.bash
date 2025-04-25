#!/bin/bash

set -e

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Create and navigate to the workspace
mkdir -p ~/uros_ws/src && cd ~/uros_ws/src

# Clone micro-ROS setup repository
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# System update and cleanup
sudo apt update -y
sudo apt upgrade -y
sudo apt full-upgrade -y
sudo apt autoremove -y
sudo apt autoclean -y

# Install and update dependencies
sudo apt install python3-pip -y
sudo rosdep init || true
rosdep update
rosdep install --from-paths src --ignore-src -y

# Build the workspace
cd ~/uros_ws
colcon build
source install/local_setup.sh

# Create and build micro-ROS agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

# Add workspace to bashrc
echo "source ~/uros_ws/install/local_setup.bash" >> ~/.bashrc

# Grant permissions for serial ports
sudo chmod a+rw /dev/tty*
sudo usermod -a -G dialout $USER

echo "Setup complete. Please reboot or log out and back in for user group changes to take effect."
