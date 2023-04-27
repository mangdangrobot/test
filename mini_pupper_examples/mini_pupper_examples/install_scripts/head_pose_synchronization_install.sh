#!/bin/bash
#
# Script: head_pose_synchronization_install.sh
# Description: Installs dependencies for mini pupper OpenCV head pose synchronization demo.
# Author: Herman Ye (hermanye233@icloud.com)
# Copyright 2023 Mangdang
# License: Apache-2.0
# Version: 1.3
# Date: 2023-04-27

# Exit the script immediately if a command exits with a non-zero status
set -e

# Update the package list
sudo apt update

# Install depthai-ros
sudo apt install ros-humble-depthai-ros -y

# Set Udev rules
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Install depthai dependencies
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash

# Download depthai
cd ~
rm -rf ~/depthai
git clone https://github.com/luxonis/depthai.git

# Install depthai python dependencies
cd depthai
sudo apt install -y python3
sudo apt install -y python-is-python3
python3 install_requirements.py

# Set specific version of depthai-sdk for head_pose_synchronization demo
pip install depthai-sdk==1.9.1.1

# Check .bashrc environment
if grep -Fxq "source ~/ros2_ws/install/setup.bash" ~/.bashrc; then
    echo "ROS2 workspace environment setting found, skipping..."
else
    # Add the line to .bashrc if it doesn't exist
    echo "Adding ROS2 workspace environment setting to .bashrc..."
    echo "source ~/ros2_ws/install/setup.bash" >>~/.bashrc
fi

if grep -Fxq "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "ROS2 Humble environment setting found, skipping..."
else
    # Add the line to .bashrc if it doesn't exist
    echo "Adding ROS2 Humble environment setting to .bashrc..."
    echo "source /opt/ros/humble/setup.bash" >>~/.bashrc
fi

echo "Dependencies for OpenCV head pose synchronization demo have been installed successfully."
echo "Please unplug your camera USB3 cable and plug it back in."
