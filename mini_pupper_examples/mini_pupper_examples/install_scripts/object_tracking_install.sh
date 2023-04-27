#!/bin/bash
#
# Script: object_tracking_install.sh
# Description: Installs dependencies for the Mini Pupper OpenCV object tracking demo.
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

# Install vision-msgs package
sudo apt install ros-humble-vision-msgs -y

# Install depthai dependencies
sudo curl -fL https://docs.luxonis.com/install_dependencies.sh | bash

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

# Installation complete
echo "Installation of Mini Pupper OpenCV object tracking demo dependencies complete."
echo "Please unplug your camera USB3 cable and plug it back in."
