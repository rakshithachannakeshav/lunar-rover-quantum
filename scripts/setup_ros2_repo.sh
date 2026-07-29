#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

echo "=== STEP 1: CONFIGURING LOCALES ==="
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "Locales configured successfully."

echo "=== STEP 2: ADDING ROS2 REPOSITORY ==="
sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "=== STEP 3: UPDATING PACKAGE LISTS ==="
sudo apt-get update
echo "ROS2 Repository added and system updated."
