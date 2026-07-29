#!/bin/bash
set -e

echo "=== STEP 1: ADDING OSRF GAZEBO REPOSITORY ==="
sudo apt-get update
sudo apt-get install -y wget gnupg lsb-release

# Download OSRF GPG key
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs.osrfoundation.org-keyring.gpg

# Add OSRF repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs.osrfoundation.org-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo "=== STEP 2: INSTALLING GAZEBO HARMONIC ==="
sudo apt-get update
sudo apt-get install -y gz-harmonic

echo "=== STEP 3: INSTALLING ROS_GZ BRIDGE ==="
sudo apt-get install -y ros-jazzy-ros-gz

echo "=== STEP 4: VERIFYING GAZEBO INSTALLATION ==="
gz sim --version

echo "=== GAZEBO HARMONIC INSTALLATION COMPLETE ==="
