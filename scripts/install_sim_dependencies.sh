#!/bin/bash
# Exit immediately if a command exits with a non-zero status
set -e

PACKAGES=(
    "ros-jazzy-xacro"
    "ros-jazzy-joint-state-publisher"
    "ros-jazzy-joint-state-publisher-gui"
    "ros-jazzy-robot-state-publisher"
    "ros-jazzy-rviz2"
    "ros-jazzy-teleop-twist-keyboard"
    "ros-jazzy-navigation2"
    "ros-jazzy-nav2-bringup"
    "ros-jazzy-tf-transformations"
    "ros-jazzy-tf2-tools"
    "ros-jazzy-urdf"
)

echo "=== STEP 1: INSTALLING ROS2 SIMULATION DEPENDENCIES ==="
sudo apt-get update
for pkg in "${PACKAGES[@]}"; do
    echo "Installing $pkg..."
    sudo apt-get install -y "$pkg"
done

echo "=== STEP 2: VERIFYING PACKAGE INSTALLATIONS ==="
FAILED=0
for pkg in "${PACKAGES[@]}"; do
    if dpkg -l "$pkg" &> /dev/null; then
        echo "[VERIFIED] $pkg is installed."
    else
        echo "[ERROR] $pkg is NOT installed!"
        FAILED=1
    fi
done

if [ $FAILED -eq 0 ]; then
    echo "All dependencies installed successfully! ✅"
else
    echo "Some dependencies failed to install! ❌"
    exit 1
fi
