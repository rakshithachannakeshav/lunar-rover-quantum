#!/bin/bash
set -e

echo "=== STEP 1: INSTALLING ROS2 JAZZY DESKTOP ==="
# Note: This is a large package, apt-get install may take several minutes.
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop

echo "=== STEP 2: INSTALLING COLCON BUILD TOOLS AND ROSDEP ==="
sudo apt-get install -y python3-colcon-common-extensions python3-rosdep

echo "=== STEP 3: INITIALIZING ROSDEP ==="
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
else
    echo "rosdep already initialized, skipping init."
fi

echo "=== STEP 4: UPDATING ROSDEP ==="
# Run rosdep update as the normal user (monis), not sudo/root
rosdep update

echo "=== STEP 5: CONFIGURING BASHRC SOURCE ==="
BASHRC="/home/monis/.bashrc"
SOURCE_LINE="source /opt/ros/jazzy/setup.bash"
if ! grep -Fxq "$SOURCE_LINE" "$BASHRC"; then
    echo "$SOURCE_LINE" >> "$BASHRC"
    echo "Added ROS2 sourcing to $BASHRC"
else
    echo "ROS2 sourcing already in $BASHRC"
fi

echo "=== ROS2 INSTALLATION COMPLETE ==="
