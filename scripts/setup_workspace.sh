#!/bin/bash
set -e

echo "=== PHASE F: ROS2 WORKSPACE SETUP ==="

# Step 1: Create workspace structure
echo "--- Step 1: Creating workspace directory structure ---"
WS_ROOT=~/lunar-rover-quantum
mkdir -p $WS_ROOT/src/rover_simulation/urdf
mkdir -p $WS_ROOT/src/rover_simulation/worlds
mkdir -p $WS_ROOT/src/rover_simulation/config
mkdir -p $WS_ROOT/src/rover_simulation/launch
mkdir -p $WS_ROOT/src/rover_simulation/rover_simulation

echo "Directory structure created ✅"

# Step 2: Copy starter files from Windows mount
echo "--- Step 2: Copying starter simulation files ---"
SRC=/mnt/c/Users/monis/Downloads/lunar-rover-quantum/sim_files
DEST=$WS_ROOT/src/rover_simulation

cp $SRC/rover.urdf.xacro         $DEST/urdf/
cp $SRC/lunar_terrain.world       $DEST/worlds/
cp $SRC/gazebo_params.yaml        $DEST/config/
cp $SRC/rviz_config.rviz          $DEST/config/
cp $SRC/simulation_launch.py      $DEST/launch/
cp $SRC/package.xml               $DEST/
cp $SRC/setup.py                  $DEST/
cp $SRC/__init__.py               $DEST/rover_simulation/
echo "Starter files copied ✅"

# Step 3: Source ROS2
source /opt/ros/jazzy/setup.bash

# Step 4: Build workspace with colcon
echo "--- Step 3: Building workspace with colcon ---"
cd $WS_ROOT
colcon build --symlink-install
echo "Workspace built ✅"

# Step 5: Add workspace sourcing to .bashrc
echo "--- Step 4: Adding workspace sourcing to ~/.bashrc ---"
BASHRC="$HOME/.bashrc"
WS_SOURCE="source $WS_ROOT/install/setup.bash"
if ! grep -Fxq "$WS_SOURCE" "$BASHRC"; then
    echo "$WS_SOURCE" >> "$BASHRC"
    echo "Added workspace sourcing to ~/.bashrc ✅"
else
    echo "Workspace sourcing already in ~/.bashrc"
fi

# Step 6: Verify
echo "--- Step 5: Verifying workspace ---"
source $WS_ROOT/install/setup.bash
ros2 pkg list | grep rover_simulation && echo "rover_simulation package found ✅" || echo "Package not found ❌"

echo ""
echo "=== PHASE F COMPLETE ==="
