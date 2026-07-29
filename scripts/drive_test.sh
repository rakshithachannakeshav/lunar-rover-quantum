#!/usr/bin/env bash
# Quick test: move rover forward 3 seconds (run while sim is up)
source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source ~/lunar-rover-quantum/install/setup.bash

echo "Publishing /cmd_vel linear.x=0.3 for 5 seconds..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 5
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
echo "Done."
