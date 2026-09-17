#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
source /home/monisha/lunar-rover-quantum/lunar_rover_ws/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/opt/gz_sim_vendor/lib

echo "1. Starting Gazebo server (headless) ..."
gz sim -s -r /home/monisha/lunar-rover-quantum/lunar_rover_ws/install/rover_simulation/share/rover_simulation/worlds/lunar_terrain.world &
GZ_PID=$!
sleep 5

echo "2. Starting Robot State Publisher ..."
xacro /home/monisha/lunar-rover-quantum/lunar_rover_ws/install/rover_simulation/share/rover_simulation/urdf/rover.urdf.xacro > /tmp/rover.urdf
ros2 run robot_state_publisher robot_state_publisher /tmp/rover.urdf &
RSP_PID=$!
sleep 2

echo "3. Starting ROS-GZ Bridge ..."
ros2 run ros_gz_bridge parameter_bridge \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
  /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model &
BRIDGE_PID=$!
sleep 3

echo "4. Spawning Lunar Rover at z=2.0 ..."
ros2 run ros_gz_sim create -name lunar_rover -topic robot_description -z 2.0
sleep 6

echo "5. Checking active topics ..."
ros2 topic list

echo "6. Capturing Odometry ..."
ros2 topic echo --once /odom

echo "7. Capturing TF ..."
timeout 2 ros2 topic echo --once /tf

echo "8. Cleaning up ..."
kill $GZ_PID $RSP_PID $BRIDGE_PID
echo "Done!"
