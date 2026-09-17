#!/usr/bin/env python3
"""
pybullet_sim.py
---------------
Replaces Gazebo entirely.
- Simulates lunar rover with 4 wheels on rough terrain
- Publishes /scan (LiDAR), /imu/data, /rover/odom to ROS2
- Subscribes to /rover/cmd_vel to drive the rover
- Opens a 3D GUI window (works on WSL2)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Header
import pybullet as p
import pybullet_data
import numpy as np
import math
import time


class PyBulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim')

        # ── PyBullet setup ──────────────────────────────────────
        self.client = p.connect(p.GUI)          # GUI window
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -1.62)              # Moon gravity
        p.setRealTimeSimulation(0)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

        # Camera angle
        p.resetDebugVisualizerCamera(
            cameraDistance=3.0,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0]
        )

        # ── Build lunar terrain ──────────────────────────────────
        self._build_terrain()

        # ── Build rover ──────────────────────────────────────────
        self._build_rover()

        # ── ROS2 publishers ─────────────────────────────────────
        self.scan_pub  = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub   = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub  = self.create_publisher(Odometry, '/rover/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── ROS2 subscriber ──────────────────────────────────────
        self.cmd_sub = self.create_subscription(
            Twist, '/rover/cmd_vel', self.cmd_callback, 10)
        self.cmd_sub_global = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # ── State ────────────────────────────────────────────────
        self.linear_vel  = 0.0
        self.angular_vel = 0.0
        self.wheel_sep   = 0.44
        self.wheel_radius = 0.08

        # ── Timers ───────────────────────────────────────────────
        self.create_timer(0.1,  self.step_simulation)   # 10 Hz physics
        self.create_timer(0.1,  self.publish_lidar)     # 10 Hz LiDAR
        self.create_timer(0.02, self.publish_imu_odom)  # 50 Hz IMU/odom

        self.get_logger().info('PyBullet simulation started')
        self.get_logger().info('Drive with: ros2 run teleop_twist_keyboard '
                               'teleop_twist_keyboard --ros-args '
                               '--remap cmd_vel:=/rover/cmd_vel')

    # ─────────────────────────────────────────────────────────────
    def _build_terrain(self):
        """Create lunar terrain: flat ground + rocks + slope."""
        # Ground — grey lunar regolith
        ground_shape = p.createCollisionShape(p.GEOM_PLANE)
        ground_vis   = p.createVisualShape(
            p.GEOM_PLANE,
            planeNormal=[0, 0, 1],
            rgbaColor=[0.55, 0.53, 0.50, 1]
        )
        p.createMultiBody(0, ground_shape, ground_vis)

        # Rocks
        rocks = [
            ([3.0,  1.0, 0.3], [0.5, 0.4, 0.4]),
            ([-2.0, 4.0, 0.3], [0.35, 0.35, 0.4]),
            ([6.0, -2.0, 0.5], [0.7, 0.5, 0.5]),
            ([1.5,  3.0, 0.2], [0.3, 0.3, 0.3]),
            ([-1.0,-3.0, 0.25],[0.4, 0.4, 0.3]),
        ]
        for pos, half_ext in rocks:
            shape = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=half_ext)
            vis   = p.createVisualShape(
                p.GEOM_BOX, halfExtents=half_ext,
                rgbaColor=[0.48, 0.44, 0.40, 1])
            p.createMultiBody(0, shape, vis, basePosition=pos)

        # Slope
        slope_shape = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[2.0, 1.5, 0.1])
        slope_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[2.0, 1.5, 0.1],
            rgbaColor=[0.52, 0.50, 0.47, 1])
        orn = p.getQuaternionFromEuler([0, 0.25, 0.3])
        p.createMultiBody(0, slope_shape, slope_vis,
                          basePosition=[-4, -3, 0.5],
                          baseOrientation=orn)

        # Crater rim (4 walls)
        rim_positions = [
            ([5, 7,  0.15], [0, 0, 0]),
            ([5, 3,  0.15], [0, 0, 0]),
            ([7, 5,  0.15], [0, 0, math.pi/2]),
            ([3, 5,  0.15], [0, 0, math.pi/2]),
        ]
        for pos, rpy in rim_positions:
            shape = p.createCollisionShape(
                p.GEOM_BOX, halfExtents=[1.5, 0.2, 0.15])
            vis = p.createVisualShape(
                p.GEOM_BOX, halfExtents=[1.5, 0.2, 0.15],
                rgbaColor=[0.46, 0.44, 0.41, 1])
            orn = p.getQuaternionFromEuler(rpy)
            p.createMultiBody(0, shape, vis,
                              basePosition=pos,
                              baseOrientation=orn)

    def _build_rover(self):
        """Build a simple rover with a box chassis and 4 cylinder wheels."""
        # Chassis
        chassis_col = p.createCollisionShape(
            p.GEOM_BOX, halfExtents=[0.25, 0.20, 0.075])
        chassis_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.25, 0.20, 0.075],
            rgbaColor=[0.6, 0.6, 0.6, 1])

        # Wheels
        wheel_col = p.createCollisionShape(
            p.GEOM_CYLINDER, radius=0.08, height=0.05)
        wheel_vis = p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.08, length=0.05,
            rgbaColor=[0.15, 0.15, 0.15, 1])

        wheel_positions = [
            [ 0.18,  0.22, 0],
            [ 0.18, -0.22, 0],
            [-0.18,  0.22, 0],
            [-0.18, -0.22, 0],
        ]

        link_masses        = [0.5] * 4
        link_col_shapes    = [wheel_col] * 4
        link_vis_shapes    = [wheel_vis] * 4
        link_positions     = wheel_positions
        link_orientations  = [p.getQuaternionFromEuler([math.pi/2, 0, 0])] * 4
        link_inertial_pos  = [[0, 0, 0]] * 4
        link_inertial_orn  = [[0, 0, 0, 1]] * 4
        link_parent_idx    = [0, 0, 0, 0]
        link_joint_types   = [p.JOINT_REVOLUTE] * 4
        link_joint_axes    = [[0, 0, 1]] * 4

        self.rover = p.createMultiBody(
            baseMass=10.0,
            baseCollisionShapeIndex=chassis_col,
            baseVisualShapeIndex=chassis_vis,
            basePosition=[0, 0, 0.2],
            linkMasses=link_masses,
            linkCollisionShapeIndices=link_col_shapes,
            linkVisualShapeIndices=link_vis_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_pos,
            linkInertialFrameOrientations=link_inertial_orn,
            linkParentIndices=link_parent_idx,
            linkJointTypes=link_joint_types,
            linkJointAxis=link_joint_axes,
        )

        # Wheel friction
        for i in range(4):
            p.changeDynamics(self.rover, i,
                             lateralFriction=1.0,
                             spinningFriction=0.1)

        self.wheel_indices = [0, 1, 2, 3]

    # ─────────────────────────────────────────────────────────────
    def cmd_callback(self, msg: Twist):
        self.linear_vel  = msg.linear.x
        self.angular_vel = msg.angular.z

    def step_simulation(self):
        """Apply differential drive and step physics."""
        v_left  = (self.linear_vel - self.angular_vel * self.wheel_sep / 2)
        v_right = (self.linear_vel + self.angular_vel * self.wheel_sep / 2)

        w_left  = v_left  / self.wheel_radius
        w_right = v_right / self.wheel_radius

        # Left wheels: 0, 2 | Right wheels: 1, 3
        speeds = [w_left, w_right, w_left, w_right]
        for i, speed in zip(self.wheel_indices, speeds):
            p.setJointMotorControl2(
                self.rover, i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=speed,
                force=20.0
            )

        p.stepSimulation()

    def publish_lidar(self):
        """Simulate 360° LiDAR using PyBullet raycasting."""
        pos, orn = p.getBasePositionAndOrientation(self.rover)
        lidar_pos = [pos[0], pos[1], pos[2] + 0.27]  # sensor height

        # Get rover yaw for LiDAR orientation
        euler = p.getEulerFromQuaternion(orn)
        yaw   = euler[2]

        num_rays  = 360
        max_range = 12.0
        min_range = 0.12

        ray_starts = []
        ray_ends   = []
        for i in range(num_rays):
            angle = yaw + (i / num_rays) * 2 * math.pi - math.pi
            ray_starts.append(lidar_pos)
            ray_ends.append([
                lidar_pos[0] + max_range * math.cos(angle),
                lidar_pos[1] + max_range * math.sin(angle),
                lidar_pos[2] + 0.27
            ])

        results = p.rayTestBatch(ray_starts, ray_ends)
        ranges  = []
        for r in results:
            hit_fraction = r[2]
            dist = hit_fraction * max_range
            if dist < min_range:
                dist = max_range
            ranges.append(dist + np.random.normal(0, 0.01))

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min       = -math.pi
        msg.angle_max       =  math.pi
        msg.angle_increment = (2 * math.pi) / num_rays
        msg.time_increment  = 0.0
        msg.scan_time       = 0.1
        msg.range_min       = min_range
        msg.range_max       = max_range
        msg.ranges          = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def publish_imu_odom(self):
        """Publish IMU and odometry from PyBullet state."""
        pos, orn = p.getBasePositionAndOrientation(self.rover)
        vel, ang_vel = p.getBaseVelocity(self.rover)
        now = self.get_clock().now().to_msg()

        # ── Odometry ──────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x  = pos[0]
        odom.pose.pose.position.y  = pos[1]
        odom.pose.pose.position.z  = pos[2]
        odom.pose.pose.orientation.x = orn[0]
        odom.pose.pose.orientation.y = orn[1]
        odom.pose.pose.orientation.z = orn[2]
        odom.pose.pose.orientation.w = orn[3]
        odom.twist.twist.linear.x  = vel[0]
        odom.twist.twist.linear.y  = vel[1]
        odom.twist.twist.angular.z = ang_vel[2]
        self.odom_pub.publish(odom)

        # ── TF: odom → base_footprint ────────────────────────────
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = orn[0]
        t.transform.rotation.y = orn[1]
        t.transform.rotation.z = orn[2]
        t.transform.rotation.w = orn[3]
        self.tf_broadcaster.sendTransform(t)

        # ── IMU ───────────────────────────────────────────────────
        imu = Imu()
        imu.header.stamp    = now
        imu.header.frame_id = 'imu_link'
        imu.orientation.x = orn[0]
        imu.orientation.y = orn[1]
        imu.orientation.z = orn[2]
        imu.orientation.w = orn[3]
        imu.angular_velocity.x = ang_vel[0] + np.random.normal(0, 0.002)
        imu.angular_velocity.y = ang_vel[1] + np.random.normal(0, 0.002)
        imu.angular_velocity.z = ang_vel[2] + np.random.normal(0, 0.002)
        imu.linear_acceleration.x = vel[0] + np.random.normal(0, 0.017)
        imu.linear_acceleration.y = vel[1] + np.random.normal(0, 0.017)
        imu.linear_acceleration.z = -1.62  + np.random.normal(0, 0.017)
        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
