#!/usr/bin/env python3
"""First-run checks for the Arjuna Edu Kit before letting quantum_nav drive.

    rosrun arjuna_quantum_nav drive_test.py _mode:=sensors     # no motion: print sensor readings
    rosrun arjuna_quantum_nav drive_test.py _mode:=straight _distance:=0.5
    rosrun arjuna_quantum_nav drive_test.py _mode:=turn _angle_deg:=90

sensors  : prints pose, LiDAR front clearance, IMU roll/pitch and sonar for 10 s.
           Put a box ~30 cm in FRONT of the robot: "lidar front" must read ~0.30 m.
           If it doesn't, set ~laser_yaw_offset_deg (try 180) in quantum_nav.launch.
           Tilt the nose UP: pitch must go POSITIVE, else set ~pitch_sign to -1.
straight : drives forward slowly until the pose says `distance` metres; measure the real
           distance with a tape. A big mismatch means the kit's tick→metre calibration
           (odometry package) needs fixing before autonomous runs.
turn     : turns in place by `angle_deg` on the pose's yaw; check the real angle.
Ctrl+C stops the robot at any time.
"""

import math
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64

import nav_core

state = {"pose": None, "front": None, "roll": 0.0, "pitch": 0.0, "sonar": None}


def on_pose(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    state["pose"] = (p.x, p.y, nav_core.yaw_from_quaternion(q.x, q.y, q.z, q.w))


def make_on_scan(offset):
    def on_scan(msg):
        best, a = float("inf"), msg.angle_min
        for r in msg.ranges:
            if abs(nav_core.wrap_angle(a + offset)) < math.radians(15) and msg.range_min < r < msg.range_max:
                best = min(best, r)
            a += msg.angle_increment
        state["front"] = best
    return on_scan


def on_imu(msg):
    q = msg.orientation
    state["roll"], state["pitch"] = nav_core.roll_pitch_from_quaternion(q.x, q.y, q.z, q.w)


def on_sonar(msg):
    state["sonar"] = msg.data


def main():
    rospy.init_node("arjuna_drive_test")
    mode = rospy.get_param("~mode", "sensors")
    speed = min(float(rospy.get_param("~speed", 0.08)), 0.15)
    offset = math.radians(float(rospy.get_param("~laser_yaw_offset_deg", 0.0)))
    pub = rospy.Publisher(rospy.get_param("~cmd_vel_topic", "/cmd_vel"), Twist, queue_size=1)
    rospy.Subscriber(rospy.get_param("~pose_topic", "/robot_pose_ekf/odom_combined"),
                     PoseWithCovarianceStamped, on_pose)
    rospy.Subscriber("/scan", LaserScan, make_on_scan(offset))
    rospy.Subscriber("/imu/data", Imu, on_imu)
    rospy.Subscriber("/distance", Float64, on_sonar)

    def stop():
        for _ in range(5):
            pub.publish(Twist())
            time.sleep(0.05)
    rospy.on_shutdown(stop)

    t0 = time.time()
    while state["pose"] is None and not rospy.is_shutdown():
        if time.time() - t0 > 5:
            rospy.logerr("No pose on the EKF topic after 5 s — is the kit bringup (robot_pose_ekf) running?")
            return
        time.sleep(0.1)

    rate = rospy.Rate(10)
    if mode == "sensors":
        end = time.time() + 10
        while time.time() < end and not rospy.is_shutdown():
            x, y, yaw = state["pose"]
            front = state["front"]
            print("pose x=%.2f y=%.2f yaw=%5.1f° | lidar front %s | roll %5.1f° pitch %5.1f° | sonar %s" % (
                x, y, math.degrees(yaw), "—" if front in (None, float("inf")) else "%.2f m" % front,
                math.degrees(state["roll"]), math.degrees(state["pitch"]),
                "—" if state["sonar"] is None else "%.0f cm" % state["sonar"]))
            rate.sleep()
        return

    x0, y0, yaw0 = state["pose"]
    if mode == "straight":
        target = float(rospy.get_param("~distance", 0.5))
        while not rospy.is_shutdown():
            x, y, _ = state["pose"]
            d = math.hypot(x - x0, y - y0)
            if d >= target:
                break
            if state["front"] is not None and state["front"] < 0.25:
                rospy.logwarn("Obstacle ahead — stopping test")
                break
            t = Twist()
            t.linear.x = speed
            pub.publish(t)
            rate.sleep()
        stop()
        x, y, _ = state["pose"]
        print("Pose says %.3f m travelled. Measure the real distance and compare." % math.hypot(x - x0, y - y0))
    elif mode == "turn":
        target = math.radians(float(rospy.get_param("~angle_deg", 90.0)))
        turned, last = 0.0, yaw0
        while not rospy.is_shutdown() and abs(turned) < abs(target):
            yaw = state["pose"][2]
            turned += nav_core.wrap_angle(yaw - last)
            last = yaw
            t = Twist()
            t.angular.z = math.copysign(0.4, target)
            pub.publish(t)
            rate.sleep()
        stop()
        print("Pose says %.1f° turned. Check the real angle." % math.degrees(turned))
    else:
        rospy.logerr("Unknown ~mode %r (sensors | straight | turn)", mode)


if __name__ == "__main__":
    main()
