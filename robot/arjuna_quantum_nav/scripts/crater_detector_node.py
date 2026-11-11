#!/usr/bin/env python3
"""ROS Melodic node: see the lunar arena's terrain with the OAK-D Lite depth camera.

Finds, from the calibrated floor plane (terrain_core.py):
    crater RIMS   raised ~2 cm rings; the whole circle inside is reported (rim + inside)
    rocks, the border wall   anything clearly above the floor
    no floor      the table edge (no valid ground where floor should be)

Publishes (frame base_footprint: x forward, y left, metres)
    /terrain/detections  std_msgs/String  JSON {"detections": [{type, cells, x, y, r, ...}]}
                         type = crater | rock | border | nofloor. quantum_nav_node.py reads this.
    /craters/points      geometry_msgs/PoseArray  crater cells only (kept for older users)
    /terrain/status      std_msgs/String  what it sees right now (for rostopic echo)

Floor calibration: on first start (or with _calibrate:=true) the rover must stand on FLAT,
CLEAR floor with ~1 m of it in front of the camera. The node fits the floor plane (camera height
and tilt, no tape measure needed) and saves it to ~calib_file. Redo it if the camera mount moves.

Measure mode (_measure:=true): stand still, first looking at plain floor, then at a crater from
0.4-0.8 m. The node records ~measure_frames frames, prints the stereo noise on the floor and the
height of the crater rim, says whether the 2 cm rim is visible above the noise and suggests
rim_min / tall_min, then exits. Use those numbers for the real run.

Uses the stereo depth pipeline from the kit's Lab 17 (MonoCamera 400p + StereoDepth). Stop the
kit's own Arjuna_camera_pub.py first: only one program can open the camera. With no camera
attached (or depthai missing) this node just reports "no camera" and the navigator carries on
with LiDAR and the IMU alone.
"""

import json
import os
import time

import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import crater_core
import terrain_core

try:
    import depthai as dai
except ImportError:          # no camera library: everything else still runs
    dai = None


def _socket(new_name, old_name):
    s = getattr(dai.CameraBoardSocket, new_name, None)
    return s if s is not None else getattr(dai.CameraBoardSocket, old_name)


def make_pipeline(fps):
    pipeline = dai.Pipeline()
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for cam, sock in ((left, _socket("CAM_B", "LEFT")), (right, _socket("CAM_C", "RIGHT"))):
        cam.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        cam.setBoardSocket(sock)
        cam.setFps(fps)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)   # sees the ground close to the rover (< 30 cm)
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    xout = pipeline.create(dai.node.XLinkOut)
    xout.setStreamName("depth")
    stereo.depth.link(xout.input)
    return pipeline


def intrinsics(device, width, height):
    m = device.readCalibration().getCameraIntrinsics(_socket("CAM_C", "RIGHT"), width, height)
    return crater_core.Intrinsics(m[0][0], m[1][1], m[0][2], m[1][2], width, height)


class TerrainNode(object):
    def __init__(self):
        p = rospy.get_param
        self.calib_file = os.path.expanduser(p("~calib_file", "~/.ros/crater_camera.json"))
        self.calibrate = bool(p("~calibrate", False))
        self.calib_frames = int(p("~calib_frames", 15))
        self.measure = bool(p("~measure", False))
        self.measure_frames = int(p("~measure_frames", 20))
        self.rate_hz = float(p("~rate", 8.0))
        self.fps = int(p("~fps", 15))
        self.frame_id = p("~frame_id", "base_footprint")
        # every TerrainParams field can be set as a ROS parameter, e.g. _rim_min:=0.012
        defaults = terrain_core.TerrainParams()
        self.params = terrain_core.TerrainParams(**dict(
            (k, type(v)(p("~" + k, v))) for k, v in sorted(vars(defaults).items())))
        self.pose = None
        self.pose_time = 0.0
        pose_topic = p("~pose_topic", "/robot_pose_ekf/odom_combined")
        if p("~pose_type", "pose_cov") == "odometry":
            rospy.Subscriber(pose_topic, Odometry, self._on_pose, queue_size=1)
        else:
            rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, self._on_pose, queue_size=1)
        self.det_pub = rospy.Publisher("/terrain/detections", String, queue_size=1)
        self.pub = rospy.Publisher("/craters/points", PoseArray, queue_size=1)
        self.status_pub = rospy.Publisher("/terrain/status", String, queue_size=1)
        self.floor = None
        self.last_status = None

    def _on_pose(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (pose.position.x, pose.position.y, float(yaw))
        self.pose_time = time.time()

    def _fresh_pose(self):
        """Odometry pose for fusing frames while driving; None if it has gone quiet."""
        return self.pose if time.time() - self.pose_time < 0.5 else None

    # ── floor calibration ────────────────────────────────────────────────────
    def _load_floor(self):
        if self.calibrate or not os.path.exists(self.calib_file):
            return None
        with open(self.calib_file) as f:
            floor = crater_core.FloorModel.from_json(f.read())
        rospy.loginfo("Floor calibration loaded from %s: camera %.2f m up, tilted %.0f deg down",
                      self.calib_file, floor.camera_height, floor.tilt_deg)
        return floor

    def _calibrate(self, queue, intr):
        rospy.loginfo("Calibrating the floor: keep the rover still on flat, clear ground...")
        u, v = crater_core.pixel_grid(intr, step=6)
        chunks = []
        while len(chunks) < self.calib_frames and not rospy.is_shutdown():
            frame = queue.get().getFrame()
            chunks.append(crater_core.depth_to_points(frame, intr, u, v))
        try:
            floor = crater_core.fit_floor(np.concatenate(chunks, axis=0))
        except ValueError as e:
            raise RuntimeError("Floor fit failed: %s. Is the camera pointed at clear floor?" % e)
        h, tilt = floor.camera_height, floor.tilt_deg
        if not (0.03 < h < 1.0 and 3.0 < tilt < 85.0):
            raise RuntimeError("Floor fit looks wrong (camera %.2f m up, tilt %.0f deg). Is the "
                               "camera pointed at clear, flat floor?" % (h, tilt))
        d = os.path.dirname(self.calib_file)
        if d and not os.path.isdir(d):
            os.makedirs(d)
        with open(self.calib_file, "w") as f:
            f.write(floor.to_json())
        rospy.loginfo("Floor calibrated: camera %.2f m above the floor, tilted %.0f deg down. "
                      "Saved to %s. Check the height with a tape measure.",
                      h, tilt, self.calib_file)
        return floor

    # ── publishing ───────────────────────────────────────────────────────────
    def _publish(self, dets):
        stamp = rospy.Time.now()
        self.det_pub.publish(String(data=json.dumps(
            {"detections": terrain_core.detections_to_json_obj(dets)})))
        msg = PoseArray()
        msg.header.frame_id, msg.header.stamp = self.frame_id, stamp
        for d in dets:
            if d["type"] != "crater":
                continue
            for x, y in d["cells"]:
                q = Pose()
                q.position.x, q.position.y = float(x), float(y)
                q.orientation.w = 1.0
                msg.poses.append(q)
        self.pub.publish(msg)
        if dets:
            parts = []
            for d in dets:
                parts.append("%s@(%.2f,%.2f)" % (d["type"], d["x"], d["y"]) +
                             (" r=%.2f" % d["r"] if d["type"] == "crater" and d["r"] else ""))
            status = "sees " + ", ".join(parts)
        else:
            status = "clear"
        self.status_pub.publish(String(data=status))
        if status != self.last_status and dets:
            rospy.loginfo(status)
        self.last_status = status

    def _idle(self, text):
        self.status_pub.publish(String(data=text))
        rospy.logwarn_throttle(10.0, text)
        time.sleep(1.0)

    # ── main loop ────────────────────────────────────────────────────────────
    def run(self):
        if dai is None:
            while not rospy.is_shutdown():
                self._idle("no camera: depthai is not installed (camera features off)")
            return
        while not rospy.is_shutdown():
            try:
                self._run_camera()
                return
            except RuntimeError as e:
                if "calibrat" in str(e).lower() or "floor fit" in str(e).lower():
                    rospy.logerr(str(e))
                    return
                self._idle("no camera: %s (camera features off, retrying)" % e)

    def _run_camera(self):
        with dai.Device(make_pipeline(self.fps)) as device:
            queue = device.getOutputQueue(name="depth", maxSize=2, blocking=False)
            first = queue.get().getFrame()
            intr = intrinsics(device, first.shape[1], first.shape[0])
            self.floor = self._load_floor() or self._calibrate(queue, intr)
            detector = terrain_core.TerrainDetector(self.floor, intr, self.params)
            if self.measure:
                self._measure(queue, detector)
                return
            rospy.loginfo("Terrain detector running: floor %.2f-%.2f m ahead, +/-%.2f m wide; rim "
                          "height >= %.1f cm, obstacles >= %.1f cm above the floor",
                          self.params.x_min, self.params.x_max, self.params.half_width,
                          self.params.rim_min * 100, self.params.tall_min * 100)
            rate = rospy.Rate(self.rate_hz)
            while not rospy.is_shutdown():
                pkt = queue.tryGet()
                if pkt is not None:
                    pose = self._fresh_pose()
                    if pose is None:        # no odometry: frames cannot be fused while moving
                        detector.reset()
                    dets, _ = detector.process(pkt.getFrame(), pose)
                    self._publish(dets)
                rate.sleep()

    def _measure(self, queue, detector):
        rospy.loginfo("MEASURE: keep the rover still. Recording %d frames...", self.measure_frames)
        frames = []
        while len(frames) < self.measure_frames and not rospy.is_shutdown():
            frames.append(np.array(queue.get().getFrame()))
        m = terrain_core.measure(detector, frames)
        for k in sorted(m):
            if k not in ("message", "detections"):
                rospy.loginfo("  %-32s %s", k, m[k])
        rospy.loginfo("  detections: %s", m["detections"])
        rospy.loginfo("RESULT: %s", m["message"])
        out = os.path.expanduser("~/.ros/terrain_measure.json")
        try:
            with open(out, "w") as f:
                json.dump(m, f, indent=2, default=float)
            rospy.loginfo("Saved to %s", out)
        except (IOError, OSError) as e:
            rospy.logwarn("Could not save %s: %s", out, e)


if __name__ == "__main__":
    rospy.init_node("crater_detector")
    TerrainNode().run()
