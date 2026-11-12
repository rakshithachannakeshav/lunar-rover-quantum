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
    /terrain/status      std_msgs/String  what it sees right now (for rostopic echo); first line
                         as before, then one line per object in view, e.g.
                         "rock #2: 31 x 27 cm, 24 cm tall, 0.82 m ahead"
    /terrain/objects     std_msgs/String  JSON {"frame_id", "stamp", "rover", "objects": [{id,
                         class (crater|rock|wall|table_edge), x, y (map frame, ~map_frame), yaw,
                         length_m, width_m, height_m, height_is_min, footprint_complete,
                         inner_diameter_m, outer_diameter_m (craters), distance_m (rover centre
                         to the object's edge), bearing_deg, frames_seen, confidence, visible,
                         outline [[x, y], ...], label, text}]}. Ids stay the same while the
                         rover drives (terrain_objects.py). Without a pose the frame is
                         ~frame_id (robot frame) instead.
    /terrain/object_markers  visualization_msgs/MarkerArray  label text + outline per object
    /terrain/preview/compressed  sensor_msgs/CompressedImage  (only with _use_preview:=true)
                         low-rate JPEG from the colour camera with each object drawn on it

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
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseArray, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

import crater_core
import terrain_core
import terrain_objects

try:
    import depthai as dai
except ImportError:          # no camera library: everything else still runs
    dai = None

try:
    import queue as _queue
except ImportError:          # Python 2 (not used on the Nano, kept harmless)
    import Queue as _queue

# RViz / preview colours per object class (r, g, b in 0..1)
CLASS_RGB = {"crater": (1.0, 0.55, 0.0), "rock": (0.95, 0.25, 0.25),
             "wall": (0.66, 0.33, 0.97), "table_edge": (0.6, 0.6, 0.65)}


def _socket(new_name, old_name):
    s = getattr(dai.CameraBoardSocket, new_name, None)
    return s if s is not None else getattr(dai.CameraBoardSocket, old_name)


def make_pipeline(fps, preview=None):
    """Stereo depth; with `preview` = (width, height, fps) also a small BGR colour stream."""
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
    if preview is not None:
        w, h, cam_fps = preview
        rgb = pipeline.create(dai.node.ColorCamera)
        rgb.setBoardSocket(_socket("CAM_A", "RGB"))
        rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        rgb.setPreviewSize(w, h)                 # scaled from the 1920x1080 ISP image
        rgb.setInterleaved(True)
        rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        rgb.setFps(cam_fps)
        xrgb = pipeline.create(dai.node.XLinkOut)
        xrgb.setStreamName("rgb")
        rgb.preview.link(xrgb.input)
    return pipeline


def intrinsics(device, width, height):
    m = device.readCalibration().getCameraIntrinsics(_socket("CAM_C", "RIGHT"), width, height)
    return crater_core.Intrinsics(m[0][0], m[1][1], m[0][2], m[1][2], width, height)


def colour_geometry(device, width, height):
    """(intrinsics, rotation, translation m) of the colour preview relative to the depth
    camera (the right mono camera the depth is aligned to). Depth is rectified and this uses
    the raw right-camera extrinsics, so overlays can be off by a few pixels: fine for a view."""
    calib = device.readCalibration()
    m = calib.getCameraIntrinsics(_socket("CAM_A", "RGB"), 1920, 1080)
    k = width / 1920.0
    intr = crater_core.Intrinsics(m[0][0] * k, m[1][1] * k, m[0][2] * k, m[1][2] * k,
                                  width, height)
    ext = np.array(calib.getCameraExtrinsics(_socket("CAM_C", "RIGHT"), _socket("CAM_A", "RGB")),
                   dtype=np.float64)
    return intr, ext[:3, :3], ext[:3, 3] / 100.0          # depthai gives centimetres


class PreviewWorker(threading.Thread):
    """Draws the objects on a colour frame, JPEG-encodes it and publishes it, off the detector
    thread. Holds at most one pending frame: if it is still busy, new frames are dropped, so
    it can never slow the detector down."""

    def __init__(self, cv2, pub, quality, geometry, floor, camera_forward):
        threading.Thread.__init__(self)
        self.daemon = True
        self.cv2, self.pub, self.quality = cv2, pub, int(quality)
        self.intr, self.rot, self.trans = geometry
        self.floor, self.camera_forward = floor, camera_forward
        self.jobs = _queue.Queue(maxsize=1)
        self.times = []

    def offer(self, frame, objects, pose):
        try:
            self.jobs.put_nowait((frame, objects, pose))
            return True
        except _queue.Full:
            return False

    def run(self):
        while not rospy.is_shutdown():
            try:
                frame, objects, pose = self.jobs.get(timeout=1.0)
            except _queue.Empty:
                continue
            t0 = time.time()
            try:
                frame = np.array(frame, dtype=np.uint8, copy=True)   # never draw on a driver buffer
                self._draw(frame, objects, pose)
                ok, buf = self.cv2.imencode(".jpg", frame,
                                            [int(self.cv2.IMWRITE_JPEG_QUALITY), self.quality])
                if ok:
                    msg = CompressedImage()
                    msg.header.stamp, msg.header.frame_id = rospy.Time.now(), "oak_rgb"
                    msg.format = "jpeg"
                    msg.data = buf.tobytes()
                    self.pub.publish(msg)
            except Exception as e:             # a drawing bug must not kill the detector
                rospy.logwarn_throttle(10.0, "preview: %s" % e)
            self.times.append(time.time() - t0)
            del self.times[:-20]

    def _draw(self, img, objects, pose):
        cv2 = self.cv2
        if pose is None:
            pose = (0.0, 0.0, 0.0)
        for ov in terrain_objects.image_overlays(objects, pose, self.floor, self.camera_forward,
                                                 self.rot, self.trans, self.intr):
            r, g, b = CLASS_RGB.get(ov["class"], (1.0, 1.0, 1.0))
            col = (int(255 * b), int(255 * g), int(255 * r))
            for line in ov["lines"]:
                pts = np.array([[int(round(u)), int(round(v))] for u, v in line], dtype=np.int32)
                cv2.polylines(img, [pts], False, col, 2)
            x, y = int(ov["anchor"][0]), int(max(12, ov["anchor"][1]))
            (tw, th), _ = cv2.getTextSize(ov["label"], cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
            cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + 3), (20, 20, 20), -1)
            cv2.putText(img, ov["label"], (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.42, col, 1)

    def mean_ms(self):
        return 1000.0 * sum(self.times) / len(self.times) if self.times else 0.0


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
        # objects with sizes and stable ids (terrain_objects.py)
        self.map_frame = p("~map_frame", "map")      # frame of the pose topic = the navigator's map
        self.tracker = terrain_objects.ObjectTracker(
            miss_limit=int(p("~object_miss_frames", 6)),
            min_frames=int(p("~object_min_frames", 2)))
        self.tracking_with_pose = None
        self.objects = []
        self.obj_pub = rospy.Publisher("/terrain/objects", String, queue_size=1)
        self.obj_marker_pub = rospy.Publisher("/terrain/object_markers", MarkerArray, queue_size=1)
        self.markers_t = 0.0
        self.objects_logged = (None, 0.0)
        # optional colour preview with the objects drawn on it (off by default)
        self.use_preview = bool(p("~use_preview", False))
        self.preview_fps = float(p("~preview_fps", 2.0))
        self.preview_width = int(p("~preview_width", 480))
        self.preview_quality = int(p("~preview_quality", 70))
        self.preview_pub = None
        self.cv2 = None
        if self.use_preview:
            try:
                import cv2
                self.cv2 = cv2
                self.preview_pub = rospy.Publisher("/terrain/preview/compressed", CompressedImage,
                                                   queue_size=1)
            except ImportError:
                rospy.logwarn("use_preview needs OpenCV for Python 3 (python3-opencv); "
                              "preview off, the detector runs as usual")
                self.use_preview = False
        self.timing = {"n": 0, "process": 0.0, "t0": time.time()}

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
        lines = terrain_objects.status_lines(self.objects)
        self.status_pub.publish(String(data="\n".join([status] + lines)))
        if status != self.last_status and dets:
            rospy.loginfo(status)
        self.last_status = status
        # object lines: log when the set in view changes, else at most every 5 s
        ids = tuple(o["id"] for o in self.objects if o["visible"])
        last_ids, last_t = self.objects_logged
        if lines and (ids != last_ids or time.time() - last_t > 5.0):
            for line in lines:
                rospy.loginfo("  %s", line)
            self.objects_logged = (ids, time.time())
        elif not lines:
            self.objects_logged = ((), last_t)

    # ── objects ──────────────────────────────────────────────────────────────
    def _update_objects(self, info, pose, detector):
        """Track the measured objects (map frame with a pose, robot frame without)."""
        with_pose = pose is not None
        if with_pose != self.tracking_with_pose:     # frames changed: ids would not match
            self.tracker.reset()
            self.tracking_with_pose = with_pose
        frame = self.map_frame if with_pose else self.frame_id
        self.objects = self.tracker.update(info.get("objects", []), pose, sees=detector.sees,
                                           stamp=time.time())
        self.obj_pub.publish(String(data=json.dumps(terrain_objects.objects_message(
            self.objects, frame, time.time(), pose))))
        if time.time() - self.markers_t >= 0.25:
            self.markers_t = time.time()
            self.obj_marker_pub.publish(self._object_markers(frame))

    def _object_markers(self, frame):
        """RViz: one text label + one outline per object (add a MarkerArray display for
        /terrain/object_markers next to /quantum_nav/terrain_markers)."""
        arr = MarkerArray()
        wipe = Marker()
        wipe.action = Marker.DELETEALL
        arr.markers.append(wipe)
        stamp = rospy.Time.now()
        for o in self.objects:
            r, g, b = CLASS_RGB.get(o["class"], (1.0, 1.0, 1.0))
            alpha = 1.0 if o["visible"] else 0.45
            top = (o["height_m"] or 0.0)
            t = Marker()
            t.header.frame_id, t.header.stamp = frame, stamp
            t.ns, t.id, t.type, t.action = "terrain_object_labels", o["id"], \
                Marker.TEXT_VIEW_FACING, Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = o["x"], o["y"], top + 0.08
            t.pose.orientation.w = 1.0
            t.scale.z = 0.045
            t.color.r, t.color.g, t.color.b, t.color.a = 1.0, 1.0, 1.0, alpha
            t.text = o["label"]
            arr.markers.append(t)
            if o["outline"]:
                ln = Marker()
                ln.header.frame_id, ln.header.stamp = frame, stamp
                ln.ns, ln.id, ln.type, ln.action = "terrain_object_outlines", o["id"], \
                    Marker.LINE_STRIP, Marker.ADD
                ln.pose.orientation.w = 1.0
                ln.scale.x = 0.01
                ln.color.r, ln.color.g, ln.color.b, ln.color.a = r, g, b, alpha
                z = top if o["class"] in ("rock", "wall") else 0.03
                for x, y in o["outline"] + o["outline"][:1]:
                    ln.points.append(Point(x=x, y=y, z=z))
                arr.markers.append(ln)
        return arr

    def _log_timing(self, preview):
        tm = self.timing
        if time.time() - tm["t0"] < 30.0 or not tm["n"]:
            return
        rospy.loginfo("terrain: %.1f frames/s, %.0f ms per frame (objects %s)%s",
                      tm["n"] / (time.time() - tm["t0"]), 1000.0 * tm["process"] / tm["n"],
                      "on" if self.params.measure_objects else "off",
                      ", preview %.0f ms per image" % preview.mean_ms() if preview else "")
        self.timing = {"n": 0, "process": 0.0, "t0": time.time()}

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

    def _open_device(self):
        """dai.Device with the colour preview if asked for; without it if that fails (no colour
        camera, old depthai), so the preview can never stop the terrain detector."""
        want = self.use_preview and not self.measure
        if want:
            h = int(round(self.preview_width * 9 / 16.0)) // 2 * 2
            cam_fps = max(2, int(round(2 * self.preview_fps)))
            try:
                return dai.Device(make_pipeline(self.fps, (self.preview_width, h, cam_fps))), True
            except (RuntimeError, AttributeError) as e:
                rospy.logwarn("Colour preview unavailable (%s); running without it", e)
                self.use_preview = False
        return dai.Device(make_pipeline(self.fps)), False

    def _run_camera(self):
        device, with_preview = self._open_device()
        with device:
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
            preview, rgb_queue, preview_t = None, None, 0.0
            if with_preview:
                try:
                    rgb_queue = device.getOutputQueue(name="rgb", maxSize=1, blocking=False)
                    h = int(round(self.preview_width * 9 / 16.0)) // 2 * 2
                    preview = PreviewWorker(self.cv2, self.preview_pub, self.preview_quality,
                                            colour_geometry(device, self.preview_width, h),
                                            self.floor, self.params.camera_forward)
                    preview.start()
                    rospy.loginfo("Colour preview on /terrain/preview/compressed at %.1f fps",
                                  self.preview_fps)
                except Exception as e:      # no colour calibration etc.: detector carries on
                    rospy.logwarn("Colour preview off: %s", e)
                    preview = None
            rate = rospy.Rate(self.rate_hz)
            while not rospy.is_shutdown():
                pkt = queue.tryGet()
                if pkt is not None:
                    pose = self._fresh_pose()
                    if pose is None:        # no odometry: frames cannot be fused while moving
                        detector.reset()
                    t0 = time.time()
                    dets, info = detector.process(pkt.getFrame(), pose)
                    self.timing["process"] += time.time() - t0
                    self.timing["n"] += 1
                    self._update_objects(info, pose, detector)
                    self._publish(dets)
                if preview is not None and time.time() - preview_t >= 1.0 / self.preview_fps:
                    preview_t = time.time()
                    try:
                        rgb = rgb_queue.tryGet()
                        if rgb is not None:
                            preview.offer(rgb.getCvFrame(), list(self.objects),
                                          self._fresh_pose())
                    except Exception as e:
                        rospy.logwarn_throttle(10.0, "preview frame: %s" % e)
                self._log_timing(preview)
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
