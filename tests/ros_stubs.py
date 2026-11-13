"""Minimal stand-ins for rospy and the ROS 1 message packages, so the REAL
quantum_nav_node.py can be imported and driven by a simulator without ROS installed.

install() puts fake modules into sys.modules (idempotent). Publishers record what was
published in `rospy.PUBLISHED`; Subscriber callbacks are kept in `rospy.SUBSCRIBERS`;
`rospy.PARAMS` holds the node's private parameters (keys without the "~").
"""

import sys
import types


class Msg(object):
    """Attribute bag; subclasses list their fields as (name, default factory)."""
    FIELDS = ()

    def __init__(self, **kw):
        for name, make in self.FIELDS:
            setattr(self, name, make())
        for k, v in kw.items():
            setattr(self, k, v)


def _msg(name, fields, consts=None):
    cls = type(name, (Msg,), {"FIELDS": tuple(fields)})
    for k, v in (consts or {}).items():
        setattr(cls, k, v)
    return cls


def _build():
    f0, l0 = (lambda: 0.0), (lambda: [])
    Time = type("Time", (object,), {
        "__init__": lambda self, t=0.0: setattr(self, "t", t),
        "to_sec": lambda self: self.t,
        "now": classmethod(lambda cls: cls(_clock[0])),
    })
    Header = _msg("Header", [("seq", lambda: 0), ("stamp", lambda: Time()), ("frame_id", lambda: "")])
    Point = _msg("Point", [("x", f0), ("y", f0), ("z", f0)])
    Vector3 = _msg("Vector3", [("x", f0), ("y", f0), ("z", f0)])
    Quaternion = _msg("Quaternion", [("x", f0), ("y", f0), ("z", f0), ("w", f0)])
    Pose = _msg("Pose", [("position", Point), ("orientation", Quaternion)])
    PoseStamped = _msg("PoseStamped", [("header", Header), ("pose", Pose)])
    PoseWithCovariance = _msg("PoseWithCovariance", [("pose", Pose)])
    PoseWithCovarianceStamped = _msg("PoseWithCovarianceStamped",
                                     [("header", Header), ("pose", PoseWithCovariance)])
    Twist = _msg("Twist", [("linear", Vector3), ("angular", Vector3)])
    PoseArray = _msg("PoseArray", [("header", Header), ("poses", l0)])
    Path = _msg("Path", [("header", Header), ("poses", l0)])
    Odometry = _msg("Odometry", [("header", Header), ("pose", PoseWithCovariance)])
    MapMetaData = _msg("MapMetaData", [("resolution", f0), ("width", lambda: 0),
                                       ("height", lambda: 0), ("origin", Pose)])
    OccupancyGrid = _msg("OccupancyGrid", [("header", Header), ("info", MapMetaData), ("data", l0)])
    Imu = _msg("Imu", [("header", Header), ("orientation", Quaternion)])
    LaserScan = _msg("LaserScan", [("header", Header), ("angle_min", f0), ("angle_max", f0),
                                   ("angle_increment", f0), ("range_min", f0), ("range_max", f0),
                                   ("ranges", l0)])
    String = _msg("String", [("data", lambda: "")])
    Empty = _msg("Empty", [])
    Float64 = _msg("Float64", [("data", f0)])
    ColorRGBA = _msg("ColorRGBA", [("r", f0), ("g", f0), ("b", f0), ("a", f0)])
    Marker = _msg("Marker", [("header", Header), ("ns", lambda: ""), ("id", lambda: 0),
                             ("type", lambda: 0), ("action", lambda: 0), ("pose", Pose),
                             ("scale", Vector3), ("color", ColorRGBA), ("points", l0),
                             ("text", lambda: "")],
                  dict(ADD=0, DELETEALL=3, CYLINDER=3, CUBE_LIST=6, LINE_STRIP=4,
                       TEXT_VIEW_FACING=9))
    MarkerArray = _msg("MarkerArray", [("markers", l0)])
    CompressedImage = _msg("CompressedImage", [("header", Header), ("format", lambda: ""),
                                               ("data", lambda: b"")])
    return dict(Time=Time, Header=Header, Point=Point, Pose=Pose, PoseStamped=PoseStamped,
                PoseWithCovarianceStamped=PoseWithCovarianceStamped, Twist=Twist,
                PoseArray=PoseArray, Path=Path, Odometry=Odometry, OccupancyGrid=OccupancyGrid,
                Imu=Imu, LaserScan=LaserScan, String=String, Empty=Empty, Float64=Float64,
                Marker=Marker, MarkerArray=MarkerArray, Quaternion=Quaternion,
                CompressedImage=CompressedImage)


_clock = [0.0]


class _Publisher(object):
    def __init__(self, rospy, topic, *a, **kw):
        self.rospy, self.topic = rospy, topic
        self.last = None

    def publish(self, msg):
        self.last = msg
        self.rospy.PUBLISHED.setdefault(self.topic, []).append(msg)
        del self.rospy.PUBLISHED[self.topic][:-50]


def install():
    if "rospy" in sys.modules and getattr(sys.modules["rospy"], "IS_STUB", False):
        return sys.modules["rospy"]
    m = _build()
    rospy = types.ModuleType("rospy")
    rospy.IS_STUB = True
    rospy.PARAMS, rospy.PUBLISHED, rospy.SUBSCRIBERS, rospy.LOG = {}, {}, {}, []
    rospy.Time = m["Time"]

    def key(name):
        return name.lstrip("~/")

    def get_param(name, default=None):
        if key(name) in rospy.PARAMS:
            return rospy.PARAMS[key(name)]
        if default is None:
            raise KeyError(name)
        return default

    rospy.get_param = get_param
    rospy.has_param = lambda name: key(name) in rospy.PARAMS
    rospy.Subscriber = lambda topic, typ, cb, **kw: rospy.SUBSCRIBERS.__setitem__(key(topic), cb)
    rospy.Publisher = lambda topic, typ, *a, **kw: _Publisher(rospy, key(topic))
    rospy.init_node = lambda *a, **kw: None
    rospy.on_shutdown = lambda f: None
    rospy.is_shutdown = lambda: False
    rospy.Rate = lambda hz: types.SimpleNamespace(sleep=lambda: None)

    def log(level):
        def f(msg, *args):
            rospy.LOG.append((level, (msg % args) if args else msg))
            del rospy.LOG[:-2000]

        def throttled(period, msg, *args):
            f(msg, *args)
        return f, throttled

    for level in ("info", "warn", "err"):
        f, t = log(level)
        setattr(rospy, "log" + level, f)
        setattr(rospy, "log%s_throttle" % level, t)
    sys.modules["rospy"] = rospy

    def pkg(name, **classes):
        mod = types.ModuleType(name)
        for k, v in classes.items():
            setattr(mod, k, v)
        sys.modules[name] = mod
        return mod

    for base in ("geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "visualization_msgs"):
        pkg(base)
    pkg("geometry_msgs.msg", Point=m["Point"], PoseArray=m["PoseArray"], PoseStamped=m["PoseStamped"],
        PoseWithCovarianceStamped=m["PoseWithCovarianceStamped"], Twist=m["Twist"], Pose=m["Pose"])
    pkg("nav_msgs.msg", OccupancyGrid=m["OccupancyGrid"], Odometry=m["Odometry"], Path=m["Path"])
    pkg("sensor_msgs.msg", Imu=m["Imu"], LaserScan=m["LaserScan"],
        CompressedImage=m["CompressedImage"])
    pkg("std_msgs.msg", Empty=m["Empty"], Float64=m["Float64"], String=m["String"])
    pkg("visualization_msgs.msg", Marker=m["Marker"], MarkerArray=m["MarkerArray"])
    rospy.msgs = m
    return rospy


def set_clock(t):
    _clock[0] = t
