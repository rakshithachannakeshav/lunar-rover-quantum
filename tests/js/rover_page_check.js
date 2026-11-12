// Drives web/js/rover_control.js against a tiny fake DOM + WebSocket (no browser needed).
// Run by tests/test_rover_web.py:  node tests/js/rover_page_check.js
"use strict";
const fs = require("fs");
const path = require("path");
const root = path.join(__dirname, "..", "..", "web");
const html = fs.readFileSync(path.join(root, "rover.html"), "utf8");
const code = fs.readFileSync(path.join(root, "js", "rover_control.js"), "utf8");

// ---- fake DOM: every id in the HTML gets an element; <input value="..."> defaults are honoured
const defaults = {};
html.replace(/<(?:input|select)[^>]*id="([^"]+)"[^>]*?value="([^"]*)"/g, (m, id, v) => { defaults[id] = v; return m; });
const els = {};
function makeEl(id) {
  const el = {
    id, value: defaults[id] !== undefined ? defaults[id] : "", textContent: "", className: "", hidden: false,
    _html: "", children: [], style: {}, handlers: {}, disabled: false,
    // like a browser: replacing the HTML replaces the children
    set innerHTML(v) { this._html = v; if (v === "") this.children = []; }, get innerHTML() { return this._html; },
    appendChild(c) { this.children.push(c); return c; },
    addEventListener(t, f) { this.handlers[t] = f; },
    setPointerCapture() {},
    getContext() { return new Proxy({}, { get: (t, k) => (k in t ? t[k] : () => ({ data: [] })), set: (t, k, v) => { t[k] = v; return true; } }); },
    set onclick(f) { this._onclick = f; }, get onclick() { return this._onclick; },
  };
  return el;
}
const document = {
  getElementById(id) { return els[id] || (els[id] = makeEl(id)); },
  createElement() { return makeEl("_"); },
};
const store = {};
const sockets = [];
class FakeWS {
  constructor(url) { this.url = url; this.sent = []; this.readyState = 1; sockets.push(this); }
  send(s) { this.sent.push(JSON.parse(s)); }
  close() { this.readyState = 3; if (this.onclose) this.onclose(); }
}
FakeWS.OPEN = 1;
const winHandlers = {};
global.document = document;
global.window = { innerWidth: 1000, innerHeight: 800, devicePixelRatio: 1, addEventListener(t, f) { winHandlers[t] = f; } };
global.localStorage = { getItem: (k) => (k in store ? store[k] : null), setItem: (k, v) => { store[k] = String(v); } };
global.WebSocket = FakeWS;
global.requestAnimationFrame = () => {};
global.setTimeout = () => 0;
global.clearTimeout = () => {};

let failures = 0;
function check(cond, what) { if (!cond) { failures++; console.log("FAIL: " + what); } else { console.log("ok:   " + what); } }

// an old page stored "0" for the laser offset: it must NOT override the new default
store["rover.t-yaw"] = "0";
// pre-create elements so the defaults are read before the script runs
Object.keys(defaults).forEach((id) => document.getElementById(id));
check(defaults["t-yaw"] === "180", "HTML default for the laser yaw offset is 180");
eval(code);                                    // runs the page script
check(document.getElementById("t-yaw").value === "180", "a stale saved 0 does not override the 180 default");

// connect, then feed a terrain message
document.getElementById("connect").onclick();
const ws = sockets[sockets.length - 1];
ws.onopen();
const subs = ws.sent.filter((m) => m.op === "subscribe").map((m) => m.topic);
check(subs.indexOf("/quantum_nav/terrain") >= 0, "subscribes to /quantum_nav/terrain");
const terrain = { clearance: 0.14, version: 1,
  craters: [{ x: 1.0, y: 1.0, r: 0.2, src: "camera" }],
  rock: [[0.5, 0.5], [0.54, 0.5], [0.5, 0.54]], border: [[2.0, 0.1]], nofloor: [[0.0, 2.3]], rim: [] };
ws.onmessage({ data: JSON.stringify({ op: "publish", topic: "/quantum_nav/terrain", msg: { data: JSON.stringify(terrain) } }) });
check(/1 crater/.test(document.getElementById("terrain-summary").textContent), "terrain summary shows the crater");

// view: centre (0,0), 80 px per metre, canvas 1000x800 -> world (x, y) is at px (500 + 80x, 400 - 80y)
const canvas = document.getElementById("map");
function click(x, y) {
  const e = { clientX: 500 + 80 * x, clientY: 400 - 80 * y, button: 0, pointerId: 1 };
  canvas.handlers.pointerdown(e);
  canvas.handlers.pointerup(e);
}
const msg = () => document.getElementById("point-msg");
const lastPublish = (topic) => ws.sent.filter((m) => m.op === "publish" && m.topic === topic).pop();

click(1.05, 0.95);                                         // inside the crater
check(!msg().hidden && /crater/.test(msg().textContent), "click inside a crater is refused with a message");
document.getElementById("send").onclick();
check(!lastPublish("/move_base_simple/goal"), "nothing is sent for a goal on a crater");

click(0.52, 0.5);                                          // on a rock cell
check(/rock/.test(msg().textContent), "click on a rock is refused with a message");
click(0.0, 2.3);
check(/table edge/.test(msg().textContent), "click on the table edge (no floor) is refused");

click(0.3, 0.3);                                           // open floor
check(msg().hidden, "click on open floor is accepted (message cleared)");
document.getElementById("send").onclick();
const goal = lastPublish("/move_base_simple/goal");
check(goal && Math.abs(goal.msg.pose.position.x - 0.3) < 0.02, "goal on open floor is published");

// route mode: points placed on open floor, then a rock is detected under one of them
document.getElementById("mode-route").onclick();
document.getElementById("clear").onclick();
click(1.5, 0.3);
click(1.5, 1.5);
const terrain2 = JSON.parse(JSON.stringify(terrain));
terrain2.rock.push([1.5, 1.5]);
ws.onmessage({ data: JSON.stringify({ op: "publish", topic: "/quantum_nav/terrain", msg: { data: JSON.stringify(terrain2) } }) });
check(!msg().hidden && /#2/.test(msg().textContent), "a route point that a new rock covers is flagged by number");
const before = ws.sent.length;
document.getElementById("send").onclick();
check(!ws.sent.slice(before).some((m) => m.op === "publish" && m.topic === "/quantum_nav/route"), "the route is not sent while a point is on a rock");

// ---- objects with sizes (/terrain/objects) -------------------------------------------------
check(subs.indexOf("/terrain/objects") >= 0, "subscribes to /terrain/objects");
check(subs.indexOf("/terrain/preview/compressed") >= 0, "subscribes to the colour preview");
const feed = (topic, msg) => ws.onmessage({ data: JSON.stringify({ op: "publish", topic, msg }) });
const rock = { id: 2, class: "rock", x: 1.2, y: 0.4, yaw: 0, length_m: 0.31, width_m: 0.27, height_m: 0.24,
  height_is_min: false, footprint_complete: true, size_is_estimate: false, inner_diameter_m: null,
  outer_diameter_m: null, distance_m: 0.82, centre_distance_m: 0.97, bearing_deg: 3, frames_seen: 9,
  confidence: 0.9, visible: true, source: "depth",
  outline: [[1.355, 0.535], [1.045, 0.535], [1.045, 0.265], [1.355, 0.265]] };
const crater = { id: 1, class: "crater", x: 2.0, y: 1.0, yaw: 0, length_m: 0.34, width_m: 0.34, height_m: 0.02,
  height_is_min: false, footprint_complete: false, size_is_estimate: false, inner_diameter_m: 0.29,
  outer_diameter_m: 0.34, distance_m: 1.9, centre_distance_m: 2.07, bearing_deg: 20, frames_seen: 4,
  confidence: 0.7, visible: false, source: "depth", outline: [[2.17, 1.0], [2.0, 1.17], [1.83, 1.0], [2.0, 0.83]] };
feed("/terrain/objects", { data: JSON.stringify({ frame_id: "map", stamp: 1, rover: null, objects: [crater, rock] }) });
const list = () => document.getElementById("objects").children;
check(list().length === 2, "both objects are listed");
check(/rock #2/.test(list()[0].innerHTML) && /0\.82 m/.test(list()[0].innerHTML), "nearest object first, with its distance");
check(/L 31 × W 27 × H 24 cm/.test(list()[0].innerHTML), "rock row shows length, width and height");
check(/34 cm across, inner 29 cm, rim 2\.0 cm/.test(list()[1].innerHTML), "crater row shows outer / inner diameter and rim height");
check(/not in view/.test(list()[1].innerHTML) && /gone/.test(list()[1].className), "an object out of view is marked");
check(window.roverDebug.label(rock) === "rock #2 31×27×24 cm", "map label: class, id and size");
check(window.roverDebug.label(Object.assign({}, rock, { height_is_min: true })) === "rock #2 31×27×≥24 cm",
  "a height the camera could not see the top of is shown as a minimum");
list()[0].onclick();
check(window.roverDebug.selected() === 2, "clicking a row selects that object");
check(/sel/.test(list()[0].className), "the selected row is highlighted");
let drew = true;
try { window.roverDebug.draw(); } catch (e) { drew = false; console.log(e); }
check(drew, "objects (with a highlighted one) draw without errors");
list()[0].onclick();
check(window.roverDebug.selected() === null, "clicking it again clears the highlight");
// the selected object disappears from the list: the highlight goes too
list()[1].onclick();
feed("/terrain/objects", { data: JSON.stringify({ frame_id: "map", stamp: 2, rover: null, objects: [rock] }) });
check(window.roverDebug.selected() === null && list().length === 1, "a vanished object loses its highlight");

// robot-frame objects (detector without a pose) are placed with the rover pose
feed("/robot_pose_ekf/odom_combined", { pose: { pose: { position: { x: 1, y: 1, z: 0 }, orientation: { x: 0, y: 0, z: Math.sin(Math.PI / 4), w: Math.cos(Math.PI / 4) } } } });
feed("/terrain/objects", { data: JSON.stringify({ frame_id: "base_footprint", stamp: 3, rover: null,
  objects: [Object.assign({}, rock, { x: 0.5, y: 0.0, outline: [[0.6, 0.1], [0.4, 0.1], [0.4, -0.1], [0.6, -0.1]] })] }) });
const placed = window.roverDebug.objects()[0];
check(Math.abs(placed.x - 1.0) < 1e-6 && Math.abs(placed.y - 1.5) < 1e-6, "robot-frame object moved to the map with the pose");

// colour preview
feed("/terrain/preview/compressed", { format: "jpeg", data: "QUJD" });
check(document.getElementById("preview").src === "data:image/jpeg;base64,QUJD" &&
  document.getElementById("preview-box").hidden === false, "preview image is shown");

process.exit(failures ? 1 : 0);
