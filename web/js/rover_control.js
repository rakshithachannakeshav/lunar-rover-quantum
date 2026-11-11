/*
 * Web control for the real Arjuna rover (robot/arjuna_quantum_nav).
 *
 * Talks to rosbridge (ws://<jetson>:9090) with the plain rosbridge v2 JSON
 * protocol, so it needs no library. Start the rover with:
 *   roslaunch arjuna_quantum_nav quantum_nav.launch web:=true
 *
 * Goal mode : one point  -> geometry_msgs/PoseStamped on /move_base_simple/goal
 *             (the rover's planner picks the route)
 * Route mode: N points   -> nav_msgs/Path on /quantum_nav/route
 *             (the rover visits each point in order; each leg is planned)
 * STOP      : std_msgs/Empty on /quantum_nav/cancel, plus zero /cmd_vel
 * Terrain   : /quantum_nav/terrain (JSON String, latched) = craters (circles) and rock /
 *             border / no-floor cells the depth camera found, in the map frame. Drawn on the
 *             map; a goal or route point placed on one is refused.
 */
(function () {
  "use strict";

  // ── settings (remembered per browser) ──────────────────────────────────────
  var FIELDS = ["url", "t-pose", "t-pose-type", "t-scan", "t-yaw", "t-map", "t-goal", "t-ns", "t-frame"];
  function $(id) { return document.getElementById(id); }
  // the laser offset got a new default (180); a "0" saved by an older page must not override it
  function storeKey(id) { return "rover." + id + (id === "t-yaw" ? ".v2" : ""); }
  function load() {
    FIELDS.forEach(function (id) {
      try {
        var v = localStorage.getItem(storeKey(id));
        if (v !== null) $(id).value = v;
      } catch (e) { /* storage blocked: keep defaults */ }
    });
  }
  function save() {
    FIELDS.forEach(function (id) {
      try { localStorage.setItem(storeKey(id), $(id).value); } catch (e) { /* ignore */ }
    });
  }
  function cfg() {
    var ns = $("t-ns").value.replace(/\/+$/, "");
    return {
      url: $("url").value.trim(),
      pose: $("t-pose").value.trim(),
      poseType: $("t-pose-type").value,
      scan: $("t-scan").value.trim(),
      yawOffset: (parseFloat($("t-yaw").value) || 0) * Math.PI / 180,
      map: $("t-map").value.trim(),
      goal: $("t-goal").value.trim(),
      route: ns + "/route",
      cancel: ns + "/cancel",
      status: ns + "/status",
      path: ns + "/path",
      report: ns + "/report",
      terrain: ns + "/terrain",
      frame: $("t-frame").value.trim() || "map",
    };
  }

  // ── state ─────────────────────────────────────────────────────────────────
  var ws = null, connected = false, wantConnected = false, retryTimer = null;
  var C = cfg();
  var handlers = {};
  var mapInfo = null;      // {w, h, res, ox, oy, img}
  var pose = null;         // {x, y, yaw}
  var scan = null;
  var terrain = null;      // {clearance, craters:[{x,y,r,src}], rock:[[x,y]], border:[], nofloor:[], rim:[]}
  var CELL = 0.04;         // size of one terrain cell, m
  var plannedPath = [];    // [[x, y], ...] path the rover is driving
  var mode = "goal";
  var points = [];         // user's pending points [[x, y], ...]
  var sent = false;        // the points on screen were sent
  var sentFrom = null;     // rover pose when they were sent
  var view = { cx: 0, cy: 0, scale: 80 }; // scale = px per metre
  var follow = false;
  var lastStatusTime = 0;

  // ── rosbridge ─────────────────────────────────────────────────────────────
  function send(obj) {
    if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify(obj)); return true; }
    return false;
  }
  function subscribe(topic, type, throttleMs, fn) {
    handlers[topic] = fn;
    send({ op: "subscribe", topic: topic, type: type, throttle_rate: throttleMs || 0, queue_length: 1 });
  }
  function advertise(topic, type) { send({ op: "advertise", topic: topic, type: type }); }
  function publish(topic, msg) { return send({ op: "publish", topic: topic, msg: msg }); }

  function setConn(ok, text) {
    connected = ok;
    $("conn-dot").className = "dot" + (ok ? " ok" : "");
    $("conn-text").textContent = text;
    $("connect").textContent = wantConnected ? "Disconnect" : "Connect";
    refreshButtons();
  }

  function connect() {
    clearTimeout(retryTimer);
    if (ws) { ws.onclose = null; try { ws.close(); } catch (e) { /* ignore */ } }
    save();
    C = cfg();
    handlers = {};
    setConn(false, "Connecting to " + C.url + " …");
    try {
      ws = new WebSocket(C.url);
    } catch (e) {
      setConn(false, "Bad address: " + e.message);
      return;
    }
    ws.onopen = function () {
      setConn(true, "Connected to " + C.url);
      subscribe(C.pose, C.poseType, 100, onPose);
      subscribe(C.scan, "sensor_msgs/LaserScan", 250, function (m) { scan = m; });
      subscribe(C.map, "nav_msgs/OccupancyGrid", 2000, onMap);
      subscribe(C.status, "std_msgs/String", 200, onStatus);
      subscribe(C.path, "nav_msgs/Path", 500, function (m) {
        plannedPath = (m.poses || []).map(function (p) { return [p.pose.position.x, p.pose.position.y]; });
      });
      subscribe(C.report, "std_msgs/String", 500, onReport);
      subscribe(C.terrain, "std_msgs/String", 300, onTerrain);
      advertise(C.goal, "geometry_msgs/PoseStamped");
      advertise(C.route, "nav_msgs/Path");
      advertise(C.cancel, "std_msgs/Empty");
      advertise("/cmd_vel", "geometry_msgs/Twist");
    };
    ws.onmessage = function (e) {
      var m;
      try { m = JSON.parse(e.data); } catch (err) { return; }
      if (m.op === "publish" && handlers[m.topic]) handlers[m.topic](m.msg);
      else if (m.op === "status" && m.level === "error") console.warn("rosbridge:", m.msg);
    };
    ws.onerror = function () { /* onclose follows */ };
    ws.onclose = function () {
      ws = null;
      if (wantConnected) {
        setConn(false, "Lost connection; retrying in 3 s …");
        retryTimer = setTimeout(connect, 3000);
      } else {
        setConn(false, "Not connected");
      }
    };
  }

  // ── incoming messages ─────────────────────────────────────────────────────
  function yawOf(q) { return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)); }
  function onPose(m) {
    var p = m.pose.pose;  // same shape for PoseWithCovarianceStamped and Odometry
    pose = { x: p.position.x, y: p.position.y, yaw: yawOf(p.orientation) };
    $("pose").textContent = "pose: x " + pose.x.toFixed(2) + "  y " + pose.y.toFixed(2) +
      "  yaw " + (pose.yaw * 180 / Math.PI).toFixed(0) + "°";
    if (follow) { view.cx = pose.x; view.cy = pose.y; }
  }

  function onMap(m) {
    var w = m.info.width, h = m.info.height, data = m.data;
    if (!w || !h) return;
    var off = document.createElement("canvas");
    off.width = w; off.height = h;
    var ctx = off.getContext("2d");
    var im = ctx.createImageData(w, h);
    for (var r = 0; r < h; r++) {
      for (var c = 0; c < w; c++) {
        var v = data[r * w + c];
        var i = ((h - 1 - r) * w + c) * 4;  // row 0 is the bottom of the map
        var R, G, B;
        if (v < 0) { R = 14; G = 16; B = 28; }
        else if (v >= 65) { R = 226; G = 228; B = 236; }
        else if (v <= 25) { R = 40; G = 46; B = 70; }
        else { var t = (v - 25) / 40; R = 40 + t * 186; G = 46 + t * 182; B = 70 + t * 166; }
        im.data[i] = R; im.data[i + 1] = G; im.data[i + 2] = B; im.data[i + 3] = 255;
      }
    }
    ctx.putImageData(im, 0, 0);
    var first = mapInfo === null;
    mapInfo = { w: w, h: h, res: m.info.resolution, ox: m.info.origin.position.x, oy: m.info.origin.position.y, img: off };
    if (first) fit();
  }

  function onStatus(m) {
    lastStatusTime = Date.now();
    var el = $("status");
    el.textContent = m.data;
    el.className = "mono" + (/ABORTED|no path|stale|waiting for pose/i.test(m.data) ? " warn" : "");
  }

  function onTerrain(m) {
    var t;
    try { t = JSON.parse(m.data); } catch (e) { return; }
    terrain = t;
    var n = function (k) { return (t[k] || []).length; };
    $("terrain-summary").textContent = (t.craters || []).length + " crater(s), " +
      n("rock") + " rock cell(s), " + n("border") + " border cell(s), " + n("nofloor") + " edge cell(s)";
    // a goal that was fine when placed may now sit on something the camera found
    if (points.length && !sent) checkPoints(false);
  }

  // What is at this point? Returns "crater" / "rock" / "border wall" / "table edge" or null.
  // Only the object itself counts; the rover's clearance around it is the planner's business.
  function objectAt(pt) {
    if (!terrain) return null;
    var i, d, c, pad = 0.02;
    for (i = 0; i < (terrain.craters || []).length; i++) {
      c = terrain.craters[i];
      if (Math.hypot(pt[0] - c.x, pt[1] - c.y) <= c.r + pad) return "crater";
    }
    var half = CELL * 0.75;
    var kinds = [["rock", "rock"], ["border", "border wall"], ["nofloor", "table edge"], ["rim", "crater"]];
    for (var k = 0; k < kinds.length; k++) {
      var cells = terrain[kinds[k][0]] || [];
      for (i = 0; i < cells.length; i++) {
        if (Math.abs(cells[i][0] - pt[0]) <= half && Math.abs(cells[i][1] - pt[1]) <= half) return kinds[k][1];
      }
    }
    return null;
  }
  function showMsg(text) {
    var el = $("point-msg");
    el.textContent = text || "";
    el.hidden = !text;
  }
  // Remove or refuse points on detected objects. Returns true if all points are fine.
  function checkPoints(refuseOnly) {
    for (var i = 0; i < points.length; i++) {
      var what = objectAt(points[i]);
      if (what) {
        showMsg("Point " + (mode === "goal" ? "" : "#" + (i + 1) + " ") + "(" + points[i][0].toFixed(2) + ", " +
          points[i][1].toFixed(2) + ") is on a detected " + what + " — the rover cannot go there. Move it.");
        return false;
      }
    }
    showMsg("");
    return true;
  }

  function onReport(m) {
    var rep;
    try { rep = JSON.parse(m.data); } catch (e) { return; }
    var rows = "<tr><th>algorithm</th><th>energy</th><th>dist m</th><th>ms</th></tr>";
    Object.keys(rep.results || {}).forEach(function (a) {
      var r = rep.results[a];
      rows += "<tr" + (a === rep.chosen ? ' class="chosen"' : "") + "><td>" + a + (a === rep.chosen ? " ✓" : "") + "</td>" +
        (r ? "<td>" + r.energy.toFixed(2) + "</td><td>" + r.distance_m.toFixed(2) + "</td><td>" + r.runtime_ms.toFixed(0) + "</td>"
           : '<td colspan="3">no path</td>') + "</tr>";
    });
    $("report").innerHTML = rows;
    $("report-box").hidden = false;
  }

  // ── commands ──────────────────────────────────────────────────────────────
  function header() { return { frame_id: C.frame, stamp: { secs: 0, nsecs: 0 } }; }
  function poseMsg(pt) {
    return { header: header(), pose: { position: { x: pt[0], y: pt[1], z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } } };
  }
  function sendPoints() {
    if (!connected || points.length === 0) return;
    if (!checkPoints(true)) return;       // never send a goal or route point that is on a crater / rock
    var ok;
    if (mode === "goal") ok = publish(C.goal, poseMsg(points[0]));
    else ok = publish(C.route, { header: header(), poses: points.map(poseMsg) });
    if (ok) { sent = true; sentFrom = pose; plannedPath = []; renderPoints(); }
  }
  function stop() {
    publish(C.cancel, {});
    var zero = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } };
    for (var i = 0; i < 3; i++) publish("/cmd_vel", zero);
    var el = $("status");
    el.textContent = connected ? "STOP sent" : "not connected: STOP could not be sent";
    el.className = "mono warn";
  }

  // ── UI ────────────────────────────────────────────────────────────────────
  function setMode(m) {
    mode = m;
    $("mode-goal").className = m === "goal" ? "on" : "";
    $("mode-route").className = m === "route" ? "on" : "";
    $("points-title").textContent = m === "goal" ? "Goal" : "Route points";
    $("mode-help").textContent = m === "goal"
      ? "Click the map to place one goal. The rover's planner chooses the route."
      : "Click the map to add points in order. The rover drives through each one; between points it plans round mapped obstacles.";
    if (m === "goal" && points.length > 1) points = [points[points.length - 1]];
    sent = false;
    renderPoints();
  }
  function addPoint(pt) {
    var what = objectAt(pt);
    if (what) {                           // refuse: do not place goal / route points on obstacles
      showMsg("Not placed: (" + pt[0].toFixed(2) + ", " + pt[1].toFixed(2) + ") is on a detected " + what +
        ". Pick a spot on open floor.");
      return;
    }
    showMsg("");
    if (sent) { points = []; sent = false; }
    if (mode === "goal") points = [pt]; else points.push(pt);
    renderPoints();
  }
  function renderPoints() {
    var ul = $("points");
    if (points.length === 0) showMsg("");
    ul.innerHTML = "";
    if (points.length === 0) {
      var li = document.createElement("li");
      li.className = "muted";
      li.textContent = "Click the map to place " + (mode === "goal" ? "a goal" : "route points");
      ul.appendChild(li);
    }
    points.forEach(function (p, i) {
      var li = document.createElement("li");
      li.innerHTML = "<span>" + (mode === "goal" ? "goal" : "#" + (i + 1)) + "</span><span>" +
        p[0].toFixed(2) + ", " + p[1].toFixed(2) + "</span>";
      ul.appendChild(li);
    });
    if (sent) {
      var s = document.createElement("li");
      s.innerHTML = '<span style="color:var(--success)">sent ✓</span><span></span>';
      ul.appendChild(s);
    }
    refreshButtons();
  }
  function refreshButtons() {
    $("send").disabled = !connected || points.length === 0;
    $("undo").disabled = points.length === 0;
    $("clear").disabled = points.length === 0;
  }

  // ── view ──────────────────────────────────────────────────────────────────
  var canvas = $("map"), ctx = canvas.getContext("2d"), W = 0, H = 0, dpr = 1;
  function resize() {
    dpr = window.devicePixelRatio || 1;
    W = window.innerWidth; H = window.innerHeight;
    canvas.width = W * dpr; canvas.height = H * dpr;
  }
  function sx(x) { return W / 2 + (x - view.cx) * view.scale; }
  function sy(y) { return H / 2 - (y - view.cy) * view.scale; }
  function wx(px) { return view.cx + (px - W / 2) / view.scale; }
  function wy(py) { return view.cy - (py - H / 2) / view.scale; }
  function zoomAt(f, px, py) {
    var x = wx(px), y = wy(py);
    view.scale = Math.max(5, Math.min(800, view.scale * f));
    view.cx = x - (px - W / 2) / view.scale;
    view.cy = y + (py - H / 2) / view.scale;
  }
  function fit() {
    var x0, x1, y0, y1;
    if (mapInfo) {
      x0 = mapInfo.ox; x1 = mapInfo.ox + mapInfo.w * mapInfo.res;
      y0 = mapInfo.oy; y1 = mapInfo.oy + mapInfo.h * mapInfo.res;
    } else {
      var c = pose || { x: 0, y: 0 };
      x0 = c.x - 3; x1 = c.x + 3; y0 = c.y - 3; y1 = c.y + 3;
    }
    view.cx = (x0 + x1) / 2; view.cy = (y0 + y1) / 2;
    view.scale = Math.max(5, Math.min(800, 0.9 * Math.min(W / (x1 - x0), H / (y1 - y0))));
  }

  // ── drawing ───────────────────────────────────────────────────────────────
  function drawCells(cells, fill) {
    if (!cells || !cells.length) return;
    var px = Math.max(2, CELL * view.scale);
    ctx.fillStyle = fill;
    for (var i = 0; i < cells.length; i++) {
      ctx.fillRect(sx(cells[i][0]) - px / 2, sy(cells[i][1]) - px / 2, px, px);
    }
  }
  function drawTerrain() {
    if (!terrain) return;
    // no floor (table edge) and border wall: flat dark / purple cells
    drawCells(terrain.nofloor, "rgba(10,10,16,0.9)");
    drawCells(terrain.border, "rgba(168,85,247,0.85)");
    // rocks: solid red blobs (crumpled plastic)
    drawCells(terrain.rock, "rgba(239,68,68,0.85)");
    // craters: a dashed ring with a translucent floor, so they never look like rocks
    (terrain.craters || []).forEach(function (c) {
      ctx.beginPath(); ctx.arc(sx(c.x), sy(c.y), c.r * view.scale, 0, 2 * Math.PI);
      ctx.fillStyle = "rgba(251,146,60,0.22)"; ctx.fill();
      ctx.setLineDash([7, 5]); ctx.lineWidth = 3; ctx.strokeStyle = "#fb923c"; ctx.stroke();
      ctx.setLineDash([]);
      var k = Math.min(8, c.r * view.scale * 0.3);
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.moveTo(sx(c.x) - k, sy(c.y)); ctx.lineTo(sx(c.x) + k, sy(c.y));
      ctx.moveTo(sx(c.x), sy(c.y) - k); ctx.lineTo(sx(c.x), sy(c.y) + k); ctx.stroke();
    });
    // rim arcs seen but not (yet) a full circle
    drawCells(terrain.rim, "rgba(251,146,60,0.85)");
  }

  function draw() {
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#04040a";
    ctx.fillRect(0, 0, W, H);

    if (mapInfo) {
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(mapInfo.img, sx(mapInfo.ox), sy(mapInfo.oy + mapInfo.h * mapInfo.res),
        mapInfo.w * mapInfo.res * view.scale, mapInfo.h * mapInfo.res * view.scale);
    }

    // metre grid (step grows when zoomed out)
    var step = 1;
    while (step * view.scale < 40) step *= 2;
    ctx.strokeStyle = "rgba(255,255,255,0.05)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    for (var gx = Math.floor(wx(0) / step) * step; gx <= wx(W); gx += step) { ctx.moveTo(sx(gx), 0); ctx.lineTo(sx(gx), H); }
    for (var gy = Math.floor(wy(H) / step) * step; gy <= wy(0); gy += step) { ctx.moveTo(0, sy(gy)); ctx.lineTo(W, sy(gy)); }
    ctx.stroke();
    // origin axes
    ctx.strokeStyle = "rgba(255,255,255,0.18)";
    ctx.beginPath(); ctx.moveTo(sx(0), 0); ctx.lineTo(sx(0), H); ctx.moveTo(0, sy(0)); ctx.lineTo(W, sy(0)); ctx.stroke();

    drawTerrain();

    // lidar hits
    if (scan && pose) {
      ctx.fillStyle = "#f87171";
      var a = scan.angle_min;
      for (var i = 0; i < scan.ranges.length; i++, a += scan.angle_increment) {
        var r = scan.ranges[i];
        if (typeof r !== "number" || !isFinite(r) || r <= scan.range_min || r >= scan.range_max) continue;
        var ang = pose.yaw + C.yawOffset + a;
        ctx.fillRect(sx(pose.x + r * Math.cos(ang)) - 1.5, sy(pose.y + r * Math.sin(ang)) - 1.5, 3, 3);
      }
    }

    // path the rover is driving
    if (plannedPath.length > 1) {
      ctx.strokeStyle = "#60a5fa"; ctx.lineWidth = 3; ctx.setLineDash([]);
      ctx.beginPath();
      plannedPath.forEach(function (p, k) { if (k) ctx.lineTo(sx(p[0]), sy(p[1])); else ctx.moveTo(sx(p[0]), sy(p[1])); });
      ctx.stroke();
    }

    // user's points
    if (points.length) {
      var col = sent ? "#34d399" : "#f59e0b";
      ctx.strokeStyle = col; ctx.lineWidth = 2; ctx.setLineDash([6, 5]);
      ctx.beginPath();
      var from = sent ? sentFrom : pose;
      if (from) ctx.moveTo(sx(from.x), sy(from.y)); else ctx.moveTo(sx(points[0][0]), sy(points[0][1]));
      points.forEach(function (p) { ctx.lineTo(sx(p[0]), sy(p[1])); });
      ctx.stroke();
      ctx.setLineDash([]);
      points.forEach(function (p, k) {
        ctx.fillStyle = col;
        ctx.beginPath(); ctx.arc(sx(p[0]), sy(p[1]), 9, 0, 2 * Math.PI); ctx.fill();
        ctx.fillStyle = "#1a1205"; ctx.font = "bold 11px system-ui"; ctx.textAlign = "center"; ctx.textBaseline = "middle";
        ctx.fillText(mode === "goal" ? "G" : String(k + 1), sx(p[0]), sy(p[1]) + 0.5);
      });
    }

    // rover
    if (pose) {
      var s = Math.max(12, 0.3 * view.scale);
      ctx.save();
      ctx.translate(sx(pose.x), sy(pose.y));
      ctx.rotate(-pose.yaw);
      ctx.fillStyle = "#dde1ee"; ctx.strokeStyle = "#04040a"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(s * 0.6, 0); ctx.lineTo(-s * 0.4, s * 0.4); ctx.lineTo(-s * 0.2, 0); ctx.lineTo(-s * 0.4, -s * 0.4); ctx.closePath();
      ctx.fill(); ctx.stroke();
      ctx.restore();
    } else if (connected) {
      ctx.fillStyle = "#8a90ad"; ctx.font = "13px system-ui"; ctx.textAlign = "center";
      ctx.fillText("Waiting for the rover pose on " + C.pose + " …", W / 2, H / 2);
    }

    // stale status = node not running
    if (connected && lastStatusTime && Date.now() - lastStatusTime > 3000) {
      $("status").textContent = "no status for " + ((Date.now() - lastStatusTime) / 1000).toFixed(0) + " s: is quantum_nav running?";
      $("status").className = "mono warn";
    }
    requestAnimationFrame(draw);
  }

  // ── input: click places, drag pans, wheel zooms ───────────────────────────
  var down = null;
  canvas.addEventListener("pointerdown", function (e) {
    down = { x: e.clientX, y: e.clientY, cx: view.cx, cy: view.cy, moved: false };
    canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener("pointermove", function (e) {
    $("hover").textContent = "x " + wx(e.clientX).toFixed(2) + ", y " + wy(e.clientY).toFixed(2);
    if (!down) return;
    var dx = e.clientX - down.x, dy = e.clientY - down.y;
    if (!down.moved && dx * dx + dy * dy > 25) { down.moved = true; setFollow(false); }
    if (down.moved) { view.cx = down.cx - dx / view.scale; view.cy = down.cy + dy / view.scale; }
  });
  canvas.addEventListener("pointerup", function (e) {
    if (down && !down.moved && e.button === 0) addPoint([wx(e.clientX), wy(e.clientY)]);
    down = null;
  });
  canvas.addEventListener("wheel", function (e) {
    e.preventDefault();
    zoomAt(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.clientX, e.clientY);
  }, { passive: false });

  function setFollow(on) { follow = on; $("follow").className = on ? "on" : ""; if (on && pose) { view.cx = pose.x; view.cy = pose.y; } }

  $("connect").onclick = function () {
    if (wantConnected) {
      wantConnected = false;
      clearTimeout(retryTimer);
      if (ws) ws.close(); else setConn(false, "Not connected");
    } else {
      wantConnected = true;
      connect();
    }
  };
  $("mode-goal").onclick = function () { setMode("goal"); };
  $("mode-route").onclick = function () { setMode("route"); };
  $("undo").onclick = function () { points.pop(); sent = false; renderPoints(); };
  $("clear").onclick = function () { points = []; sent = false; renderPoints(); };
  $("send").onclick = sendPoints;
  $("stop").onclick = stop;
  $("zoom-in").onclick = function () { zoomAt(1.3, W / 2, H / 2); };
  $("zoom-out").onclick = function () { zoomAt(1 / 1.3, W / 2, H / 2); };
  $("follow").onclick = function () { setFollow(!follow); };
  $("fit").onclick = function () { setFollow(false); fit(); };
  window.addEventListener("keydown", function (e) {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    if (e.code === "Space") { e.preventDefault(); stop(); }
    else if (e.key === "Enter") sendPoints();
    else if (e.key === "Backspace" || (e.key === "z" && (e.ctrlKey || e.metaKey))) { e.preventDefault(); $("undo").onclick(); }
  });
  window.addEventListener("resize", resize);

  load();
  resize();
  setMode("goal");
  setConn(false, "Not connected");
  requestAnimationFrame(draw);
})();
