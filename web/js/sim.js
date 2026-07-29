/* Lunar Rover Web Sim — Fixed: no bounce, smooth movement, proper physics */
(function () {
  "use strict";

  var errEl = document.getElementById("load-error");
  if (typeof THREE === "undefined") { if (errEl) errEl.hidden = false; return; }
  if (errEl) errEl.hidden = true;

  // ── constants ──────────────────────────────────────────────────────────────
  var GOAL_X = 8, GOAL_Z = -4;
  var PATROL_WPS = [
    [4, 0], [8, -4], [6, -8], [0, -6], [-4, -2], [0, 4], [4, 0]
  ];

  // ── state ──────────────────────────────────────────────────────────────────
  var keys = {};
  var mode = "manual";
  var maxSpeed = 0.5;
  var camMode = "chase";
  var patrolIdx = 0;

  // rover 2D world coords (x, z in THREE space)
  var rv = {
    wx: -3, wz: 3,         // world X and Z
    wy: 0,                  // smoothed Y (visual only)
    yaw: 0,
    speed: 0,               // forward speed m/s
    turn: 0,                // turn rate rad/s
    wheelAngle: 0,
    group: null,
    wheels: []
  };

  // orbit state
  var orbit = { active: false, startX: 0, startY: 0, azimuth: Math.PI, elevation: 0.35, radius: 10 };

  // ── DOM ────────────────────────────────────────────────────────────────────
  var wrap = document.getElementById("canvas-wrap");
  var elSpeed = document.getElementById("stat-speed");
  var elHead = document.getElementById("stat-heading");
  var elPos = document.getElementById("stat-pos");
  var elGoal = document.getElementById("stat-goal");
  var elStatus = document.getElementById("status-msg");
  var elBattery = document.getElementById("stat-battery");
  var elGrav = document.getElementById("stat-grav");
  var elTemp = document.getElementById("stat-temp");

  // ── THREE setup ────────────────────────────────────────────────────────────
  var renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.1;
  wrap.appendChild(renderer.domElement);

  var scene = new THREE.Scene();
  scene.background = new THREE.Color(0x04040a);
  scene.fog = new THREE.FogExp2(0x04040a, 0.007);

  var camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.05, 500);
  camera.position.set(-8, 5, 10);

  // lights
  scene.add(new THREE.AmbientLight(0x334466, 0.6));
  var sun = new THREE.DirectionalLight(0xfff0cc, 2.2);
  sun.position.set(-40, 60, 20);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 200;
  sun.shadow.camera.left = -50;
  sun.shadow.camera.right = 50;
  sun.shadow.camera.top = 50;
  sun.shadow.camera.bottom = -50;
  sun.shadow.bias = -0.001;
  scene.add(sun);

  var fill = new THREE.DirectionalLight(0x3355aa, 0.3);
  fill.position.set(30, 20, -30);
  scene.add(fill);

  // stars
  (function buildStars() {
    var sg = new THREE.BufferGeometry();
    var pos = [];
    for (var i = 0; i < 3000; i++) {
      var r = 200 + Math.random() * 100;
      var theta = Math.random() * Math.PI * 2;
      var phi = Math.acos(2 * Math.random() - 1);
      pos.push(r * Math.sin(phi) * Math.cos(theta), r * Math.cos(phi), r * Math.sin(phi) * Math.sin(theta));
    }
    sg.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    scene.add(new THREE.Points(sg, new THREE.PointsMaterial({ color: 0xffffff, size: 0.5, sizeAttenuation: true })));
  })();

  // ── terrain height function ────────────────────────────────────────────────
  // Uses smooth noise — NO high-frequency oscillation that causes bobbing
  function heightAt(x, z) {
    var h = 0;
    // large gentle hills only
    h += 0.4 * Math.sin(x * 0.04 + 0.3) * Math.cos(z * 0.05 + 0.7);
    h += 0.25 * Math.sin(x * 0.07 + 1.1) * Math.sin(z * 0.06 + 0.4);
    h += 0.12 * Math.sin(x * 0.13 + 2.0) * Math.cos(z * 0.09 + 1.5);
    // crater near (12, 10)
    var cd = Math.sqrt((x - 12) * (x - 12) + (z - 10) * (z - 10));
    if (cd < 7) h -= 0.9 * Math.pow(Math.max(0, 1 - cd / 7), 2);
    return h;
  }

  // ── terrain mesh ───────────────────────────────────────────────────────────
  var seg = 128;
  var tGeo = new THREE.PlaneGeometry(200, 200, seg, seg);
  tGeo.rotateX(-Math.PI / 2);
  var tv = tGeo.attributes.position;
  for (var i = 0; i < tv.count; i++) {
    tv.setY(i, heightAt(tv.getX(i), tv.getZ(i)));
  }
  tGeo.computeVertexNormals();

  // lunar regolith color
  var tMat = new THREE.MeshStandardMaterial({
    color: 0x9a927a, roughness: 0.95, metalness: 0.02
  });
  var terrain = new THREE.Mesh(tGeo, tMat);
  terrain.receiveShadow = true;
  scene.add(terrain);

  // ── rocks ──────────────────────────────────────────────────────────────────
  var rockMat = new THREE.MeshStandardMaterial({ color: 0x5a5248, roughness: 0.9, metalness: 0.05 });
  function addRock(x, z, rx, ry, rz) {
    var g = new THREE.DodecahedronGeometry(0.4 + Math.random() * 0.5, 0);
    var m = new THREE.Mesh(g, rockMat);
    m.scale.set(rx, ry, rz);
    var y = heightAt(x, z);
    m.position.set(x, y + ry * 0.3, z);
    m.rotation.set(Math.random(), Math.random() * 6, Math.random());
    m.castShadow = true;
    scene.add(m);
  }
  var rockDefs = [
    [3, 1, 1.8, 0.9, 1.4], [-2, 4, 1.2, 0.8, 1.1], [6, -2, 2.2, 1.1, 1.9],
    [-5, -3, 1.5, 1.0, 1.3], [9, 5, 1.1, 0.7, 1.0], [-8, 7, 2.0, 1.3, 1.8],
    [4, -7, 1.3, 0.9, 1.2], [-3, -8, 1.6, 1.1, 1.4], [7, 2, 1.0, 0.8, 1.1],
    [-6, 2, 1.4, 0.7, 1.2], [2, 8, 1.8, 1.0, 1.5], [-9, -5, 1.2, 0.6, 1.0]
  ];
  rockDefs.forEach(function (r) { addRock(r[0], r[1], r[2], r[3], r[4]); });

  // ── goal marker ────────────────────────────────────────────────────────────
  var goalGroup = new THREE.Group();
  var goalDisc = new THREE.Mesh(
    new THREE.CylinderGeometry(0.7, 0.7, 0.06, 32),
    new THREE.MeshStandardMaterial({ color: 0xff6600, emissive: 0xff3300, emissiveIntensity: 0.5, roughness: 0.4 })
  );
  goalGroup.add(goalDisc);
  var flagPole = new THREE.Mesh(
    new THREE.CylinderGeometry(0.03, 0.03, 2.0, 8),
    new THREE.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.8, roughness: 0.2 })
  );
  flagPole.position.y = 1.0;
  goalGroup.add(flagPole);
  var flag = new THREE.Mesh(
    new THREE.PlaneGeometry(0.6, 0.35),
    new THREE.MeshStandardMaterial({ color: 0xff6600, side: THREE.DoubleSide, emissive: 0xff3300, emissiveIntensity: 0.3 })
  );
  flag.position.set(0.3, 1.8, 0);
  goalGroup.add(flag);
  var gly = heightAt(GOAL_X, GOAL_Z);
  goalGroup.position.set(GOAL_X, gly, GOAL_Z);
  scene.add(goalGroup);

  // goal glow
  var goalLight = new THREE.PointLight(0xff6600, 0.8, 6);
  goalLight.position.set(GOAL_X, gly + 1, GOAL_Z);
  scene.add(goalLight);

  // ── rover build ────────────────────────────────────────────────────────────
  rv.group = new THREE.Group();

  // chassis
  var bodyMat = new THREE.MeshStandardMaterial({ color: 0xa0b0c8, roughness: 0.5, metalness: 0.4 });
  var body = new THREE.Mesh(new THREE.BoxGeometry(1.0, 0.22, 0.72), bodyMat);
  body.position.y = 0.38;
  body.castShadow = true;
  rv.group.add(body);

  // solar panels
  var panelMat = new THREE.MeshStandardMaterial({ color: 0x1a3a6a, roughness: 0.3, metalness: 0.7, emissive: 0x0a1a3a });
  var panel = new THREE.Mesh(new THREE.BoxGeometry(0.7, 0.04, 0.5), panelMat);
  panel.position.set(0, 0.52, 0);
  rv.group.add(panel);

  // antenna
  var antMat = new THREE.MeshStandardMaterial({ color: 0xcccccc, metalness: 0.9, roughness: 0.1 });
  var ant = new THREE.Mesh(new THREE.CylinderGeometry(0.015, 0.015, 0.5, 6), antMat);
  ant.position.set(-0.2, 0.78, 0);
  rv.group.add(ant);
  var dish = new THREE.Mesh(new THREE.SphereGeometry(0.1, 8, 6, 0, Math.PI), antMat);
  dish.rotation.x = -Math.PI / 4;
  dish.position.set(-0.2, 1.02, 0);
  rv.group.add(dish);

  // camera mast
  var camMast = new THREE.Mesh(new THREE.CylinderGeometry(0.02, 0.02, 0.35, 6), antMat);
  camMast.position.set(0.38, 0.67, 0);
  rv.group.add(camMast);
  var camHead = new THREE.Mesh(new THREE.BoxGeometry(0.1, 0.08, 0.08),
    new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.5 }));
  camHead.position.set(0.38, 0.87, 0);
  rv.group.add(camHead);

  // wheels — 6 wheels
  var wheelGeo = new THREE.CylinderGeometry(0.18, 0.18, 0.14, 16);
  var wheelMat = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, roughness: 0.9, metalness: 0.3 });
  var wheelOffs = [
    [0.42, 0.43], [0.42, -0.43],
    [0.0, 0.43], [0.0, -0.43],
    [-0.42, 0.43], [-0.42, -0.43]
  ];
  wheelOffs.forEach(function (o) {
    var w = new THREE.Mesh(wheelGeo, wheelMat);
    w.rotation.z = Math.PI / 2;
    w.position.set(o[0], 0.18, o[1]);
    w.castShadow = true;
    rv.group.add(w);
    rv.wheels.push(w);
  });

  // rover light
  var rovLight = new THREE.SpotLight(0xffffff, 1.5, 12, Math.PI / 6, 0.3, 1);
  rovLight.position.set(0.5, 0.5, 0);
  rv.group.add(rovLight);
  var rovLightTarget = new THREE.Object3D();
  rovLightTarget.position.set(4, 0, 0);
  rv.group.add(rovLightTarget);
  rovLight.target = rovLightTarget;

  scene.add(rv.group);

  // ── smooth rover placement (no Y-snap per frame) ───────────────────────────
  function placeRover() {
    var targetY = heightAt(rv.wx, rv.wz);
    // smooth Y — lerp toward terrain height, avoids frame-by-frame snap/bounce
    rv.wy += (targetY - rv.wy) * 0.12;
    rv.group.position.set(rv.wx, rv.wy + 0.18, rv.wz);
    rv.group.rotation.y = rv.yaw;

    // tilt body to follow terrain slope (visual only)
    var dx = heightAt(rv.wx + 0.3, rv.wz) - heightAt(rv.wx - 0.3, rv.wz);
    var dz = heightAt(rv.wx, rv.wz + 0.3) - heightAt(rv.wx, rv.wz - 0.3);
    rv.group.rotation.x += ((-dz / 0.6) - rv.group.rotation.x) * 0.1;
    rv.group.rotation.z += ((dx / 0.6) - rv.group.rotation.z) * 0.1;
  }

  // init Y
  rv.wy = heightAt(rv.wx, rv.wz);
  placeRover();

  // ── input ──────────────────────────────────────────────────────────────────
  function setKey(k, v) { if (v) keys[k] = true; else delete keys[k]; }

  window.addEventListener("keydown", function (e) {
    var k = e.key.toLowerCase();
    if (k === "arrowup") k = "w";
    if (k === "arrowdown") k = "s";
    if (k === "arrowleft") k = "a";
    if (k === "arrowright") k = "d";
    if ("wasd ".indexOf(k) >= 0) { e.preventDefault(); setKey(k, true); }
  });
  window.addEventListener("keyup", function (e) {
    var k = e.key.toLowerCase();
    if (k === "arrowup") k = "w";
    if (k === "arrowdown") k = "s";
    if (k === "arrowleft") k = "a";
    if (k === "arrowright") k = "d";
    setKey(k, false);
  });

  document.querySelectorAll(".dpad-btn").forEach(function (btn) {
    var k = btn.dataset.key;
    btn.addEventListener("pointerdown", function (e) { e.preventDefault(); btn.classList.add("pressed"); setKey(k, true); });
    btn.addEventListener("pointerup", function () { btn.classList.remove("pressed"); setKey(k, false); });
    btn.addEventListener("pointerleave", function () { btn.classList.remove("pressed"); setKey(k, false); });
  });

  var speedSlider = document.getElementById("speed-slider");
  if (speedSlider) speedSlider.addEventListener("input", function () { maxSpeed = parseFloat(speedSlider.value); });

  document.querySelectorAll(".tab").forEach(function (tab) {
    tab.addEventListener("click", function () {
      document.querySelectorAll(".tab").forEach(function (t) { t.classList.remove("active"); });
      tab.classList.add("active");
      mode = tab.dataset.mode;
      patrolIdx = 0;
      if (elStatus) elStatus.textContent = mode === "patrol"
        ? "Patrol AI — autonomous waypoint navigation"
        : "Manual — WASD / Arrow keys / on-screen pad";
    });
  });

  document.querySelectorAll(".cam-btn").forEach(function (btn) {
    btn.addEventListener("click", function () {
      document.querySelectorAll(".cam-btn").forEach(function (b) { b.classList.remove("active"); });
      btn.classList.add("active");
      camMode = btn.dataset.cam;
    });
  });

  // orbit drag
  wrap.addEventListener("pointerdown", function (e) {
    if (camMode !== "orbit") return;
    orbit.active = true; orbit.startX = e.clientX; orbit.startY = e.clientY;
  });
  window.addEventListener("pointermove", function (e) {
    if (!orbit.active) return;
    orbit.azimuth -= (e.clientX - orbit.startX) * 0.005;
    orbit.elevation -= (e.clientY - orbit.startY) * 0.004;
    orbit.elevation = Math.max(0.1, Math.min(1.3, orbit.elevation));
    orbit.startX = e.clientX; orbit.startY = e.clientY;
  });
  window.addEventListener("pointerup", function () { orbit.active = false; });
  wrap.addEventListener("wheel", function (e) {
    if (camMode !== "orbit") return;
    orbit.radius = Math.max(3, Math.min(30, orbit.radius + e.deltaY * 0.02));
  });

  // ── physics / drive ────────────────────────────────────────────────────────
  function drive(dt) {
    if (mode === "patrol") {
      if (patrolIdx >= PATROL_WPS.length) patrolIdx = 0;
      var wp = PATROL_WPS[patrolIdx];
      var dx = wp[0] - rv.wx, dz = wp[1] - rv.wz;
      var dist = Math.sqrt(dx * dx + dz * dz);
      if (dist < 0.5) { patrolIdx++; return; }
      var targetYaw = Math.atan2(dx, dz);
      var err = targetYaw - rv.yaw;
      while (err > Math.PI) err -= 2 * Math.PI;
      while (err < -Math.PI) err += 2 * Math.PI;
      var targetTurn = Math.max(-1.5, Math.min(1.5, err * 2.5));
      var targetSpd = Math.abs(err) > 0.6 ? 0.1 : Math.min(maxSpeed, dist * 0.5);
      rv.turn += (targetTurn - rv.turn) * Math.min(1, dt * 6);
      rv.speed += (targetSpd - rv.speed) * Math.min(1, dt * 4);
    } else {
      var fwd = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
      var trn = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
      if (keys[" "]) { fwd = 0; trn = 0; }
      var tspd = fwd * maxSpeed;
      var ttrn = trn * 1.6;
      rv.speed += (tspd - rv.speed) * Math.min(1, dt * 7);
      rv.turn += (ttrn - rv.turn) * Math.min(1, dt * 8);
    }

    rv.yaw += rv.turn * dt;
    rv.wx += Math.sin(rv.yaw) * rv.speed * dt;
    rv.wz += Math.cos(rv.yaw) * rv.speed * dt;

    // wheel spin
    rv.wheelAngle += rv.speed * dt * 5.5;
    rv.wheels.forEach(function (w) { w.rotation.x = rv.wheelAngle; });
  }

  // ── camera ─────────────────────────────────────────────────────────────────
  var camVel = new THREE.Vector3();
  function updateCamera() {
    var rx = rv.wx, rz = rv.wz, ry = rv.wy + 0.4;

    if (camMode === "top") {
      camera.position.set(rx, ry + 18, rz + 0.01);
      camera.lookAt(rx, ry, rz);
    } else if (camMode === "orbit") {
      var ox = rx + orbit.radius * Math.sin(orbit.azimuth) * Math.cos(orbit.elevation);
      var oy = ry + orbit.radius * Math.sin(orbit.elevation);
      var oz = rz + orbit.radius * Math.cos(orbit.azimuth) * Math.cos(orbit.elevation);
      camera.position.set(ox, oy, oz);
      camera.lookAt(rx, ry, rz);
    } else {
      // chase cam — smooth follow behind rover
      var behind = 8, above = 3.5;
      var tx = rx - Math.sin(rv.yaw) * behind;
      var ty = ry + above;
      var tz = rz - Math.cos(rv.yaw) * behind;
      camera.position.x += (tx - camera.position.x) * 0.06;
      camera.position.y += (ty - camera.position.y) * 0.05;
      camera.position.z += (tz - camera.position.z) * 0.06;
      camera.lookAt(rx, ry + 0.5, rz);
    }
  }

  // ── HUD update ─────────────────────────────────────────────────────────────
  var simTime = 0;
  var battery = 100;
  function updateHUD(dt) {
    simTime += dt;
    battery = Math.max(0, 100 - simTime * 0.05);

    var spd = Math.abs(rv.speed);
    var deg = (((rv.yaw * 57.2958) % 360) + 360) % 360;
    var gdx = GOAL_X - rv.wx, gdz = GOAL_Z - rv.wz;
    var gdist = Math.sqrt(gdx * gdx + gdz * gdz);

    if (elSpeed) elSpeed.textContent = spd.toFixed(2) + " m/s";
    if (elHead) elHead.textContent = deg.toFixed(0) + "°";
    if (elPos) elPos.textContent = rv.wx.toFixed(1) + ", " + rv.wz.toFixed(1);
    if (elGoal) elGoal.textContent = gdist.toFixed(1) + " m";
    if (elBattery) elBattery.textContent = battery.toFixed(0) + "%";
    if (elGrav) elGrav.textContent = "1.62 m/s²";
    if (elTemp) elTemp.textContent = "−153 °C";

    // goal reached
    if (gdist < 1.2 && elStatus) {
      elStatus.textContent = "🎯 Goal reached! Drive to explore further.";
    }

    // animate goal flag
    flag.rotation.z = Math.sin(simTime * 2) * 0.12;
    goalLight.intensity = 0.6 + Math.sin(simTime * 3) * 0.2;
  }

  // ── render loop ────────────────────────────────────────────────────────────
  var last = performance.now();
  function loop(now) {
    requestAnimationFrame(loop);
    var dt = Math.min(0.05, (now - last) / 1000);
    last = now;

    drive(dt);
    placeRover();
    updateCamera();
    updateHUD(dt);
    renderer.render(scene, camera);
  }
  requestAnimationFrame(loop);

  // ── resize ─────────────────────────────────────────────────────────────────
  window.addEventListener("resize", function () {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // init
  rv.wy = heightAt(rv.wx, rv.wz);
  if (elStatus) elStatus.textContent = "Click here, then press W to drive forward  ·  WASD / Arrow keys";
  document.body.setAttribute("tabindex", "0");
  document.body.focus();
})();
