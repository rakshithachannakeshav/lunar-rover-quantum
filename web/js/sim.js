/* Lunar Rover Web Sim — Obstacle avoidance + classical algorithm visualization */
(function () {
  "use strict";

  var errEl = document.getElementById("load-error");
  if (typeof THREE === "undefined") { if (errEl) errEl.hidden = false; return; }
  if (errEl) errEl.hidden = true;

  // ── constants ──────────────────────────────────────────────────────────────
  // spawn and goal sit ~48 m apart across the map
  var START_X = -18, START_Z = 14;
  var GOAL_X = 20, GOAL_Z = -16;
  var PATROL_WPS = [
    [10, 0], [20, -10], [15, -20], [0, -15], [-10, -5], [0, 10], [10, 0]
  ];

  // rocks: [x, z, scaleX, scaleY, scaleZ]
  var ROCK_DEFS = [
    [3, 1, 1.8, 0.9, 1.4], [-2, 4, 1.2, 0.8, 1.1], [6, -2, 2.2, 1.1, 1.9],
    [-5, -3, 1.5, 1.0, 1.3], [9, 5, 1.1, 0.7, 1.0], [-8, 7, 2.0, 1.3, 1.8],
    [4, -7, 1.3, 0.9, 1.2], [-3, -8, 1.6, 1.1, 1.4], [7, 2, 1.0, 0.8, 1.1],
    [-6, 2, 1.4, 0.7, 1.2], [2, 8, 1.8, 1.0, 1.5], [-9, -5, 1.2, 0.6, 1.0],
    // outer field along the long spawn -> goal crossing
    [-14, 10, 1.6, 1.0, 1.4], [-10, 13, 1.3, 0.8, 1.2], [-12, 4, 2.0, 1.2, 1.7],
    [-16, -2, 1.4, 0.9, 1.3], [-20, 6, 1.5, 0.9, 1.3], [-4, 12, 1.3, 0.8, 1.2],
    [12, -8, 1.7, 1.0, 1.5], [15, -13, 1.2, 0.8, 1.1], [17, -20, 1.9, 1.1, 1.6],
    [10, -16, 1.4, 0.9, 1.2], [14, -2, 1.5, 1.0, 1.3], [5, -12, 1.6, 1.0, 1.4],
    [20, -8, 1.1, 0.7, 1.0], [22, -24, 1.3, 0.8, 1.1]
  ];

  // seeded PRNG (mulberry32) so the world is identical on every load
  function makeRand(seed) {
    return function () {
      seed = (seed + 0x6D2B79F5) | 0;
      var t = Math.imul(seed ^ (seed >>> 15), 1 | seed);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  // planning grid (shared by A*, Dijkstra and QAOA)
  var GRID_RES = 0.5;       // meters per cell
  var GRID_MIN = -30;
  var GRID_MAX = 30;

  // ── lunar environment constants ────────────────────────────────────────────
  var G_MOON = 1.62;                 // m/s²
  var SOLAR_CONST = 1361;            // W/m² — no atmosphere, so all of it reaches the ground
  var PANEL_AREA = 0.35;             // m² (0.7 × 0.5 m deck)
  var PANEL_EFF = 0.28;              // triple-junction GaAs cells
  var SITE_LAT = 20 * Math.PI / 180; // landing-site latitude
  var LUNAR_DAY_EARTH_DAYS = 29.53;  // one sunrise-to-sunrise cycle
  var GCR_DOSE_USV_H = 57;           // Chang'e-4 LND surface dose rate ≈ 1369 µSv/day
  var EARTH_LIGHT_S = 1.28;          // one-way radio delay Earth ↔ Moon

  // Sun path: rises in the east, sets in the west, leaning toward the equator.
  // The scene is turned so the start view looks roughly west, toward Earth.
  var UP = new THREE.Vector3(0, 1, 0);
  var EAST = new THREE.Vector3(-0.5, 0, -0.866).normalize();
  var SOUTH = new THREE.Vector3().crossVectors(EAST, UP).normalize();
  // local solar time on a 24 "lunar hour" clock: 06:00 sunrise, 12:00 noon, 18:00 sunset
  function sunDirection(hour, out) {
    var th = (hour - 6) / 12 * Math.PI;
    var tilt = new THREE.Vector3().copy(UP).multiplyScalar(Math.cos(SITE_LAT)).addScaledVector(SOUTH, Math.sin(SITE_LAT));
    return (out || new THREE.Vector3()).copy(EAST).multiplyScalar(Math.cos(th)).addScaledVector(tilt, Math.sin(th)).normalize();
  }

  // ── obstacle registry (rocks with collision radii) ─────────────────────────
  var obstacles = [];
  var ROVER_RADIUS = 0.65;  // rover collision radius

  // ── state ──────────────────────────────────────────────────────────────────
  var keys = {};
  var mode = "manual";
  var maxSpeed = 0.5;
  var camMode = "chase";
  var patrolIdx = 0;

  // rover 2D world coords (x, z in THREE space)
  var rv = {
    wx: START_X, wz: START_Z,  // world X and Z
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

  // ── THREE setup ────────────────────────────────────────────────────────────
  var renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.0;
  wrap.appendChild(renderer.domElement);

  var scene = new THREE.Scene();
  // pitch-black sky and NO fog: the Moon has no atmosphere, so there is no blue
  // sky, no haze and distant hills stay razor sharp
  scene.background = new THREE.Color(0x000000);

  var camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 2600);
  camera.position.set(-8, 5, 10);

  // lights — a single hard white sun (no atmospheric scattering to soften or
  // redden it), faint light bounced off the regolith, and earthshine at night
  var hemi = new THREE.HemisphereLight(0x1a1c24, 0x5c5850, 0.4);
  scene.add(hemi);
  var sun = new THREE.DirectionalLight(0xffffff, 3.0);
  sun.castShadow = true;
  sun.shadow.mapSize.set(2048, 2048);
  sun.shadow.camera.near = 1;
  sun.shadow.camera.far = 420;
  sun.shadow.camera.left = -38;
  sun.shadow.camera.right = 38;
  sun.shadow.camera.top = 38;
  sun.shadow.camera.bottom = -38;
  sun.shadow.bias = -0.0004;
  sun.shadow.normalBias = 0.04;
  scene.add(sun);
  scene.add(sun.target);
  var earthshine = new THREE.DirectionalLight(0x9ab8ff, 0.05);
  scene.add(earthshine);
  scene.add(earthshine.target);

  // ── sky: stars, sun disc and Earth, drawn around the camera ───────────────
  var SKY_R = 1800;
  var skyGroup = new THREE.Group();
  scene.add(skyGroup);

  var starMat = new THREE.PointsMaterial({ size: 1.6, sizeAttenuation: false, vertexColors: true,
                                           transparent: true, opacity: 1, depthWrite: false });
  (function buildStars() {
    var srand = makeRand(4242);
    var sg = new THREE.BufferGeometry();
    var pos = [], col = [];
    for (var i = 0; i < 5000; i++) {
      var r = SKY_R * 1.1;
      var theta = srand() * Math.PI * 2;
      var phi = Math.acos(2 * srand() - 1);
      pos.push(r * Math.sin(phi) * Math.cos(theta), r * Math.cos(phi), r * Math.sin(phi) * Math.sin(theta));
      var b = 0.25 + 0.75 * Math.pow(srand(), 3);       // most stars faint, a few bright
      var tint = srand();                                // a few warm / cool stars
      col.push(b * (tint > 0.85 ? 1.0 : 0.85), b * 0.9, b * (tint < 0.15 ? 1.0 : 0.85));
    }
    sg.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
    sg.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
    skyGroup.add(new THREE.Points(sg, starMat));
  })();

  function canvasTexture(w, h, draw) {
    var c = document.createElement("canvas");
    c.width = w; c.height = h;
    draw(c.getContext("2d"), w, h);
    return new THREE.CanvasTexture(c);
  }

  // the sun: a white disc with glare (seen from the Moon it is white, not yellow)
  var sunSprite = new THREE.Sprite(new THREE.SpriteMaterial({
    map: canvasTexture(256, 256, function (g, w, h) {
      var grd = g.createRadialGradient(w / 2, h / 2, 0, w / 2, h / 2, w / 2);
      grd.addColorStop(0.0, "rgba(255,255,255,1)");
      grd.addColorStop(0.07, "rgba(255,255,255,1)");
      grd.addColorStop(0.1, "rgba(255,250,240,0.55)");
      grd.addColorStop(0.3, "rgba(255,245,230,0.12)");
      grd.addColorStop(1.0, "rgba(255,245,230,0)");
      g.fillStyle = grd; g.fillRect(0, 0, w, h);
    }),
    blending: THREE.AdditiveBlending, depthWrite: false, transparent: true
  }));
  sunSprite.scale.set(220, 220, 1);
  skyGroup.add(sunSprite);

  // Earth: ~1.9° across, hangs almost fixed in the lunar sky; its phase is lit
  // by the same sun direction, so it waxes while the Moon's day turns to night
  var earthTex = canvasTexture(512, 256, function (g, w, h) {
    var er = makeRand(99);
    g.fillStyle = "#1d4a8a"; g.fillRect(0, 0, w, h);
    for (var y = 0; y < h; y++) {
      for (var x = 0; x < w; x += 2) {
        var u = x / w * Math.PI * 2, v = y / h * Math.PI;
        var land = Math.sin(u * 2 + 1.3) * Math.sin(v * 3 + 0.5) + 0.6 * Math.sin(u * 5 + v * 2) + 0.35 * Math.sin(u * 11 - v * 7);
        if (land > 0.75) { g.fillStyle = land > 1.25 ? "#8a7a55" : "#4f7a3c"; g.fillRect(x, y, 2, 1); }
      }
    }
    for (var i = 0; i < 260; i++) {                       // cloud swirls
      g.fillStyle = "rgba(255,255,255," + (0.25 + er() * 0.45) + ")";
      g.beginPath();
      g.ellipse(er() * w, h * (0.12 + er() * 0.76), 6 + er() * 26, 2 + er() * 5, er() * 0.6 - 0.3, 0, Math.PI * 2);
      g.fill();
    }
    g.fillStyle = "rgba(245,248,255,0.95)";                 // polar caps
    g.fillRect(0, 0, w, h * 0.07); g.fillRect(0, h * 0.93, w, h * 0.07);
  });
  var earthMat = new THREE.ShaderMaterial({
    uniforms: { map: { value: earthTex }, sunDir: { value: new THREE.Vector3(1, 0, 0) } },
    vertexShader: [
      "varying vec3 vN; varying vec2 vUv; varying vec3 vV;",
      "void main() {",
      "  vUv = uv; vN = normalize(mat3(modelMatrix) * normal);",
      "  vec4 wp = modelMatrix * vec4(position, 1.0); vV = normalize(cameraPosition - wp.xyz);",
      "  gl_Position = projectionMatrix * viewMatrix * wp;",
      "}"].join("\n"),
    fragmentShader: [
      "uniform sampler2D map; uniform vec3 sunDir; varying vec3 vN; varying vec2 vUv; varying vec3 vV;",
      "void main() {",
      "  float l = dot(normalize(vN), sunDir);",
      "  float day = smoothstep(-0.08, 0.2, l);",
      "  vec3 c = texture2D(map, vUv).rgb * (0.015 + 1.25 * day * max(l, 0.15));",
      "  float rim = pow(1.0 - max(dot(normalize(vN), vV), 0.0), 3.0);",
      "  c += vec3(0.25, 0.45, 1.0) * rim * 0.9 * day;",   // thin atmosphere glow
      "  gl_FragColor = vec4(c, 1.0);",
      "}"].join("\n")
  });
  var earth = new THREE.Mesh(new THREE.SphereGeometry(SKY_R * Math.tan(0.95 * Math.PI / 180), 48, 32), earthMat);
  // Earth hangs ~8° above the western horizon (a site near the Moon's eastern
  // limb as seen from Earth). The Earth–Moon line lies close to the ecliptic, so
  // Earth sits a few degrees off the sun's daily path; its phase is therefore
  // complementary to the local time: nearly full at sunrise, new at sunset.
  var pathNormal = new THREE.Vector3().crossVectors(EAST, sunDirection(12)).normalize();
  var EARTH_DIR = sunDirection(17.45).addScaledVector(pathNormal, Math.sin(6 * Math.PI / 180)).normalize();
  earth.position.copy(EARTH_DIR).multiplyScalar(SKY_R);
  earth.rotation.z = 0.41;   // axial tilt
  skyGroup.add(earth);

  // ── terrain profiles ───────────────────────────────────────────────────────
  // Each profile places uphill/downhill features on the spawn→goal route so the
  // planners have to trade distance against climbing.
  var MID_X = (START_X + GOAL_X) / 2, MID_Z = (START_Z + GOAL_Z) / 2;
  // unit vector along the spawn→goal axis, and its perpendicular
  var AX_LEN = Math.sqrt((GOAL_X - START_X) * (GOAL_X - START_X) + (GOAL_Z - START_Z) * (GOAL_Z - START_Z));
  var AX_X = (GOAL_X - START_X) / AX_LEN, AX_Z = (GOAL_Z - START_Z) / AX_LEN;

  var TERRAINS = {
    flat:    { label: "Flat",    desc: "Gentle undulation only — the baseline." },
    rolling: { label: "Rolling", desc: "Several hills and dips scattered across the route." },
    hill:    { label: "Hill",    desc: "A 2.6 m hill on the straight line — climb it or drive around?" },
    crater:  { label: "Crater",  desc: "A 1.5 m deep bowl on the direct route — downhill in, uphill out." },
    ridge:   { label: "Ridge",   desc: "A 2.5 m ridge too steep to cross, except at one saddle pass." }
  };
  var terrainId = "flat";

  var ROLLING_BUMPS = [
    [0.15, 3, 1.0, 2.6], [0.28, -4, -0.8, 3.0], [0.40, 1, 1.2, 3.2], [0.52, 5, -0.9, 2.8],
    [0.63, -3, 1.1, 3.0], [0.75, 4, 1.0, 2.6], [0.87, -2, -0.8, 2.8]
  ];

  function gauss(x, z, cx, cz, amp, sigma) {
    var dx = x - cx, dz = z - cz;
    return amp * Math.exp(-(dx * dx + dz * dz) / (2 * sigma * sigma));
  }

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

    if (terrainId === "rolling") {
      // [fraction along the route, sideways offset (m), amplitude (m), sigma (m)]
      for (var b = 0; b < ROLLING_BUMPS.length; b++) {
        var bp = ROLLING_BUMPS[b];
        h += gauss(x, z, START_X + bp[0] * (GOAL_X - START_X) + bp[1] * AX_Z,
                   START_Z + bp[0] * (GOAL_Z - START_Z) - bp[1] * AX_X, bp[2], bp[3]);
      }
    } else if (terrainId === "hill") {
      h += gauss(x, z, MID_X, MID_Z, 2.6, 4.0);
    } else if (terrainId === "crater") {
      var r = Math.sqrt((x - MID_X) * (x - MID_X) + (z - MID_Z) * (z - MID_Z));
      var R = 7.0;
      if (r < R) { var u = r / R; h -= 1.5 * (1 - u * u) * (1 - u * u); }
      h += 0.35 * Math.exp(-(r - R) * (r - R) / (2 * 0.9 * 0.9));   // raised rim
    } else if (terrainId === "ridge") {
      var along = (x - MID_X) * AX_X + (z - MID_Z) * AX_Z;    // across the ridge
      var lateral = (x - MID_X) * AX_Z - (z - MID_Z) * AX_X;  // along the ridge crest
      var saddle = 1 - 0.85 * Math.exp(-(lateral + 9) * (lateral + 9) / (2 * 2.0 * 2.0));
      h += 2.5 * Math.exp(-along * along / (2 * 1.6 * 1.6)) * saddle;
    }
    return h;
  }

  // ── visible lunar surface detail ───────────────────────────────────────────
  // heightAt() is the coarse "orbital elevation map" the planners plan on. The
  // ground the rover actually drives over (surfaceAt) adds what a planning map
  // at that resolution cannot see: small degraded impact craters, fine regolith
  // undulation, and the large craters / massifs on the horizon. Planner
  // results (and the benchmark) are therefore unchanged by this layer.
  var craterRand = makeRand(77031);
  var CRATER_CELL = 8;
  var smallCraterBins = {};   // "cx,cz" -> [crater], for fast local lookup
  var bigCraters = [];

  function craterClear(x, z, R) {
    var keep = [[START_X, START_Z], [GOAL_X, GOAL_Z]].concat(ROCK_DEFS);
    for (var k = 0; k < keep.length; k++) {
      var dx = x - keep[k][0], dz = z - keep[k][1], m = R * 1.6 + 1.5;
      if (dx * dx + dz * dz < m * m) return false;
    }
    return true;
  }

  (function buildCraters() {
    var placed = 0;
    for (var tries = 0; placed < 170 && tries < 3000; tries++) {
      var x = (craterRand() * 2 - 1) * 75, z = (craterRand() * 2 - 1) * 75;
      // power-law size distribution: many small craters, few large ones
      var R = 0.5 + 3.5 * Math.pow(craterRand(), 2.2);
      var fresh = craterRand() < 0.18;
      if (!craterClear(x, z, R)) continue;
      var c = { x: x, z: z, R: R, depth: R * (fresh ? 0.11 : 0.06), rim: R * (fresh ? 0.035 : 0.015), fresh: fresh };
      var key = Math.floor(x / CRATER_CELL) + "," + Math.floor(z / CRATER_CELL);
      (smallCraterBins[key] = smallCraterBins[key] || []).push(c);
      placed++;
    }
    // big craters only outside the ±30 m planning area
    for (var b = 0; b < 14; b++) {
      var ang = craterRand() * Math.PI * 2, R2 = 10 + 30 * craterRand();
      var dist = 45 + 1.6 * R2 + craterRand() * 170;
      bigCraters.push({ x: Math.cos(ang) * dist, z: Math.sin(ang) * dist, R: R2,
                        depth: R2 * 0.16, rim: R2 * 0.05, fresh: craterRand() < 0.3 });
    }
  })();

  // bowl + raised rim + ejecta apron
  function craterShape(r, c) {
    var u = r / c.R;
    if (u < 1) return -c.depth * (1 - u * u) + c.rim * u * u * u * u;
    var t = (u - 1) / 0.45;
    return c.rim * Math.exp(-t * t);
  }

  // distant massifs ringing the landing site (they also hide the edge of the world)
  function massifAt(x, z) {
    var r = Math.sqrt(x * x + z * z);
    if (r < 160) return 0;
    var th = Math.atan2(z, x);
    var s = Math.min(1, (r - 160) / 200); s = s * s * (3 - 2 * s);
    var ridge = 20 + 10 * Math.sin(th * 3 + 0.7) + 7 * Math.sin(th * 7 + 2.1) + 4 * Math.sin(th * 13 + 0.3);
    return s * Math.max(4, ridge);
  }

  function forEachCrater(x, z, fn) {
    var cx = Math.floor(x / CRATER_CELL), cz = Math.floor(z / CRATER_CELL);
    for (var i = -1; i <= 1; i++) {
      for (var j = -1; j <= 1; j++) {
        var bin = smallCraterBins[(cx + i) + "," + (cz + j)];
        if (bin) for (var k = 0; k < bin.length; k++) fn(bin[k]);
      }
    }
    for (var b = 0; b < bigCraters.length; b++) fn(bigCraters[b]);
  }

  function detailAt(x, z) {
    // gentle, irregular undulation: several waves in unrelated directions
    var h = 0.018 * Math.sin(x * 0.83 + z * 0.31) * Math.sin(z * 0.97 - x * 0.27 + 0.6 * Math.sin(x * 0.21)) +
            0.012 * Math.sin(x * 1.37 - z * 0.71 + 1.3) +
            0.009 * Math.sin(z * 1.93 + x * 0.52 + 2.0 * Math.sin(z * 0.17)) + massifAt(x, z);
    forEachCrater(x, z, function (c) {
      var dx = x - c.x, dz = z - c.z, r2 = dx * dx + dz * dz;
      if (r2 < 4.84 * c.R * c.R) h += craterShape(Math.sqrt(r2), c);
    });
    return h;
  }

  // relative brightness: mare / highland patches, bright ejecta of fresh craters
  function albedoAt(x, z) {
    var a = 0.93 + 0.07 * Math.sin(x * 0.013 + 0.5) * Math.cos(z * 0.017 + 1.1) +
            0.04 * Math.sin(x * 0.051 + z * 0.043);
    forEachCrater(x, z, function (c) {
      if (!c.fresh) return;
      var dx = x - c.x, dz = z - c.z, u = Math.sqrt(dx * dx + dz * dz) / c.R;
      if (u < 3) a += 0.22 * Math.exp(-(u - 1) * (u - 1) / 0.5);
    });
    return a;
  }

  // the ground the rover actually drives on
  function surfaceAt(x, z) { return heightAt(x, z) + detailAt(x, z); }

  // ── directional slope energy model ─────────────────────────────────────────
  // grade = rise / run along the direction of travel (signed).
  //  uphill   : work against gravity → cost grows with grade
  //  downhill : gentle descents get gravity assist; steeper ones need braking
  //  |slope| > MAX_SLOPE_DEG : impassable (traction / tip-over limit)
  var MAX_SLOPE_DEG = 25;
  var MAX_GRADE = Math.tan(MAX_SLOPE_DEG * Math.PI / 180);
  var UPHILL_K = 4.0;
  var ASSIST_GRADE = 0.15;   // descents gentler than this are gravity-assisted
  var BRAKE_K = 3.0;
  var MIN_GRADE_FACTOR = 1 - ASSIST_GRADE;  // cheapest possible metre (keeps A* admissible)

  function gradeFactor(grade) {
    if (grade >= 0) return 1 + UPHILL_K * grade;
    var g = -grade;
    var f = 1 - Math.min(g, ASSIST_GRADE);
    if (g > ASSIST_GRADE) f += BRAKE_K * (g - ASSIST_GRADE);
    return f;
  }

  // energy to drive straight from (x1,z1) to (x2,z2); Infinity if too steep
  function edgeCost(x1, z1, x2, z2) {
    var dx = x2 - x1, dz = z2 - z1;
    var d = Math.sqrt(dx * dx + dz * dz);
    if (d < 1e-9) return 0;
    var grade = (heightAt(x2, z2) - heightAt(x1, z1)) / d;
    if (Math.abs(grade) > MAX_GRADE) return Infinity;
    return d * gradeFactor(grade);
  }

  function slopeGradeAt(x, z) {
    var gx = (heightAt(x + 0.3, z) - heightAt(x - 0.3, z)) / 0.6;
    var gz = (heightAt(x, z + 0.3) - heightAt(x, z - 0.3)) / 0.6;
    return Math.sqrt(gx * gx + gz * gz);
  }

  // ── terrain mesh ───────────────────────────────────────────────────────────
  // One 800 m × 800 m mesh whose vertices are packed densely near the landing
  // site (~0.3 m apart) and spread out toward the horizon (~8 m apart), so close
  // terrain is detailed and the world has no visible edge.
  var TERRAIN_HALF = 400, TERRAIN_SEG = 400;
  function warpCoord(u) {
    var a = Math.abs(u);
    return (u < 0 ? -1 : 1) * TERRAIN_HALF * (0.15 * a + 0.85 * Math.pow(a, 5));
  }
  var tGeo = new THREE.PlaneGeometry(2, 2, TERRAIN_SEG, TERRAIN_SEG);
  tGeo.rotateX(-Math.PI / 2);
  var tv = tGeo.attributes.position, tuv = tGeo.attributes.uv;
  var vDetail = new Float32Array(tv.count), vAlbedo = new Float32Array(tv.count);
  for (var vi = 0; vi < tv.count; vi++) {
    var vx = warpCoord(tv.getX(vi)), vz = warpCoord(tv.getZ(vi));
    tv.setX(vi, vx); tv.setZ(vi, vz);
    tuv.setXY(vi, vx / 2.5, vz / 2.5);          // regolith grain tiles every 2.5 m
    vDetail[vi] = detailAt(vx, vz);              // terrain-profile independent: cache once
    vAlbedo[vi] = albedoAt(vx, vz);
  }
  tGeo.setAttribute("color", new THREE.Float32BufferAttribute(new Float32Array(tv.count * 3), 3));

  // neutral grey lunar regolith; optional overlay tints steep ground amber and
  // impassable ground red inside the planning area
  var REGOLITH = new THREE.Color(0xa29e96);
  var STEEP = new THREE.Color(0xc9893f);
  var IMPASSABLE = new THREE.Color(0xa8443a);
  var tmpColor = new THREE.Color();
  var slopeOverlay = true;

  function rebuildTerrainMesh() {
    var col = tGeo.attributes.color;
    for (var i = 0; i < tv.count; i++) {
      var x = tv.getX(i), z = tv.getZ(i);
      tv.setY(i, heightAt(x, z) + vDetail[i]);
      tmpColor.copy(REGOLITH).multiplyScalar(vAlbedo[i]);
      if (slopeOverlay && Math.abs(x) <= GRID_MAX && Math.abs(z) <= GRID_MAX) {
        var g = slopeGradeAt(x, z);
        if (g > MAX_GRADE) tmpColor.lerp(IMPASSABLE, 0.85);
        else tmpColor.lerp(STEEP, Math.min(1, g / MAX_GRADE) * 0.75);
      }
      col.setXYZ(i, tmpColor.r, tmpColor.g, tmpColor.b);
    }
    tv.needsUpdate = true;
    col.needsUpdate = true;
    tGeo.computeVertexNormals();
  }
  rebuildTerrainMesh();

  // fine regolith grain: speckled noise with small pebbles, used as colour + bump
  var regolithTex = canvasTexture(512, 512, function (g, w, h) {
    var rr = makeRand(31337);
    var img = g.createImageData(w, h);
    for (var p = 0; p < w * h; p++) {
      var v = 205 + (rr() - 0.5) * 42;
      img.data[p * 4] = v; img.data[p * 4 + 1] = v; img.data[p * 4 + 2] = v; img.data[p * 4 + 3] = 255;
    }
    g.putImageData(img, 0, 0);
    for (var s = 0; s < 600; s++) {               // pebbles and clods
      var px = rr() * w, py = rr() * h, pr = 0.8 + Math.pow(rr(), 3) * 5;
      var shade = rr() < 0.5 ? 140 + rr() * 45 : 225 + rr() * 25;
      g.fillStyle = "rgb(" + (shade | 0) + "," + (shade | 0) + "," + (shade | 0) + ")";
      g.beginPath(); g.arc(px, py, pr, 0, Math.PI * 2); g.fill();
    }
  });
  regolithTex.wrapS = regolithTex.wrapT = THREE.RepeatWrapping;
  regolithTex.anisotropy = renderer.capabilities.getMaxAnisotropy();

  var tMat = new THREE.MeshStandardMaterial({
    color: 0xffffff, vertexColors: true, map: regolithTex,
    bumpMap: regolithTex, bumpScale: 0.035, roughness: 1.0, metalness: 0.0
  });
  var terrain = new THREE.Mesh(tGeo, tMat);
  terrain.receiveShadow = true;
  terrain.castShadow = true;    // hills and crater rims cast long shadows at low sun
  scene.add(terrain);

  // ── rocks (with collision) ─────────────────────────────────────────────────
  var rockMat = new THREE.MeshStandardMaterial({ color: 0x77736c, roughness: 1.0, metalness: 0.0,
                                                 map: regolithTex, bumpMap: regolithTex, bumpScale: 0.05 });
  var rockRand = makeRand(20240607);
  function addRock(x, z, rx, ry, rz) {
    var baseRadius = 0.4 + rockRand() * 0.5;
    var g = new THREE.DodecahedronGeometry(baseRadius, 1);
    // roughen the vertices so boulders look fractured instead of perfect polyhedra
    var gp = g.attributes.position;
    for (var vi = 0; vi < gp.count; vi++) {
      var k = 0.82 + 0.3 * Math.abs(Math.sin(gp.getX(vi) * 7.1 + gp.getY(vi) * 5.3 + gp.getZ(vi) * 3.7 + x));
      gp.setXYZ(vi, gp.getX(vi) * k, gp.getY(vi) * k, gp.getZ(vi) * k);
    }
    g.computeVertexNormals();
    var m = new THREE.Mesh(g, rockMat);
    m.scale.set(rx, ry, rz);
    var y = surfaceAt(x, z);
    m.position.set(x, y + ry * 0.3, z);
    m.rotation.set(rockRand(), rockRand() * 6, rockRand());
    m.castShadow = true;
    m.receiveShadow = true;
    scene.add(m);

    // register obstacle for collision detection
    // collision radius = average XZ scale * base geometry radius
    var collisionRadius = baseRadius * Math.max(rx, rz) * 0.85;
    obstacles.push({ x: x, z: z, radius: collisionRadius, mesh: m, lift: ry * 0.3,
                     top: ry * 0.3 + baseRadius * ry * 0.9, ring: null });
  }
  ROCK_DEFS.forEach(function (r) { addRock(r[0], r[1], r[2], r[3], r[4]); });

  // ── collision detection ────────────────────────────────────────────────────
  function checkCollision(wx, wz) {
    for (var i = 0; i < obstacles.length; i++) {
      var o = obstacles[i];
      var dx = wx - o.x, dz = wz - o.z;
      var dist = Math.sqrt(dx * dx + dz * dz);
      var minDist = o.radius + ROVER_RADIUS;
      if (dist < minDist) {
        return { hit: true, obstacle: o, dx: dx, dz: dz, dist: dist, minDist: minDist };
      }
    }
    return { hit: false };
  }

  function resolveCollision(newX, newZ) {
    // push rover out of any obstacle it would overlap
    for (var iter = 0; iter < 3; iter++) {
      var col = checkCollision(newX, newZ);
      if (!col.hit) break;
      // push rover along the vector from obstacle center to rover
      var pushDist = col.minDist - col.dist + 0.02;
      var nx = col.dx / col.dist;
      var nz = col.dz / col.dist;
      newX += nx * pushDist;
      newZ += nz * pushDist;
    }
    return { x: newX, z: newZ };
  }

  // ── A* pathfinding on a 2D grid ───────────────────────────────────────────
  // Grid-based A* that respects obstacle positions for the web sim
  // GRID_RES / GRID_MIN / GRID_MAX are defined with the constants at the top

  function worldToGrid(wx, wz) {
    return {
      gx: Math.round((wx - GRID_MIN) / GRID_RES),
      gz: Math.round((wz - GRID_MIN) / GRID_RES)
    };
  }

  function gridToWorld(gx, gz) {
    return {
      wx: GRID_MIN + gx * GRID_RES,
      wz: GRID_MIN + gz * GRID_RES
    };
  }

  function isBlocked(gx, gz) {
    var w = gridToWorld(gx, gz);
    for (var i = 0; i < obstacles.length; i++) {
      var o = obstacles[i];
      var dx = w.wx - o.x, dz = w.wz - o.z;
      if (dx * dx + dz * dz < (o.radius + ROVER_RADIUS) * (o.radius + ROVER_RADIUS)) {
        return true;
      }
    }
    return false;
  }

  function gridKey(gx, gz) { return gx + "," + gz; }

  // binary min-heap of grid keys for the open sets; stale entries (already
  // closed, or superseded by a cheaper push) are skipped when popped
  function MinHeap() { this.keys = []; this.pri = []; }
  MinHeap.prototype.push = function (key, p) {
    var k = this.keys, q = this.pri, i = k.length;
    k.push(key); q.push(p);
    while (i > 0) {
      var up = (i - 1) >> 1;
      if (q[up] <= p) break;
      k[i] = k[up]; q[i] = q[up]; i = up;
    }
    k[i] = key; q[i] = p;
  };
  MinHeap.prototype.pop = function () {
    var k = this.keys, q = this.pri;
    if (!k.length) return null;
    var top = k[0], lastK = k.pop(), lastP = q.pop(), n = k.length, i = 0;
    if (n) {
      while (true) {
        var c = 2 * i + 1;
        if (c >= n) break;
        if (c + 1 < n && q[c + 1] < q[c]) c++;
        if (q[c] >= lastP) break;
        k[i] = k[c]; q[i] = q[c]; i = c;
      }
      k[i] = lastK; q[i] = lastP;
    }
    return top;
  };
  function popOpen(heap, open) {
    var key;
    while ((key = heap.pop()) !== null) if (open[key]) return key;
    return null;
  }

  // A* with terrain slope cost (energy-aware, matching the classical_planning.py approach)
  function astarPath(startX, startZ, goalX, goalZ) {
    var s = worldToGrid(startX, startZ);
    var g = worldToGrid(goalX, goalZ);
    var gridSize = Math.round((GRID_MAX - GRID_MIN) / GRID_RES);

    if (isBlocked(g.gx, g.gz)) {
      // try to find nearest unblocked cell to goal
      var found = false;
      for (var r = 1; r <= 6 && !found; r++) {
        for (var ddx = -r; ddx <= r && !found; ddx++) {
          for (var ddz = -r; ddz <= r && !found; ddz++) {
            if (Math.abs(ddx) === r || Math.abs(ddz) === r) {
              if (!isBlocked(g.gx + ddx, g.gz + ddz)) {
                g.gx += ddx; g.gz += ddz;
                found = true;
              }
            }
          }
        }
      }
    }

    // 8-directional neighbors
    var dirs = [
      [1,0],[0,1],[-1,0],[0,-1],
      [1,1],[1,-1],[-1,1],[-1,-1]
    ];

    var open = {};   // key -> { gx, gz, f, g, parent }
    var heap = new MinHeap();
    var closed = {};
    var startKey = gridKey(s.gx, s.gz);
    var goalKey = gridKey(g.gx, g.gz);

    // scaled by the cheapest possible metre so it never overestimates (admissible)
    function heuristic(ax, az) {
      var dx = ax - g.gx, dz = az - g.gz;
      return Math.sqrt(dx * dx + dz * dz) * GRID_RES * MIN_GRADE_FACTOR;
    }

    open[startKey] = { gx: s.gx, gz: s.gz, f: heuristic(s.gx, s.gz), g: 0, parent: null };

    heap.push(startKey, open[startKey].f);

    var iterations = 0;
    var maxIter = gridSize * gridSize;

    while (true) {
      // find lowest f in open
      var bestKey = popOpen(heap, open);
      if (bestKey === null || iterations++ > maxIter) return null;  // no path

      var current = open[bestKey];
      delete open[bestKey];
      closed[bestKey] = current;

      if (bestKey === goalKey) {
        // reconstruct
        var path = [];
        var node = current;
        while (node) {
          var w = gridToWorld(node.gx, node.gz);
          path.unshift([w.wx, w.wz]);
          node = node.parent;
        }
        path.expanded = iterations;
        return path;
      }

      for (var d = 0; d < dirs.length; d++) {
        var ngx = current.gx + dirs[d][0];
        var ngz = current.gz + dirs[d][1];
        if (ngx < 0 || ngx >= gridSize || ngz < 0 || ngz >= gridSize) continue;

        var nk = gridKey(ngx, ngz);
        if (closed[nk]) continue;
        if (isBlocked(ngx, ngz)) continue;

        // directional energy cost: uphill costs more, gentle downhill less, too steep = blocked
        var cw = gridToWorld(current.gx, current.gz);
        var nw = gridToWorld(ngx, ngz);
        var moveCost = edgeCost(cw.wx, cw.wz, nw.wx, nw.wz);
        if (moveCost === Infinity) continue;

        var tentG = current.g + moveCost;

        if (open[nk] && open[nk].g <= tentG) continue;

        open[nk] = {
          gx: ngx, gz: ngz,
          f: tentG + heuristic(ngx, ngz),
          g: tentG,
          parent: current
        };

        heap.push(nk, open[nk].f);
      }
    }
  }

  // Dijkstra — classic shortest-DISTANCE planner. It ignores uphill/downhill cost
  // (it only refuses impassable >25° slopes), so it is the slope-blind baseline that
  // the energy-aware A* and QAOA are compared against.
  function dijkstraPath(startX, startZ, goalX, goalZ) {
    var s = worldToGrid(startX, startZ);
    var g = worldToGrid(goalX, goalZ);
    var gridSize = Math.round((GRID_MAX - GRID_MIN) / GRID_RES);

    if (isBlocked(g.gx, g.gz)) {
      var found = false;
      for (var r = 1; r <= 6 && !found; r++) {
        for (var ddx = -r; ddx <= r && !found; ddx++) {
          for (var ddz = -r; ddz <= r && !found; ddz++) {
            if (Math.abs(ddx) === r || Math.abs(ddz) === r) {
              if (!isBlocked(g.gx + ddx, g.gz + ddz)) {
                g.gx += ddx; g.gz += ddz;
                found = true;
              }
            }
          }
        }
      }
    }

    var dirs = [
      [1,0],[0,1],[-1,0],[0,-1],
      [1,1],[1,-1],[-1,1],[-1,-1]
    ];

    var open = {};
    var closed = {};
    var heap = new MinHeap();
    var startKey = gridKey(s.gx, s.gz);
    var goalKey = gridKey(g.gx, g.gz);

    open[startKey] = { gx: s.gx, gz: s.gz, f: 0, g: 0, parent: null };

    heap.push(startKey, open[startKey].f);

    var iterations = 0;
    var maxIter = gridSize * gridSize;

    while (true) {
      var bestKey = popOpen(heap, open);
      if (bestKey === null || iterations++ > maxIter) return null;

      var current = open[bestKey];
      delete open[bestKey];
      closed[bestKey] = current;

      if (bestKey === goalKey) {
        var path = [];
        var node = current;
        while (node) {
          var w = gridToWorld(node.gx, node.gz);
          path.unshift([w.wx, w.wz]);
          node = node.parent;
        }
        path.expanded = iterations;
        return path;
      }

      for (var d = 0; d < dirs.length; d++) {
        var ngx = current.gx + dirs[d][0];
        var ngz = current.gz + dirs[d][1];
        if (ngx < 0 || ngx >= gridSize || ngz < 0 || ngz >= gridSize) continue;

        var nk = gridKey(ngx, ngz);
        if (closed[nk]) continue;
        if (isBlocked(ngx, ngz)) continue;

        // distance only — slope is ignored except for the impassable limit
        var cw = gridToWorld(current.gx, current.gz);
        var nw = gridToWorld(ngx, ngz);
        if (edgeCost(cw.wx, cw.wz, nw.wx, nw.wz) === Infinity) continue;
        var moveCost = (dirs[d][0] !== 0 && dirs[d][1] !== 0) ? GRID_RES * Math.SQRT2 : GRID_RES;

        var tentG = current.g + moveCost;

        if (open[nk] && open[nk].g <= tentG) continue;

        open[nk] = {
          gx: ngx, gz: ngz,
          f: tentG,
          g: tentG,
          parent: current
        };

        heap.push(nk, open[nk].f);
      }
    }
  }

  // ── QAOA path planning: real QAOA simulation (js/qaoa.js) ───────────────────
  // Mirrors quantum_optimizer.py with no A*, no Dijkstra and no classical
  // fallback: the corridor is a geometric lattice of anchors joined by straight
  // walks, the route choice is an edge-selection QUBO, the circuit is simulated
  // exactly (statevector), and the answer is the valid path QAOA sampled most
  // often. If no shot is a valid path, QAOA reports "no path".
  var QAOA_REPS = 2;       // circuit depth p
  var QAOA_SHOTS = 2048;
  var qaoaState = {
    running: false,
    progress: 0,        // 0-100
    phase: "",           // status text
    qubits: 0,
    shots: 0,
    reps: 0,
    expectation: 0,
    validSamples: 0,
    randomValidSamples: 0,
    selectedCount: 0,
    corridorEdges: 0,
    error: null
  };

  function qaoaPath(startX, startZ, goalX, goalZ) {
    if (typeof QaoaCore === "undefined") {
      qaoaState.error = "js/qaoa.js is not loaded";
      qaoaState.running = false;
      return null;
    }
    qaoaState.phase = "Running QAOA simulation (p=" + QAOA_REPS + ")...";
    var r = QaoaCore.plan({
      start: [startX, startZ], goal: [goalX, goalZ],
      gridRes: GRID_RES, gridMin: GRID_MIN, gridMax: GRID_MAX,
      isBlocked: isBlocked, edgeCost: edgeCost,
      reps: QAOA_REPS, shots: QAOA_SHOTS, seed: 7
    });
    var s = r.stats;
    qaoaState.qubits = s.qubits;
    qaoaState.shots = s.shots;
    qaoaState.reps = s.reps;
    qaoaState.expectation = s.expectation;
    qaoaState.validSamples = s.validSamples;
    qaoaState.randomValidSamples = s.randomValidSamples;
    qaoaState.selectedCount = s.selectedCount;
    qaoaState.corridorEdges = s.corridorEdges;
    qaoaState.error = s.error;
    qaoaState.phase = s.error || "Path decoded";
    qaoaState.progress = 100;
    qaoaState.running = false;
    if (!r.path) return null;
    r.path.expanded = s.walkCells;   // grid cells walked to build the corridor (no search)
    return r.path;
  }

  // ── path visualization ─────────────────────────────────────────────────────
  var pathLines = { dijkstra: null, astar: null, qaoa: null };
  var pathResults = { dijkstra: null, astar: null, qaoa: null };

  function computePathEnergy(path) {
    var energy = 0, dist = 0, climb = 0, descent = 0, maxGrade = 0;
    for (var i = 1; i < path.length; i++) {
      var dx = path[i][0] - path[i-1][0];
      var dz = path[i][1] - path[i-1][1];
      var segDist = Math.sqrt(dx*dx + dz*dz);
      var dh = heightAt(path[i][0], path[i][1]) - heightAt(path[i-1][0], path[i-1][1]);
      energy += edgeCost(path[i-1][0], path[i-1][1], path[i][0], path[i][1]);
      dist += segDist;
      if (dh > 0) climb += dh; else descent -= dh;
      if (segDist > 1e-9) maxGrade = Math.max(maxGrade, Math.abs(dh) / segDist);
    }
    return {
      energy: energy, distance: dist, waypoints: path.length,
      climb: climb, descent: descent, maxSlopeDeg: Math.atan(maxGrade) * 57.2958,
      expanded: path.expanded || 0
    };
  }

  // lateral offset (m) so overlapping paths stay visible side by side
  var PATH_OFFSET = { dijkstra: -0.12, astar: 0, qaoa: 0.12 };

  function drawPath(path, color, name) {
    if (pathLines[name]) {
      scene.remove(pathLines[name]);
      pathLines[name].geometry.dispose();
      pathLines[name].material.dispose();
      pathLines[name] = null;
    }
    if (!path || path.length < 2) return;

    var off = PATH_OFFSET[name] || 0;
    var points = [];
    for (var i = 0; i < path.length; i++) {
      // offset perpendicular to the local direction of travel
      var a0 = path[Math.max(0, i - 1)], a1 = path[Math.min(path.length - 1, i + 1)];
      var tx = a1[0] - a0[0], tz = a1[1] - a0[1], tl = Math.sqrt(tx * tx + tz * tz) || 1;
      var px = path[i][0] - tz / tl * off, pz = path[i][1] + tx / tl * off;
      var py = surfaceAt(px, pz) + 0.15;
      points.push(new THREE.Vector3(px, py, pz));
    }
    var geo = new THREE.BufferGeometry().setFromPoints(points);
    var mat = new THREE.LineBasicMaterial({ color: color, linewidth: 2, transparent: true, opacity: 0.85 });
    var line = new THREE.Line(geo, mat);
    scene.add(line);
    pathLines[name] = line;
  }

  // Waypoint markers for the active navigation path
  var waypointMarkers = [];
  function drawWaypointMarkers(path, color) {
    waypointMarkers.forEach(function(m) { scene.remove(m); m.geometry.dispose(); m.material.dispose(); });
    waypointMarkers = [];
    if (!path || path.length < 2) return;

    // only place markers at every Nth point to avoid clutter
    var step = Math.max(1, Math.floor(path.length / 20));
    var markerGeo = new THREE.SphereGeometry(0.12, 8, 6);
    var markerMat = new THREE.MeshStandardMaterial({ color: color, emissive: color, emissiveIntensity: 0.4 });
    for (var i = 0; i < path.length; i += step) {
      var py = surfaceAt(path[i][0], path[i][1]) + 0.25;
      var m = new THREE.Mesh(markerGeo, markerMat);
      m.position.set(path[i][0], py, path[i][1]);
      scene.add(m);
      waypointMarkers.push(m);
    }
  }

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
  // tall translucent beacon so the far-away goal is visible from the spawn
  var beacon = new THREE.Mesh(
    new THREE.CylinderGeometry(0.15, 0.15, 14, 12, 1, true),
    new THREE.MeshBasicMaterial({ color: 0xff6600, transparent: true, opacity: 0.25, depthWrite: false })
  );
  beacon.position.y = 7;
  goalGroup.add(beacon);
  var flag = new THREE.Mesh(
    new THREE.PlaneGeometry(0.6, 0.35),
    new THREE.MeshStandardMaterial({ color: 0xff6600, side: THREE.DoubleSide, emissive: 0xff3300, emissiveIntensity: 0.3 })
  );
  flag.position.set(0.3, 1.8, 0);
  goalGroup.add(flag);
  var gly = surfaceAt(GOAL_X, GOAL_Z);
  goalGroup.position.set(GOAL_X, gly, GOAL_Z);
  scene.add(goalGroup);

  // goal glow
  var goalLight = new THREE.PointLight(0xff6600, 0.8, 6);
  goalLight.position.set(GOAL_X, gly + 1, GOAL_Z);
  scene.add(goalLight);

  // ── obstacle collision markers (red rings) ─────────────────────────────────
  obstacles.forEach(function (o) {
    var ringGeo = new THREE.RingGeometry(o.radius + ROVER_RADIUS - 0.05, o.radius + ROVER_RADIUS + 0.05, 32);
    var ringMat = new THREE.MeshBasicMaterial({
      color: 0xff3333, side: THREE.DoubleSide, transparent: true, opacity: 0.18
    });
    var ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.set(o.x, surfaceAt(o.x, o.z) + 0.05, o.z);
    scene.add(ring);
    o.ring = ring;
  });

  // ── rover build ────────────────────────────────────────────────────────────
  // Rover local frame: +Z = forward (direction of travel), +X = left side, +Y = up.
  // Six-wheel rocker-bogie layout, three wheels per side; wheel bottoms sit at y = 0.
  rv.group = new THREE.Group();
  var WHEEL_R = 0.18;

  function addPart(geo, mat, x, y, z, parent) {
    var m = new THREE.Mesh(geo, mat);
    m.position.set(x, y, z);
    m.castShadow = true;
    m.receiveShadow = true;
    (parent || rv.group).add(m);
    return m;
  }

  // chassis ("warm electronics box") wrapped in gold thermal foil (MLI)
  var bodyMat = new THREE.MeshStandardMaterial({ color: 0xd9dde3, roughness: 0.45, metalness: 0.35 });
  var foilMat = new THREE.MeshStandardMaterial({ color: 0xc8a14a, roughness: 0.3, metalness: 0.85 });
  var darkMat = new THREE.MeshStandardMaterial({ color: 0x2a2c30, roughness: 0.6, metalness: 0.3 });
  var antMat = new THREE.MeshStandardMaterial({ color: 0xcfd3d8, metalness: 0.9, roughness: 0.2 });
  addPart(new THREE.BoxGeometry(0.62, 0.22, 0.96), bodyMat, 0, 0.44, 0);
  addPart(new THREE.BoxGeometry(0.64, 0.06, 0.98), foilMat, 0, 0.32, 0);

  // solar deck (the panel normal is used for the solar-power model)
  var panelMat = new THREE.MeshStandardMaterial({ color: 0x14254a, roughness: 0.25, metalness: 0.6, emissive: 0x050a18 });
  addPart(new THREE.BoxGeometry(0.72, 0.03, 0.52), panelMat, 0, 0.57, -0.12);
  addPart(new THREE.BoxGeometry(0.74, 0.015, 0.54), antMat, 0, 0.555, -0.12);

  // high-gain antenna (points at Earth) and a low-gain whip
  addPart(new THREE.CylinderGeometry(0.015, 0.015, 0.36, 6), antMat, -0.22, 0.73, -0.36);
  var dish = addPart(new THREE.SphereGeometry(0.12, 16, 8, 0, Math.PI * 2, 0, Math.PI / 2.6), antMat, -0.22, 0.93, -0.36);
  dish.material = new THREE.MeshStandardMaterial({ color: 0xe8eaee, metalness: 0.5, roughness: 0.3, side: THREE.DoubleSide });
  dish.rotation.x = Math.PI * 0.75;
  addPart(new THREE.CylinderGeometry(0.006, 0.006, 0.45, 4), antMat, 0.24, 0.78, -0.4);

  // camera mast with stereo navcams at the front
  addPart(new THREE.CylinderGeometry(0.022, 0.022, 0.42, 8), antMat, 0.16, 0.76, 0.36);
  var camHead = addPart(new THREE.BoxGeometry(0.2, 0.08, 0.09), darkMat, 0.16, 0.99, 0.37);
  var lensMat = new THREE.MeshStandardMaterial({ color: 0x0a0a12, metalness: 0.9, roughness: 0.05, emissive: 0x0a1424 });
  addPart(new THREE.CylinderGeometry(0.022, 0.022, 0.02, 12), lensMat, -0.05, 0, 0.05, camHead).rotation.x = Math.PI / 2;
  addPart(new THREE.CylinderGeometry(0.022, 0.022, 0.02, 12), lensMat, 0.05, 0, 0.05, camHead).rotation.x = Math.PI / 2;

  // front LiDAR puck and hazard cameras
  addPart(new THREE.CylinderGeometry(0.06, 0.06, 0.07, 16), darkMat, -0.12, 0.6, 0.38);
  addPart(new THREE.BoxGeometry(0.3, 0.05, 0.04), darkMat, 0, 0.42, 0.5);

  // wheels: aluminium drums with grousers; rocker-bogie links on each side
  var wheelGeo = new THREE.CylinderGeometry(WHEEL_R, WHEEL_R, 0.14, 20);
  var wheelMat = new THREE.MeshStandardMaterial({ color: 0x9a9fa6, roughness: 0.55, metalness: 0.7 });
  var hubMat = new THREE.MeshStandardMaterial({ color: 0x3a3d42, roughness: 0.5, metalness: 0.6 });
  var grouserGeo = new THREE.BoxGeometry(0.03, 0.145, 0.018);
  var linkMat = new THREE.MeshStandardMaterial({ color: 0xb8bcc2, roughness: 0.4, metalness: 0.7 });
  var WHEEL_Z = [0.42, 0.0, -0.42];
  [0.43, -0.43].forEach(function (side) {
    WHEEL_Z.forEach(function (wz) {
      var w = addPart(wheelGeo, wheelMat, side, WHEEL_R, wz);
      w.rotation.z = Math.PI / 2;            // axle along X, rolls along Z
      for (var gi = 0; gi < 10; gi++) {      // grousers make the spin visible
        var a = gi / 10 * Math.PI * 2;
        var gr = addPart(grouserGeo, hubMat, Math.cos(a) * WHEEL_R, 0, Math.sin(a) * WHEEL_R, w);
        gr.rotation.y = -a;
        gr.castShadow = false;
      }
      addPart(new THREE.CylinderGeometry(0.06, 0.06, 0.15, 10), hubMat, 0, 0, 0, w);
      rv.wheels.push(w);
    });
    // rocker (front wheel → body pivot → bogie) and bogie (middle ↔ rear wheel)
    var rocker = addPart(new THREE.BoxGeometry(0.035, 0.035, 0.62), linkMat, side * 0.8, 0.33, 0.2);
    rocker.rotation.x = -0.32;
    var bogie = addPart(new THREE.BoxGeometry(0.035, 0.035, 0.5), linkMat, side * 0.8, 0.27, -0.21);
    bogie.rotation.x = 0.2;
    addPart(new THREE.CylinderGeometry(0.03, 0.03, 0.14, 8), linkMat, side * 0.76, 0.42, 0.02).rotation.z = Math.PI / 2;
  });

  // headlights (switch on automatically at night or in shadow)
  var rovLight = new THREE.SpotLight(0xf4f6ff, 0, 22, Math.PI / 5, 0.45, 1.2);
  rovLight.position.set(0, 0.5, 0.5);
  rv.group.add(rovLight);
  var rovLightTarget = new THREE.Object3D();
  rovLightTarget.position.set(0, 0, 5);
  rv.group.add(rovLightTarget);
  rovLight.target = rovLightTarget;
  var lampMat = new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x000000 });
  addPart(new THREE.BoxGeometry(0.08, 0.04, 0.02), lampMat, 0.18, 0.47, 0.49);
  addPart(new THREE.BoxGeometry(0.08, 0.04, 0.02), lampMat, -0.18, 0.47, 0.49);

  scene.add(rv.group);

  // ── smooth rover placement (no Y-snap per frame) ───────────────────────────
  function placeRover() {
    var targetY = surfaceAt(rv.wx, rv.wz);
    // smooth Y — lerp toward terrain height, avoids frame-by-frame snap/bounce
    rv.wy += (targetY - rv.wy) * 0.12;
    rv.group.position.set(rv.wx, rv.wy - 0.015, rv.wz);   // wheels sink ~1.5 cm into the regolith
    rv.group.rotation.y = rv.yaw;

    // tilt body to follow terrain slope, sampled over the wheelbase
    var fx = Math.sin(rv.yaw), fz = Math.cos(rv.yaw);   // forward
    var lx = fz, lz = -fx;                              // left
    var pitchG = (surfaceAt(rv.wx + fx * 0.42, rv.wz + fz * 0.42) - surfaceAt(rv.wx - fx * 0.42, rv.wz - fz * 0.42)) / 0.84;
    var rollG = (surfaceAt(rv.wx + lx * 0.43, rv.wz + lz * 0.43) - surfaceAt(rv.wx - lx * 0.43, rv.wz - lz * 0.43)) / 0.86;
    rv.group.rotation.order = "YXZ";
    rv.group.rotation.x += (-Math.atan(pitchG) - rv.group.rotation.x) * 0.1;
    rv.group.rotation.z += (Math.atan(rollG) - rv.group.rotation.z) * 0.1;
  }

  // init Y
  rv.wy = surfaceAt(rv.wx, rv.wz);
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

  // ── A* navigation state ────────────────────────────────────────────────────
  var navPath = null;         // current A* path the rover follows
  var navPathIdx = 0;         // which waypoint we're heading to
  var lastComputedMode = null;

  document.querySelectorAll(".tab").forEach(function (tab) {
    tab.addEventListener("click", function () {
      document.querySelectorAll(".tab").forEach(function (t) { t.classList.remove("active"); });
      tab.classList.add("active");
      mode = tab.dataset.mode;
      patrolIdx = 0;
      navPath = null;
      navPathIdx = 0;
      lastComputedMode = null;
      if (elStatus) {
        if (mode === "patrol") {
          elStatus.textContent = "Patrol AI — obstacle-aware waypoint navigation";
        } else if (mode === "astar") {
          elStatus.textContent = "A* — computing optimal path to goal...";
        } else if (mode === "dijkstra") {
          elStatus.textContent = "Dijkstra — computing shortest path to goal...";
        } else if (mode === "qaoa") {
          elStatus.textContent = "QAOA — running quantum path optimisation...";
        } else if (mode === "auto") {
          elStatus.textContent = "Auto — comparing all algorithms, selecting best...";
        } else {
          elStatus.textContent = "Manual — WASD / Arrow keys / on-screen pad";
        }
      }
      // Run pathfinding when switching to algorithm modes
      if (mode === "astar" || mode === "dijkstra" || mode === "qaoa" || mode === "auto") {
        computeAndShowPaths();
      }
    });
  });

  // ── compute and display all algorithm paths ─────────────────────────────────
  // Overall ranking: lowest energy wins; within 0.01 J it is a tie, broken by runtime.
  var ENERGY_TIE = 0.01;
  function byEnergyThenRuntime(a, b) {
    var de = a.energy - b.energy;
    return Math.abs(de) >= ENERGY_TIE ? de : a.runtime - b.runtime;
  }
  function rankAlgos(results) {
    return Object.keys(results).filter(function (a) { return results[a]; }).sort(function (a, b) {
      return byEnergyThenRuntime(results[a], results[b]);
    });
  }

  // Run all three planners from (sx, sz) to the goal on the current terrain.
  // Returns { paths: {dijkstra, astar, qaoa}, results: {...metrics + runtime} }.
  function runPlanners(sx, sz) {
    var planners = { dijkstra: dijkstraPath, astar: astarPath, qaoa: qaoaPath };
    var out = { paths: {}, results: {} };
    Object.keys(planners).forEach(function (name) {
      if (name === "qaoa") { qaoaState.running = true; qaoaState.progress = 0; }
      var t0 = performance.now();
      var path = planners[name](sx, sz, GOAL_X, GOAL_Z);
      var runtime = performance.now() - t0;
      out.paths[name] = path;
      out.results[name] = null;
      if (!path) return;
      var r = computePathEnergy(path);
      r.runtime = runtime;
      if (name === "qaoa") {
        r.qubits = qaoaState.qubits;
        r.shots = qaoaState.shots;
        r.reps = qaoaState.reps;
        r.expectation = qaoaState.expectation;
        r.validSamples = qaoaState.validSamples;
        r.randomValidSamples = qaoaState.randomValidSamples;
        r.selectedCount = qaoaState.selectedCount;
        r.corridorEdges = qaoaState.corridorEdges;
      }
      out.results[name] = r;
    });
    return out;
  }

  function computeAndShowPaths() {
    var run = runPlanners(rv.wx, rv.wz);
    var dPath = run.paths.dijkstra, aPath = run.paths.astar, qPath = run.paths.qaoa;
    pathResults.dijkstra = run.results.dijkstra;
    pathResults.astar = run.results.astar;
    pathResults.qaoa = run.results.qaoa;

    // draw all three paths
    drawPath(dPath, 0x34d399, "dijkstra");   // green
    drawPath(aPath, 0x60a5fa, "astar");      // blue
    drawPath(qPath, 0xc084fc, "qaoa");       // purple

    // set nav path for the active mode
    // Auto mode: compare all three and pick lowest energy
    var chosenAlgo = null;
    if (mode === "auto") {
      var candidates = [];
      if (dPath && pathResults.dijkstra) candidates.push({ name: "Dijkstra", path: dPath, energy: pathResults.dijkstra.energy, runtime: pathResults.dijkstra.runtime, color: 0x34d399 });
      if (aPath && pathResults.astar) candidates.push({ name: "A*", path: aPath, energy: pathResults.astar.energy, runtime: pathResults.astar.runtime, color: 0x60a5fa });
      if (qPath && pathResults.qaoa) candidates.push({ name: "QAOA", path: qPath, energy: pathResults.qaoa.energy, runtime: pathResults.qaoa.runtime, color: 0xc084fc });
      candidates.sort(byEnergyThenRuntime);
      if (candidates.length > 0) {
        var best = candidates[0];
        navPath = best.path;
        navPathIdx = 1;
        chosenAlgo = best.name;
        drawWaypointMarkers(best.path, best.color);
        if (elStatus) {
          elStatus.textContent = "Auto — " + best.name + " wins (" + best.energy.toFixed(2) + " J) — navigating optimal path";
        }
      }
    } else if (mode === "qaoa" && qPath) {
      navPath = qPath;
      navPathIdx = 1;
      drawWaypointMarkers(qPath, 0xc084fc);
    } else if (mode === "astar" && aPath) {
      navPath = aPath;
      navPathIdx = 1;
      drawWaypointMarkers(aPath, 0x60a5fa);
    } else if (mode === "dijkstra" && dPath) {
      navPath = dPath;
      navPathIdx = 1;
      drawWaypointMarkers(dPath, 0x34d399);
    }

    lastComputedMode = mode;

    // update the results panel
    updateResultsPanel(chosenAlgo);
  }

  // ── results panel DOM updates ──────────────────────────────────────────────
  function updateResultsPanel(chosenAlgo) {
    var dr = pathResults.dijkstra;
    var ar = pathResults.astar;
    var qr = pathResults.qaoa;

    // uphill / downhill rows (same layout for every algorithm)
    [["dijk", dr], ["astar", ar], ["qaoa", qr]].forEach(function (pair) {
      var r = pair[1];
      var set = function (suffix, text) {
        var el = document.getElementById("res-" + pair[0] + "-" + suffix);
        if (el) el.textContent = r ? text : "no path";
      };
      set("climb", r && r.climb.toFixed(2) + " m");
      set("desc", r && r.descent.toFixed(2) + " m");
      set("slope", r && r.maxSlopeDeg.toFixed(1) + "°");
      set("nodes", r && String(r.expanded));
    });

    var elDE = document.getElementById("res-dijk-energy");
    var elDD = document.getElementById("res-dijk-dist");
    var elDW = document.getElementById("res-dijk-wp");
    var elDT = document.getElementById("res-dijk-time");
    var elAE = document.getElementById("res-astar-energy");
    var elAD = document.getElementById("res-astar-dist");
    var elAW = document.getElementById("res-astar-wp");
    var elAT = document.getElementById("res-astar-time");
    var elComp = document.getElementById("res-comparison");

    if (dr) {
      if (elDE) elDE.textContent = dr.energy.toFixed(2) + " J";
      if (elDD) elDD.textContent = dr.distance.toFixed(2) + " m";
      if (elDW) elDW.textContent = dr.waypoints;
      if (elDT) elDT.textContent = dr.runtime.toFixed(2) + " ms";
    }
    if (ar) {
      if (elAE) elAE.textContent = ar.energy.toFixed(2) + " J";
      if (elAD) elAD.textContent = ar.distance.toFixed(2) + " m";
      if (elAW) elAW.textContent = ar.waypoints;
      if (elAT) elAT.textContent = ar.runtime.toFixed(2) + " ms";
    }
    if (dr && ar && elComp) {
      var faster = ar.runtime < dr.runtime ? "A*" : "Dijkstra";
      var speedup = ar.runtime < dr.runtime
        ? (dr.runtime / Math.max(ar.runtime, 0.01)).toFixed(1)
        : (ar.runtime / Math.max(dr.runtime, 0.01)).toFixed(1);
      var eDelta = Math.abs(ar.energy - dr.energy).toFixed(2);
      var identical = Math.abs(ar.energy - dr.energy) < 0.01;
      elComp.innerHTML =
        (identical
          ? '<span class="res-match">✓ Paths identical</span>'
          : '<span class="res-delta">ΔE = ' + eDelta + ' J</span>') +
        ' · <span class="res-speed">' + faster + ' ' + speedup + '× faster</span>';
    }

    // QAOA results
    var elQE = document.getElementById("res-qaoa-energy");
    var elQD = document.getElementById("res-qaoa-dist");
    var elQW = document.getElementById("res-qaoa-wp");
    var elQT = document.getElementById("res-qaoa-time");
    var elQQb = document.getElementById("res-qaoa-qubits");
    var elQSh = document.getElementById("res-qaoa-shots");
    var elQRp = document.getElementById("res-qaoa-reps");
    var elQEx = document.getElementById("res-qaoa-expect");
    var elQVs = document.getElementById("res-qaoa-valid");
    var elQComp = document.getElementById("res-qaoa-comparison");

    if (qr) {
      if (elQE) elQE.textContent = qr.energy.toFixed(2) + " J";
      if (elQD) elQD.textContent = qr.distance.toFixed(2) + " m";
      if (elQW) elQW.textContent = qr.waypoints;
      if (elQT) elQT.textContent = qr.runtime.toFixed(2) + " ms";
      if (elQQb) elQQb.textContent = qr.qubits;
      if (elQSh) elQSh.textContent = qr.shots;
      if (elQRp) elQRp.textContent = "p = " + qr.reps;
      if (elQEx) elQEx.textContent = qr.expectation.toFixed(2) + " J";
      if (elQVs) elQVs.textContent = qr.validSamples + " / " + qr.shots + " (random " + qr.randomValidSamples + ")";
    }
    if (!qr && elQComp && qaoaState.error) {
      // QAOA has no classical fallback: say why there is no QAOA path
      elQComp.innerHTML = '<span class="res-delta">No QAOA path: ' + qaoaState.error + '</span>';
    }
    if (qr && ar && elQComp) {
      var qDelta = qr.energy - ar.energy;
      var qBetter = qDelta < -0.01;
      var qSame = Math.abs(qDelta) < 0.01;
      if (qSame) {
        elQComp.innerHTML = '<span class="res-match">✓ Same energy as A*</span>';
      } else if (qBetter) {
        elQComp.innerHTML = '<span class="res-quantum-win">⚛ QAOA saves ' + Math.abs(qDelta).toFixed(2) + ' J vs A*</span>';
      } else {
        elQComp.innerHTML = '<span class="res-delta">A* wins by ' + qDelta.toFixed(2) + ' J (shallow p)</span>';
      }
    }

    // Auto-mode winner badge
    var elAutoWinner = document.getElementById("auto-winner");
    if (elAutoWinner) {
      if (chosenAlgo) {
        var colorMap = { "Dijkstra": "#34d399", "A*": "#60a5fa", "QAOA": "#c084fc" };
        var iconMap = { "Dijkstra": "⚡", "A*": "⚡", "QAOA": "⚛" };
        elAutoWinner.innerHTML =
          '<span class="winner-icon" style="color:' + (colorMap[chosenAlgo] || "#fff") + '">' +
          (iconMap[chosenAlgo] || "") + '</span> ' +
          '<strong style="color:' + (colorMap[chosenAlgo] || "#fff") + '">' + chosenAlgo + '</strong>' +
          ' selected — lowest energy (ties → fastest)';
        elAutoWinner.style.display = "block";
      } else if (dr || ar || qr) {
        // Not in auto mode but paths computed — show comparative summary
        var allResults = [];
        if (dr) allResults.push({ name: "Dijkstra", energy: dr.energy, runtime: dr.runtime });
        if (ar) allResults.push({ name: "A*", energy: ar.energy, runtime: ar.runtime });
        if (qr) allResults.push({ name: "QAOA", energy: qr.energy, runtime: qr.runtime });
        allResults.sort(byEnergyThenRuntime);
        var best = allResults[0];
        var colorMap2 = { "Dijkstra": "#34d399", "A*": "#60a5fa", "QAOA": "#c084fc" };
        elAutoWinner.innerHTML =
          'Best: <strong style="color:' + (colorMap2[best.name] || "#fff") + '">' +
          best.name + '</strong> (' + best.energy.toFixed(2) + ' J)' +
          ' — use <em>Auto</em> to navigate it';
        elAutoWinner.style.display = "block";
      } else {
        elAutoWinner.style.display = "none";
      }
    }
  }

  // ── terrain switching ──────────────────────────────────────────────────────
  var elTerrainDesc = document.getElementById("terrain-desc");
  var elTerrainStat = document.getElementById("stat-terrain");

  function setTerrain(id) {
    if (!TERRAINS[id]) return;
    terrainId = id;
    rebuildTerrainMesh();

    // re-seat everything that sits on the ground
    obstacles.forEach(function (o) {
      var oy = surfaceAt(o.x, o.z);
      o.mesh.position.y = oy + o.lift;
      if (o.ring) o.ring.position.y = oy + 0.05;
    });
    var gy = surfaceAt(GOAL_X, GOAL_Z);
    goalGroup.position.y = gy;
    goalLight.position.y = gy + 1;

    // every terrain run starts from spawn with fresh odometry/energy counters
    rv.wx = START_X; rv.wz = START_Z; rv.yaw = 0; rv.speed = 0; rv.turn = 0;
    rv.wy = surfaceAt(rv.wx, rv.wz);
    var enc = sensors.encoders;
    enc.odomX = enc.prevX = START_X; enc.odomZ = enc.prevZ = START_Z; enc.odomTheta = 0;
    enc.totalDistance = 0; enc.climb = 0; enc.descent = 0;
    enc.prevH = rv.wy;
    sensors.energy.consumedJ = 0;
    sensors.energy.batteryJ = sensors.energy.capacityWh * 3600;
    patrolIdx = 0;
    clearTracks();

    document.querySelectorAll(".terrain-btn").forEach(function (b) {
      b.classList.toggle("active", b.dataset.terrain === id);
    });
    if (elTerrainDesc) elTerrainDesc.textContent = TERRAINS[id].desc;
    if (elTerrainStat) elTerrainStat.textContent = TERRAINS[id].label;

    navPath = null;
    navPathIdx = 0;
    lastComputedMode = null;
    if (mode === "astar" || mode === "dijkstra" || mode === "qaoa" || mode === "auto") {
      computeAndShowPaths();
    } else {
      ["dijkstra", "astar", "qaoa"].forEach(function (n) { drawPath(null, 0, n); });
      drawWaypointMarkers(null);
    }
    if (elStatus) elStatus.textContent = "Terrain: " + TERRAINS[id].label + " — " + TERRAINS[id].desc;
  }

  document.querySelectorAll(".terrain-btn").forEach(function (btn) {
    btn.addEventListener("click", function () { setTerrain(btn.dataset.terrain); });
  });

  // ── terrain benchmark: every planner on every terrain, spawn → goal ────────
  var ALGO_NAMES = { dijkstra: "Dijkstra", astar: "A*", qaoa: "QAOA" };
  var ALGO_COLORS = { dijkstra: "#34d399", astar: "#60a5fa", qaoa: "#c084fc" };

  function benchmarkTerrains() {
    var saved = terrainId;
    var rows = [];
    Object.keys(TERRAINS).forEach(function (id) {
      terrainId = id;   // heightAt/edgeCost follow terrainId; the mesh is left alone
      rows.push({ terrain: id, results: runPlanners(START_X, START_Z).results });
    });
    terrainId = saved;

    var algos = Object.keys(ALGO_NAMES);
    var summary = {};
    algos.forEach(function (a) { summary[a] = { wins: 0, overall: 0, runtime: 0, nodes: 0, n: 0 }; });
    rows.forEach(function (row) {
      var best = Infinity;
      algos.forEach(function (a) {
        var r = row.results[a];
        if (!r) return;
        best = Math.min(best, r.energy);
        summary[a].runtime += r.runtime;
        summary[a].nodes += r.expanded;
        summary[a].n++;
      });
      row.best = best;
      row.overall = rankAlgos(row.results)[0];
      if (row.overall) summary[row.overall].overall++;
      // ties (within 0.01 J) count as a win for every tied algorithm
      algos.forEach(function (a) {
        var r = row.results[a];
        if (r && r.energy - best < 0.01) summary[a].wins++;
      });
    });
    renderBenchmark(rows, summary);
    return { rows: rows, summary: summary };
  }

  function renderBenchmark(rows, summary) {
    var el = document.getElementById("bench-out");
    if (!el) return;
    var algos = Object.keys(ALGO_NAMES);
    var html = '<table class="bench-table"><thead><tr><th>Terrain</th>';
    algos.forEach(function (a) {
      html += '<th style="color:' + ALGO_COLORS[a] + '">' + ALGO_NAMES[a] + '</th>';
    });
    html += '</tr></thead><tbody>';
    rows.forEach(function (row) {
      html += '<tr><td>' + TERRAINS[row.terrain].label + '</td>';
      algos.forEach(function (a) {
        var r = row.results[a];
        if (!r) { html += '<td class="bench-fail">—</td>'; return; }
        var win = r.energy - row.best < ENERGY_TIE;
        html += '<td class="' + (win ? "bench-win" : "") + (a === row.overall ? " bench-overall" : "") + '" title="climb ' + r.climb.toFixed(2) +
          ' m · descent ' + r.descent.toFixed(2) + ' m · max ' + r.maxSlopeDeg.toFixed(1) + '°">' +
          r.energy.toFixed(1) + '</td>';
      });
      html += '</tr>';
    });
    html += '</tbody><tfoot>';
    [["Overall", function (s) { return s.overall + "/" + rows.length; }],
     ["Lowest E", function (s) { return s.wins + "/" + rows.length; }],
     ["Avg ms", function (s) { return s.n ? (s.runtime / s.n).toFixed(1) : "—"; }],
     ["Nodes", function (s) { return s.n ? Math.round(s.nodes / s.n) : "—"; }]].forEach(function (f) {
      html += '<tr><td>' + f[0] + '</td>';
      algos.forEach(function (a) { html += '<td>' + f[1](summary[a]) + '</td>'; });
      html += '</tr>';
    });
    var champ = algos.slice().sort(function (a, b) {
      return summary[b].overall - summary[a].overall || summary[a].runtime - summary[b].runtime;
    })[0];
    html += '</tfoot></table>';
    html += '<div class="bench-champ">🏆 Best overall: <strong style="color:' + ALGO_COLORS[champ] + '">' +
      ALGO_NAMES[champ] + '</strong> (' + summary[champ].overall + '/' + rows.length + ' terrains)</div>';
    html += '<p class="hint">Energy in J, spawn → goal. Bold = lowest energy; ★ = overall winner ' +
      '(lowest energy, ties → fastest). Hover a cell for climb/descent.</p>';
    el.innerHTML = html;
  }

  var benchBtn = document.getElementById("btn-benchmark");
  if (benchBtn) benchBtn.addEventListener("click", function () {
    benchBtn.disabled = true;
    benchBtn.textContent = "Running…";
    // let the button repaint before the (blocking) searches run
    setTimeout(function () {
      benchmarkTerrains();
      benchBtn.disabled = false;
      benchBtn.textContent = "Benchmark all terrains";
    }, 30);
  });

  window.roverSim = {
    setTerrain: setTerrain, benchmarkTerrains: benchmarkTerrains, runPlanners: runPlanners,
    start: [START_X, START_Z], goal: [GOAL_X, GOAL_Z], extent: GRID_MAX,
    rover: function () { return [rv.wx, rv.wz]; },
    setLunarHour: setLunarHour, get env() { return env; }, surfaceAt: surfaceAt
  };

  function setCamMode(m) {
    camMode = m;
    document.querySelectorAll(".cam-btn[data-cam]").forEach(function (b) {
      b.classList.toggle("active", b.dataset.cam === m);
    });
  }
  document.querySelectorAll(".cam-btn[data-cam]").forEach(function (btn) {
    btn.addEventListener("click", function () { setCamMode(btn.dataset.cam); });
  });

  // stop HUD scroll from zooming the 3D camera
  document.querySelectorAll(".hud").forEach(function (hud) {
    hud.addEventListener("wheel", function (e) { e.stopPropagation(); });
  });

  // Dragging or scrolling on the 3D view works in every camera mode: from Chase
  // or Top it switches to Orbit, starting from where the camera is right now so
  // the view does not jump.
  var ORBIT_MIN_EL = 0.03, ORBIT_MAX_EL = 1.45, ORBIT_MIN_R = 2.5, ORBIT_MAX_R = 60;
  function enterOrbitFromCurrentView() {
    if (camMode === "orbit") return;
    var dx = camera.position.x - rv.wx, dz = camera.position.z - rv.wz;
    var dy = camera.position.y - (rv.wy + 0.4);
    var r = Math.sqrt(dx * dx + dy * dy + dz * dz);
    orbit.radius = Math.max(ORBIT_MIN_R, Math.min(ORBIT_MAX_R, r));
    orbit.elevation = Math.max(ORBIT_MIN_EL, Math.min(ORBIT_MAX_EL, Math.asin(Math.max(-1, Math.min(1, dy / (r || 1))))));
    orbit.azimuth = Math.atan2(dx, dz);
    setCamMode("orbit");
  }

  wrap.addEventListener("pointerdown", function (e) {
    if (e.button !== undefined && e.button !== 0) return;
    e.preventDefault();
    enterOrbitFromCurrentView();
    orbit.active = true; orbit.startX = e.clientX; orbit.startY = e.clientY;
  });
  window.addEventListener("pointermove", function (e) {
    if (!orbit.active) return;
    orbit.azimuth -= (e.clientX - orbit.startX) * 0.006;
    orbit.elevation -= (e.clientY - orbit.startY) * 0.004;
    orbit.elevation = Math.max(ORBIT_MIN_EL, Math.min(ORBIT_MAX_EL, orbit.elevation));
    orbit.startX = e.clientX; orbit.startY = e.clientY;
  });
  function endDrag() { orbit.active = false; }
  window.addEventListener("pointerup", endDrag);
  window.addEventListener("pointercancel", endDrag);
  window.addEventListener("blur", endDrag);
  wrap.addEventListener("wheel", function (e) {
    e.preventDefault();
    enterOrbitFromCurrentView();
    orbit.radius = Math.max(ORBIT_MIN_R, Math.min(ORBIT_MAX_R, orbit.radius * Math.exp(e.deltaY * 0.001)));
  }, { passive: false });

  // ── physics / drive ────────────────────────────────────────────────────────
  // grade (rise/run) along the rover's heading, sampled ±0.3 m around it
  function headingGrade() {
    var sx = Math.sin(rv.yaw) * 0.3, sz = Math.cos(rv.yaw) * 0.3;
    return (surfaceAt(rv.wx + sx, rv.wz + sz) - surfaceAt(rv.wx - sx, rv.wz - sz)) / 0.6;
  }

  // Loose regolith: the wheels always slip a little, and much more when climbing.
  // rv.speed is the WHEEL speed (what the encoders see); the rover's true ground
  // speed is lower, so wheel odometry slowly drifts from the true position.
  var slip = 0, groundSpeed = 0;
  function advance(dt) {
    rv.yaw += rv.turn * dt;
    var g = headingGrade() * (rv.speed >= 0 ? 1 : -1);   // grade along the direction of motion
    slip = Math.min(0.6, 0.03 + (g > 0 ? 1.1 * Math.pow(g, 1.25) : 0));
    groundSpeed = rv.speed * (1 - slip);
    rv.wx += Math.sin(rv.yaw) * groundSpeed * dt;
    rv.wz += Math.cos(rv.yaw) * groundSpeed * dt;
  }

  // climbing slows the rover down; descending does not speed it up
  function slopeSpeedCap(grade) {
    return grade > 0 ? maxSpeed / gradeFactor(Math.min(grade, MAX_GRADE)) : maxSpeed;
  }

  var slopeBlocked = false;

  function drive(dt) {
    var oldX = rv.wx, oldZ = rv.wz;
    var hg = headingGrade();
    var capFwd = slopeSpeedCap(hg), capRev = slopeSpeedCap(-hg);

    if (mode === "astar" || mode === "dijkstra" || mode === "qaoa" || mode === "auto") {
      // follow computed path
      if (!navPath || navPathIdx >= navPath.length) {
        rv.speed *= 0.95;
        rv.turn *= 0.9;
        // if we haven't computed yet, do it
        if (!navPath && lastComputedMode !== mode) {
          computeAndShowPaths();
        }
        // update physics anyway
        advance(dt);
      } else {
        var wp = navPath[navPathIdx];
        var dx = wp[0] - rv.wx, dz = wp[1] - rv.wz;
        var dist = Math.sqrt(dx * dx + dz * dz);
        if (dist < 0.4) {
          navPathIdx++;
          return;
        }
        var targetYaw = Math.atan2(dx, dz);
        var err = targetYaw - rv.yaw;
        while (err > Math.PI) err -= 2 * Math.PI;
        while (err < -Math.PI) err += 2 * Math.PI;
        var targetTurn = Math.max(-1.5, Math.min(1.5, err * 2.5));
        var targetSpd = Math.abs(err) > 0.6 ? 0.1 : Math.min(capFwd, dist * 0.5);
        rv.turn += (targetTurn - rv.turn) * Math.min(1, dt * 6);
        rv.speed += (targetSpd - rv.speed) * Math.min(1, dt * 4);

        advance(dt);
      }
    } else if (mode === "patrol") {
      if (patrolIdx >= PATROL_WPS.length) patrolIdx = 0;
      var wp = PATROL_WPS[patrolIdx];
      var dx = wp[0] - rv.wx, dz = wp[1] - rv.wz;
      var dist = Math.sqrt(dx * dx + dz * dz);
      if (dist < 0.5) { patrolIdx++; return; }

      // local obstacle avoidance during patrol
      var targetYaw = Math.atan2(dx, dz);

      // check if an obstacle is in the direct path — steer around it
      var lookDist = 2.5;
      var lookX = rv.wx + Math.sin(targetYaw) * lookDist;
      var lookZ = rv.wz + Math.cos(targetYaw) * lookDist;
      var lookCol = checkCollision(lookX, lookZ);
      if (lookCol.hit) {
        // steer away: offset target yaw by ±45° away from obstacle
        var obsDx = lookCol.obstacle.x - rv.wx;
        var obsDz = lookCol.obstacle.z - rv.wz;
        var obsAngle = Math.atan2(obsDx, obsDz);
        var angleDiff = targetYaw - obsAngle;
        while (angleDiff > Math.PI) angleDiff -= 2 * Math.PI;
        while (angleDiff < -Math.PI) angleDiff += 2 * Math.PI;
        // steer to the side that's more toward our target
        targetYaw = obsAngle + (angleDiff >= 0 ? 1.2 : -1.2);
      }

      var err = targetYaw - rv.yaw;
      while (err > Math.PI) err -= 2 * Math.PI;
      while (err < -Math.PI) err += 2 * Math.PI;
      var targetTurn = Math.max(-1.5, Math.min(1.5, err * 2.5));
      var targetSpd = Math.abs(err) > 0.6 ? 0.1 : Math.min(capFwd, dist * 0.5);
      rv.turn += (targetTurn - rv.turn) * Math.min(1, dt * 6);
      rv.speed += (targetSpd - rv.speed) * Math.min(1, dt * 4);

      advance(dt);
    } else {
      var fwd = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
      var trn = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
      if (keys[" "]) { fwd = 0; trn = 0; }
      var tspd = fwd > 0 ? capFwd : fwd < 0 ? -capRev : 0;
      var ttrn = trn * 1.6;
      rv.speed += (tspd - rv.speed) * Math.min(1, dt * 7);
      rv.turn += (ttrn - rv.turn) * Math.min(1, dt * 8);

      advance(dt);
    }

    // ── collision resolution (all modes) ─────────────────────────────────────
    var resolved = resolveCollision(rv.wx, rv.wz);
    if (resolved.x !== rv.wx || resolved.z !== rv.wz) {
      rv.wx = resolved.x;
      rv.wz = resolved.z;
      // dampen speed on collision
      rv.speed *= 0.3;
    }

    // ── slope limit: refuse moves steeper than MAX_SLOPE_DEG ─────────────────
    var mdx = rv.wx - oldX, mdz = rv.wz - oldZ;
    var md = Math.sqrt(mdx * mdx + mdz * mdz);
    slopeBlocked = false;
    if (md > 1e-6) {
      // sample over ≥0.3 m so tiny per-frame steps don't amplify noise
      var probe = Math.max(md, 0.3);
      var ux = mdx / md, uz = mdz / md;
      // inside the planning area use the planners' elevation model so planned paths
      // never stall; outside it the real surface (big crater walls, massifs) applies
      var hf = (Math.abs(oldX) <= GRID_MAX && Math.abs(oldZ) <= GRID_MAX) ? heightAt : surfaceAt;
      var moveGrade = (hf(oldX + ux * probe, oldZ + uz * probe) - hf(oldX, oldZ)) / probe;
      // 10% headroom over the planning limit so planned paths never stall between grid points
      if (Math.abs(moveGrade) > MAX_GRADE * 1.1) {
        rv.wx = oldX;
        rv.wz = oldZ;
        rv.speed *= 0.3;
        slopeBlocked = true;
      }
    }

    // wheel spin
    if (slopeBlocked) groundSpeed = 0;
    rv.wheelAngle += rv.speed * dt / WHEEL_R;
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
      var behind = 7, above = 2.4;   // low enough to keep the horizon, Earth and sky in view
      var tx = rx - Math.sin(rv.yaw) * behind;
      var ty = ry + above;
      var tz = rz - Math.cos(rv.yaw) * behind;
      camera.position.x += (tx - camera.position.x) * 0.06;
      camera.position.y += (ty - camera.position.y) * 0.05;
      camera.position.z += (tz - camera.position.z) * 0.06;
      camera.lookAt(rx, ry + 0.9, rz);
    }
  }

  // ── SENSOR SIMULATION ──────────────────────────────────────────────────────
  // Mirrors the ROS2 sensor_pkg: LiDAR, IMU, Encoders, Odometry, Energy model

  var sensors = {
    // ── LiDAR (lidar_processor.py) ─────────────────────────────────────────
    // 2D laser scan: casts rays from rover, reports distances to obstacles/terrain
    lidar: {
      NUM_RAYS: 36,            // 360° / 10° per ray
      MAX_RANGE: 8.0,          // meters
      MIN_RANGE: 0.15,
      ranges: [],              // latest range readings
      nearestRange: 8.0,       // closest obstacle distance
      nearestAngle: 0,         // angle to closest obstacle (deg)
      pointCloud: [],          // computed (x,z) points in world frame
      update: function () {
        this.ranges = [];
        this.pointCloud = [];
        this.nearestRange = this.MAX_RANGE;
        for (var i = 0; i < this.NUM_RAYS; i++) {
          var angle = rv.yaw + (i / this.NUM_RAYS) * Math.PI * 2;
          var range = this.MAX_RANGE;

          // ray-march against obstacles
          for (var step = this.MIN_RANGE; step <= this.MAX_RANGE; step += 0.2) {
            var px = rv.wx + Math.sin(angle) * step;
            var pz = rv.wz + Math.cos(angle) * step;
            var col = checkCollision(px, pz);
            if (col.hit) {
              range = Math.max(this.MIN_RANGE, step - 0.1);
              break;
            }
          }

          this.ranges.push(range);
          // build point cloud in world frame (like lidar_processor.py PointCloud2)
          if (range < this.MAX_RANGE) {
            this.pointCloud.push({
              x: rv.wx + Math.sin(angle) * range,
              z: rv.wz + Math.cos(angle) * range
            });
          }
          if (range < this.nearestRange) {
            this.nearestRange = range;
            this.nearestAngle = ((angle * 57.2958) % 360 + 360) % 360;
          }
        }
      }
    },

    // ── IMU (imu_processor.py) ─────────────────────────────────────────────
    // Derives slope from accelerometer: atan2(sqrt(ax²+ay²), |az|)
    imu: {
      ax: 0, ay: 0, az: 1.62,  // lunar gravity
      slope: 0,                  // radians
      slopeDeg: 0,               // degrees
      update: function () {
        // compute terrain slope under rover (matches imu_processor.py formula)
        var dx = surfaceAt(rv.wx + 0.3, rv.wz) - surfaceAt(rv.wx - 0.3, rv.wz);
        var dz = surfaceAt(rv.wx, rv.wz + 0.3) - surfaceAt(rv.wx, rv.wz - 0.3);
        var slopeX = dx / 0.6;  // rise/run
        var slopeZ = dz / 0.6;

        // simulated accelerometer in rover body frame (lunar gravity = 1.62 m/s²)
        this.ax = -slopeX * 1.62;
        this.ay = -slopeZ * 1.62;
        this.az = 1.62 * Math.cos(Math.atan(Math.sqrt(slopeX * slopeX + slopeZ * slopeZ)));

        // slope = atan2(sqrt(ax² + ay²), |az|) — from imu_processor.py
        this.slope = Math.atan2(Math.sqrt(this.ax * this.ax + this.ay * this.ay), Math.abs(this.az));
        this.slopeDeg = this.slope * 57.2958;

        // pitch along the heading: + nose up (uphill), − nose down (downhill)
        this.grade = headingGrade();
        this.pitchDeg = Math.atan(this.grade) * 57.2958;
      }
    },

    // ── Encoders / Odometry (encoder_processor.py + diff_drive_odometry.py) ─
    // Differential drive: 6 wheels, track left/right averages
    encoders: {
      wheelRadius: 0.18,       // meters (matches encoder_processor.py)
      wheelSeparation: 0.84,   // meters (matches encoder_processor.py)
      leftAngle: 0,            // cumulative left wheel angle (rad)
      rightAngle: 0,
      odomX: START_X, odomZ: START_Z,  // dead-reckoned position
      odomTheta: 0,            // dead-reckoned heading
      linearVel: 0,            // estimated linear velocity (m/s)
      angularVel: 0,           // estimated angular velocity (rad/s)
      totalDistance: 0,         // cumulative odometry distance (m)
      climb: 0, descent: 0,     // cumulative elevation gained / lost (m)
      prevX: START_X, prevZ: START_Z, prevH: 0,

      update: function (dt) {
        if (dt <= 0) return;

        // compute wheel angular deltas from rover speed & turn
        // differential drive: v_left = v - w*L/2, v_right = v + w*L/2
        var vLeft = rv.speed - rv.turn * this.wheelSeparation / 2;
        var vRight = rv.speed + rv.turn * this.wheelSeparation / 2;

        var dLeft = vLeft * dt;  // arc distance left wheels
        var dRight = vRight * dt;

        // accumulate wheel angles (joint_states)
        this.leftAngle += dLeft / this.wheelRadius;
        this.rightAngle += dRight / this.wheelRadius;

        // diff_drive_odometry.py midpoint integration
        var dCenter = (dLeft + dRight) / 2.0;
        var dTheta = (dRight - dLeft) / this.wheelSeparation;

        this.odomTheta += dTheta;
        // wrap to (-π, π]
        while (this.odomTheta > Math.PI) this.odomTheta -= 2 * Math.PI;
        while (this.odomTheta <= -Math.PI) this.odomTheta += 2 * Math.PI;

        this.odomX += dCenter * Math.sin(this.odomTheta - dTheta / 2);
        this.odomZ += dCenter * Math.cos(this.odomTheta - dTheta / 2);

        this.linearVel = rv.speed;
        this.angularVel = rv.turn;

        // accumulate distance (energy_model.py DistanceTracker)
        var ddx = rv.wx - this.prevX, ddz = rv.wz - this.prevZ;
        this.totalDistance += Math.sqrt(ddx * ddx + ddz * ddz);
        this.prevX = rv.wx;
        this.prevZ = rv.wz;

        // cumulative elevation gained / lost
        var h = surfaceAt(rv.wx, rv.wz);
        var dh = h - this.prevH;
        if (dh > 0) this.climb += dh; else this.descent -= dh;
        this.prevH = h;
      }
    },

    // ── Energy Model (energy_model.py) ─────────────────────────────────────
    // P = P_idle + k_linear * |v| + k_angular * |ω|
    energy: {
      capacityWh: 40,          // Wh (matches energy_model.py)
      voltage: 24,             // V
      pIdle: 3.0,              // Watts idle
      kLinear: 15.0,           // W/(m/s)
      kAngular: 4.0,           // W/(rad/s)
      consumedJ: 0,            // total energy consumed (Joules)
      currentPower: 3.0,       // instantaneous power (W)
      currentA: 0.125,         // instantaneous current (A)
      soc: 100,                // state of charge %

      mass: 40,                // kg
      regenEff: 0.3,           // fraction of downhill gravity power recovered
      gravityPower: 0,         // W, + climbing, − regenerating
      solarW: 0,               // W harvested by the solar deck
      heaterW: 0,              // W spent by survival heaters in the cold
      batteryJ: 40 * 3600,     // stored energy (J)
      netW: 0,                 // + charging, − discharging

      update: function (dt) {
        // P_grav = m·g·v·sinθ along the heading, using the TRUE ground speed
        var pg = this.mass * G_MOON * groundSpeed * Math.sin(Math.atan(sensors.imu.grade || 0));
        this.gravityPower = pg < 0 ? pg * this.regenEff : pg;
        this.currentPower = Math.max(this.pIdle * 0.5, this.pIdle +
          this.kLinear * Math.abs(rv.speed) +
          this.kAngular * Math.abs(rv.turn) +
          this.gravityPower);
        this.consumedJ += this.currentPower * dt;
        this.currentA = this.currentPower / this.voltage;
        this.solarW = env.solarW;
        this.heaterW = env.heaterW;
        this.netW = this.solarW - this.currentPower - this.heaterW;
        var capacityJ = this.capacityWh * 3600;
        this.batteryJ = Math.max(0, Math.min(capacityJ, this.batteryJ + this.netW * dt));
        this.soc = this.batteryJ / capacityJ * 100;
      }
    }
  };

  // ── LiDAR visualization (optional ray lines) ──────────────────────────────
  var lidarLines = null;
  var lidarDotsGroup = null;

  function updateLidarViz() {
    // remove old
    if (lidarLines) { scene.remove(lidarLines); lidarLines.geometry.dispose(); lidarLines.material.dispose(); }
    if (lidarDotsGroup) { scene.remove(lidarDotsGroup); }

    var lidar = sensors.lidar;
    if (lidar.ranges.length === 0) return;

    // ray lines
    var pts = [];
    var ry = rv.wy + 0.45;
    for (var i = 0; i < lidar.NUM_RAYS; i++) {
      var angle = rv.yaw + (i / lidar.NUM_RAYS) * Math.PI * 2;
      var range = lidar.ranges[i];
      pts.push(new THREE.Vector3(rv.wx, ry, rv.wz));
      pts.push(new THREE.Vector3(
        rv.wx + Math.sin(angle) * range,
        ry,
        rv.wz + Math.cos(angle) * range
      ));
    }
    var geo = new THREE.BufferGeometry().setFromPoints(pts);
    var mat = new THREE.LineBasicMaterial({ color: 0x00ff88, transparent: true, opacity: 0.15 });
    lidarLines = new THREE.LineSegments(geo, mat);
    scene.add(lidarLines);

    // hit-point dots
    lidarDotsGroup = new THREE.Group();
    var dotGeo = new THREE.SphereGeometry(0.06, 4, 4);
    var dotMat = new THREE.MeshBasicMaterial({ color: 0x00ff88 });
    for (var j = 0; j < lidar.pointCloud.length; j++) {
      var pt = lidar.pointCloud[j];
      var dot = new THREE.Mesh(dotGeo, dotMat);
      dot.position.set(pt.x, ry, pt.z);
      lidarDotsGroup.add(dot);
    }
    scene.add(lidarDotsGroup);
  }

  // ── LUNAR ENVIRONMENT ──────────────────────────────────────────────────────
  // Local solar time on a 24 "lunar hour" clock (sunrise 06:00, noon 12:00,
  // sunset 18:00). One lunar hour lasts ~29.5 Earth hours, so the slider and the
  // time-lapse are the only ways to see the day go by.
  var env = {
    lunarHour: 7.2,
    timelapse: false,
    sunDir: new THREE.Vector3(),
    sunElevDeg: 0,
    sunUp: 1,            // 0 = night, 1 = full disc above the horizon
    sunlit: true,        // rover in direct sunlight (not shadowed by terrain / rocks)
    tempK: 290,          // surface temperature at the rover
    solarW: 0,
    heaterW: 0,
    doseRate: GCR_DOSE_USV_H,
    doseUSv: 0
  };

  // march from the rover toward the sun over the real surface and the boulders
  function inShadowAt(x, z, y0, dir) {
    if (dir.y <= 0) return true;
    var hl = Math.sqrt(dir.x * dir.x + dir.z * dir.z);
    if (hl < 1e-4) return false;
    var ux = dir.x / hl, uz = dir.z / hl, tanE = dir.y / hl;
    for (var s = 0.5; s < 420; s += Math.max(0.35, s * 0.06)) {
      var px = x + ux * s, pz = z + uz * s, rayY = y0 + s * tanE;
      if (rayY > 70) return false;               // above the highest massif
      if (surfaceAt(px, pz) > rayY) return true;
      if (s < 12) {
        for (var i = 0; i < obstacles.length; i++) {
          var o = obstacles[i], dx = px - o.x, dz = pz - o.z;
          if (dx * dx + dz * dz < o.radius * o.radius && o.mesh.position.y - o.lift + o.top > rayY) return true;
        }
      }
    }
    return false;
  }

  var shadowCheckCounter = 0;
  var panelNormal = new THREE.Vector3();
  function updateEnvironment(dt) {
    if (env.timelapse) {
      env.lunarHour = (env.lunarHour + dt * 0.5) % 24;   // a whole lunar day in 48 s
      if (elHourSlider) elHourSlider.value = env.lunarHour.toFixed(2);
    }
    var d = sunDirection(env.lunarHour, env.sunDir);
    env.sunElevDeg = Math.asin(d.y) * 180 / Math.PI;
    // the 0.53° disc rises / sets; there is no twilight without an atmosphere
    var e = env.sunElevDeg;
    env.sunUp = e <= -0.3 ? 0 : e >= 0.3 ? 1 : (e + 0.3) / 0.6;

    // light rig follows the rover so shadows stay crisp wherever it drives
    sun.target.position.set(rv.wx, rv.wy, rv.wz);
    sun.position.copy(sun.target.position).addScaledVector(d, 160);
    sun.intensity = 3.0 * env.sunUp;
    sun.castShadow = env.sunUp > 0;
    var sinE = Math.max(0, d.y);
    hemi.intensity = 0.05 + env.sunUp * (0.22 + 0.35 * sinE);   // regolith bounce grows with sun height
    earthshine.target.position.copy(sun.target.position);
    earthshine.position.copy(sun.target.position).addScaledVector(EARTH_DIR, 100);
    // earthshine: sunlight reflected by Earth, strongest when Earth is "full"
    // (on the Moon it is ~50× brighter than full moonlight on Earth)
    var earthPhase = 0.5 * (1 - EARTH_DIR.dot(d));
    earthshine.intensity = (1 - env.sunUp) * 0.45 * earthPhase + 0.02;
    // camera auto-exposure, like a real rover navcam: shorter at noon, longer at night
    renderer.toneMappingExposure = Math.max(0.4, Math.min(2.2, 1.2 / (3.0 * sinE * env.sunUp + 0.3)));
    earthMat.uniforms.sunDir.value.copy(d);
    sunSprite.position.copy(d).multiplyScalar(SKY_R);
    sunSprite.visible = e > -0.6;
    starMat.opacity = 1 - 0.55 * env.sunUp;   // stars wash out of the camera exposure in daylight

    // is the rover itself in sunlight? (checked every few frames)
    if (shadowCheckCounter++ % 6 === 0) {
      env.sunlit = env.sunUp > 0 && !inShadowAt(rv.wx, rv.wz, rv.wy + 0.57, d);
    }
    var lightsOn = env.sunUp < 0.5 || !env.sunlit;
    rovLight.intensity = lightsOn ? 3.0 : 0;
    lampMat.emissive.setHex(lightsOn ? 0xfff6e0 : 0x000000);

    // surface temperature: ~ +110 °C at lunar noon, ~ −175 °C before dawn
    // (LRO Diviner); shaded regolith cools fast with no air to hold heat
    var sinceSunset = ((env.lunarHour - 18) % 24 + 24) % 24;          // 0..24
    var tNight = 95 + 30 * Math.max(0, 1 - sinceSunset / 12);         // K, cools all night
    var tDay = 392 * Math.pow(sinE, 0.25);
    var target = env.sunlit ? Math.max(tNight, tDay) : tNight + 0.3 * Math.max(0, tDay - tNight);
    env.tempK += (target - env.tempK) * Math.min(1, dt * (env.timelapse ? 4 : 0.8));

    // solar power on the deck, which tilts with the rover
    panelNormal.set(0, 1, 0).applyQuaternion(rv.group.quaternion);
    env.solarW = env.sunlit ? SOLAR_CONST * PANEL_AREA * PANEL_EFF * Math.max(0, panelNormal.dot(d)) * env.sunUp : 0;
    // survival heaters keep the electronics box above −40 °C
    var tC = env.tempK - 273.15;
    env.heaterW = tC < -40 ? Math.min(14, (-40 - tC) * 0.1) : 0;

    // galactic cosmic rays: no magnetic field or atmosphere to shield the rover
    env.doseRate = GCR_DOSE_USV_H * (1 + 0.04 * Math.sin(simTime * 0.37) + 0.02 * Math.sin(simTime * 2.3));
    env.doseUSv += env.doseRate * dt / 3600;
  }

  // ── regolith dust: ballistic arcs (no air drag, 1/6 g) ─────────────────────
  var DUST_MAX = 1500;
  var dustPos = new Float32Array(DUST_MAX * 3), dustVel = new Float32Array(DUST_MAX * 3);
  var dustFloor = new Float32Array(DUST_MAX), dustAlive = new Uint8Array(DUST_MAX);
  for (var di = 0; di < DUST_MAX; di++) dustPos[di * 3 + 1] = -1e4;
  var dustGeo = new THREE.BufferGeometry();
  dustGeo.setAttribute("position", new THREE.BufferAttribute(dustPos, 3));
  var dustPts = new THREE.Points(dustGeo, new THREE.PointsMaterial({
    color: 0xb9b4a9, size: 0.035, sizeAttenuation: true, transparent: true, opacity: 0.85, depthWrite: false
  }));
  dustPts.frustumCulled = false;
  scene.add(dustPts);
  var dustNext = 0, dustAccum = 0, dustRand = makeRand(5150);
  var tmpV = new THREE.Vector3();

  function updateDust(dt) {
    var spd = Math.abs(groundSpeed);
    if (spd > 0.03) dustAccum += (spd * 220 + Math.abs(rv.turn) * 40) * dt;
    var fx = Math.sin(rv.yaw), fz = Math.cos(rv.yaw), dir = rv.speed >= 0 ? 1 : -1;
    while (dustAccum >= 1) {
      dustAccum -= 1;
      var i = dustNext; dustNext = (dustNext + 1) % DUST_MAX;
      var side = dustRand() < 0.5 ? 0.43 : -0.43;
      var wz = WHEEL_Z[(dustRand() * 3) | 0];
      rv.group.localToWorld(tmpV.set(side + (dustRand() - 0.5) * 0.12, 0.03, wz - dir * WHEEL_R * 0.6));
      dustPos[i * 3] = tmpV.x; dustPos[i * 3 + 1] = tmpV.y; dustPos[i * 3 + 2] = tmpV.z;
      var back = -(0.2 + dustRand() * 0.6) * Math.min(1, spd * 2) * dir;
      var up = (0.25 + dustRand() * 0.75) * Math.min(1, 0.3 + spd * 1.6);
      var lat = (dustRand() - 0.5) * 0.3;
      dustVel[i * 3] = fx * back + fz * lat;
      dustVel[i * 3 + 1] = up;
      dustVel[i * 3 + 2] = fz * back - fx * lat;
      dustFloor[i] = tmpV.y - 0.04;
      dustAlive[i] = 1;
    }
    for (var k = 0; k < DUST_MAX; k++) {
      if (!dustAlive[k]) continue;
      dustVel[k * 3 + 1] -= G_MOON * dt;            // only gravity: no drag in vacuum
      dustPos[k * 3] += dustVel[k * 3] * dt;
      dustPos[k * 3 + 1] += dustVel[k * 3 + 1] * dt;
      dustPos[k * 3 + 2] += dustVel[k * 3 + 2] * dt;
      if (dustPos[k * 3 + 1] < dustFloor[k]) { dustAlive[k] = 0; dustPos[k * 3 + 1] = -1e4; }
    }
    dustGeo.attributes.position.needsUpdate = true;
  }

  // ── wheel tracks pressed into the regolith ─────────────────────────────────
  var TRACK_MAX = 5000;
  var trackTex = canvasTexture(32, 64, function (g, w, h) {
    g.clearRect(0, 0, w, h);
    g.fillStyle = "rgba(0,0,0,0.35)"; g.fillRect(3, 0, w - 6, h);       // compacted rut
    g.fillStyle = "rgba(0,0,0,0.55)";
    for (var y = 2; y < h; y += 13) g.fillRect(3, y, w - 6, 5);           // grouser prints
  });
  var trackGeo = new THREE.PlaneGeometry(0.15, 0.24);
  trackGeo.rotateX(-Math.PI / 2);
  var tracks = new THREE.InstancedMesh(trackGeo, new THREE.MeshBasicMaterial({
    map: trackTex, transparent: true, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2
  }), TRACK_MAX);
  tracks.count = 0;
  tracks.frustumCulled = false;
  scene.add(tracks);
  var trackNext = 0, trackTotal = 0, trackDist = 0;
  var trackDummy = new THREE.Object3D(), qYaw = new THREE.Quaternion(), nrm = new THREE.Vector3();

  function clearTracks() { trackNext = 0; trackTotal = 0; trackDist = 0; tracks.count = 0; }

  function updateTracks(dt) {
    trackDist += Math.abs(groundSpeed) * dt;
    if (trackDist < 0.22) return;
    trackDist = 0;
    qYaw.setFromAxisAngle(UP, rv.yaw);
    [0.43, -0.43].forEach(function (side) {
      rv.group.localToWorld(tmpV.set(side, 0, 0));
      var x = tmpV.x, z = tmpV.z;
      nrm.set(surfaceAt(x - 0.15, z) - surfaceAt(x + 0.15, z), 0.3,
              surfaceAt(x, z - 0.15) - surfaceAt(x, z + 0.15)).normalize();
      trackDummy.position.set(x, surfaceAt(x, z) + 0.012, z);
      trackDummy.quaternion.setFromUnitVectors(UP, nrm).multiply(qYaw);
      trackDummy.updateMatrix();
      tracks.setMatrixAt(trackNext, trackDummy.matrix);
      trackNext = (trackNext + 1) % TRACK_MAX;
      trackTotal++;
    });
    tracks.count = Math.min(trackTotal, TRACK_MAX);
    tracks.instanceMatrix.needsUpdate = true;
  }

  // ── environment controls ───────────────────────────────────────────────────
  var elHourSlider = document.getElementById("env-hour");
  var elTimelapse = document.getElementById("btn-timelapse");
  function setLunarHour(h) {
    env.lunarHour = ((h % 24) + 24) % 24;
    if (elHourSlider) elHourSlider.value = env.lunarHour.toFixed(2);
  }
  if (elHourSlider) elHourSlider.addEventListener("input", function () { setLunarHour(parseFloat(elHourSlider.value)); });
  if (elTimelapse) elTimelapse.addEventListener("click", function () {
    env.timelapse = !env.timelapse;
    elTimelapse.classList.toggle("active", env.timelapse);
    elTimelapse.textContent = env.timelapse ? "⏸ Pause time-lapse" : "▶ Time-lapse lunar day";
  });
  document.querySelectorAll(".env-preset").forEach(function (b) {
    b.addEventListener("click", function () { setLunarHour(parseFloat(b.dataset.hour)); });
  });
  var elOverlay = document.getElementById("btn-overlay");
  if (elOverlay) elOverlay.addEventListener("click", function () {
    slopeOverlay = !slopeOverlay;
    elOverlay.textContent = "Slope overlay: " + (slopeOverlay ? "ON" : "OFF");
    rebuildTerrainMesh();
  });

  function setText(id, text, cls) {
    var el = document.getElementById(id);
    if (!el) return;
    el.textContent = text;
    if (cls !== undefined) el.className = cls;
  }

  function lunarPhaseName(h) {
    if (h < 5.5 || h >= 18.5) return "Lunar night";
    if (h < 6.5) return "Sunrise";
    if (h < 10) return "Morning";
    if (h < 14) return "Noon";
    if (h < 17.5) return "Afternoon";
    return "Sunset";
  }

  function fmtTemp(tC) { return (tC >= 0 ? "+" : "−") + Math.abs(tC).toFixed(0) + " °C"; }

  function updateEnvHUD() {
    var h = env.lunarHour, hh = Math.floor(h), mm = Math.floor((h - hh) * 60);
    setText("env-time", (hh < 10 ? "0" : "") + hh + ":" + (mm < 10 ? "0" : "") + mm);
    setText("env-phase", lunarPhaseName(h));
    // Earth days elapsed since local sunrise
    var since = ((h - 6) % 24 + 24) % 24 / 24 * LUNAR_DAY_EARTH_DAYS;
    setText("env-earthdays", "Earth day " + since.toFixed(1) + " of " + LUNAR_DAY_EARTH_DAYS + " since sunrise");
    setText("env-sun", env.sunElevDeg.toFixed(1) + "°", "mono-val sensor-val" + (env.sunElevDeg < 0 ? " env-dark" : ""));
    setText("env-light", env.sunUp <= 0 ? "Night" : env.sunlit ? "☀ Sunlit" : "◐ In shadow",
            "mono-val sensor-val " + (env.sunlit && env.sunUp > 0 ? "env-sunlit" : "env-dark"));
    var tC = env.tempK - 273.15;
    setText("env-temp", fmtTemp(tC), "mono-val sensor-val " + (tC > 60 ? "env-hot" : tC < -100 ? "env-cold" : ""));
    setText("stat-temp", fmtTemp(tC));
    setText("env-solar", env.solarW.toFixed(1) + " W");
    setText("env-heater", env.heaterW > 0.05 ? env.heaterW.toFixed(1) + " W" : "off");
    setText("env-rad", env.doseRate.toFixed(0) + " µSv/h");
    setText("env-dose", env.doseUSv.toFixed(2) + " µSv");
    setText("env-lights", rovLight.intensity > 0 ? "ON" : "off");
  }

  // ── HUD update ─────────────────────────────────────────────────────────────
  var simTime = 0;
  var sensorUpdateCounter = 0;
  function updateHUD(dt) {
    simTime += dt;

    // update sensors every frame
    sensors.encoders.update(dt);
    sensors.energy.update(dt);
    sensors.imu.update();

    // LiDAR + viz at reduced rate (every 5 frames) for performance
    sensorUpdateCounter++;
    if (sensorUpdateCounter % 5 === 0) {
      sensors.lidar.update();
      updateLidarViz();
    }

    var spd = Math.abs(groundSpeed);
    var deg = (((rv.yaw * 57.2958) % 360) + 360) % 360;
    var gdx = GOAL_X - rv.wx, gdz = GOAL_Z - rv.wz;
    var gdist = Math.sqrt(gdx * gdx + gdz * gdz);

    // telemetry — driven by real sensor data
    if (elSpeed) elSpeed.textContent = spd.toFixed(2) + " m/s";
    if (elHead) elHead.textContent = deg.toFixed(0) + "°";
    if (elPos) elPos.textContent = rv.wx.toFixed(1) + ", " + rv.wz.toFixed(1);
    if (elGoal) elGoal.textContent = gdist.toFixed(1) + " m";

    // battery from energy model (not fake timer)
    if (elBattery) {
      elBattery.textContent = sensors.energy.soc.toFixed(0) + "%";
      elBattery.className = sensors.energy.soc > 50 ? "success" : sensors.energy.soc > 20 ? "accent" : "danger-text";
    }
    if (elGrav) elGrav.textContent = G_MOON.toFixed(2) + " m/s² (0.165 g)";
    updateEnvHUD();
    setText("sens-slip", (slip * 100).toFixed(0) + " %", "mono-val sensor-val" + (slip > 0.25 ? " danger-text" : ""));
    var odx = sensors.encoders.odomX - rv.wx, odz = sensors.encoders.odomZ - rv.wz;
    setText("sens-odom-err", Math.sqrt(odx * odx + odz * odz).toFixed(2) + " m");
    setText("sens-solar", "+" + sensors.energy.solarW.toFixed(1) + " W");
    setText("sens-net", (sensors.energy.netW >= 0 ? "+" : "") + sensors.energy.netW.toFixed(1) + " W",
            "mono-val sensor-val" + (sensors.energy.netW >= 0 ? " env-sunlit" : ""));

    // sensor data panel
    var elLidarNear = document.getElementById("sens-lidar-near");
    var elLidarAngle = document.getElementById("sens-lidar-angle");
    var elLidarPts = document.getElementById("sens-lidar-pts");
    var elImuSlope = document.getElementById("sens-imu-slope");
    var elImuAx = document.getElementById("sens-imu-ax");
    var elImuAz = document.getElementById("sens-imu-az");
    var elOdomX = document.getElementById("sens-odom-x");
    var elOdomZ = document.getElementById("sens-odom-z");
    var elOdomDist = document.getElementById("sens-odom-dist");
    var elOdomTheta = document.getElementById("sens-odom-theta");
    var elEnPower = document.getElementById("sens-power");
    var elEnCurrent = document.getElementById("sens-current");
    var elEnConsumed = document.getElementById("sens-consumed");

    if (elLidarNear) elLidarNear.textContent = sensors.lidar.nearestRange.toFixed(2) + " m";
    if (elLidarAngle) elLidarAngle.textContent = sensors.lidar.nearestAngle.toFixed(0) + "°";
    if (elLidarPts) elLidarPts.textContent = sensors.lidar.pointCloud.length;
    if (elImuSlope) elImuSlope.textContent = sensors.imu.slopeDeg.toFixed(1) + "°";
    if (elImuAx) elImuAx.textContent = sensors.imu.ax.toFixed(3);
    if (elImuAz) elImuAz.textContent = sensors.imu.az.toFixed(3);
    if (elOdomX) elOdomX.textContent = sensors.encoders.odomX.toFixed(2);
    if (elOdomZ) elOdomZ.textContent = sensors.encoders.odomZ.toFixed(2);
    if (elOdomDist) elOdomDist.textContent = sensors.encoders.totalDistance.toFixed(1) + " m";
    if (elOdomTheta) elOdomTheta.textContent = (sensors.encoders.odomTheta * 57.2958).toFixed(1) + "°";
    if (elEnPower) elEnPower.textContent = sensors.energy.currentPower.toFixed(1) + " W";
    if (elEnCurrent) elEnCurrent.textContent = sensors.energy.currentA.toFixed(2) + " A";
    if (elEnConsumed) elEnConsumed.textContent = (sensors.energy.consumedJ / 1000).toFixed(2) + " kJ";

    // uphill / downhill readouts
    var elPitch = document.getElementById("sens-imu-pitch");
    var elClimb = document.getElementById("sens-odom-climb");
    var elDesc = document.getElementById("sens-odom-desc");
    var elGravP = document.getElementById("sens-grav-power");
    var pitch = sensors.imu.pitchDeg || 0;
    if (elPitch) {
      elPitch.textContent = (Math.abs(pitch) < 0.5 ? "level " : pitch > 0 ? "↑ " : "↓ ") + Math.abs(pitch).toFixed(1) + "°";
      elPitch.className = "mono-val sensor-val" + (Math.abs(pitch) > MAX_SLOPE_DEG * 0.8 ? " danger-text" : "");
    }
    if (elClimb) elClimb.textContent = sensors.encoders.climb.toFixed(2) + " m";
    if (elDesc) elDesc.textContent = sensors.encoders.descent.toFixed(2) + " m";
    if (elGravP) elGravP.textContent = (sensors.energy.gravityPower >= 0 ? "+" : "") + sensors.energy.gravityPower.toFixed(1) + " W";
    if (slopeBlocked && elStatus) {
      elStatus.textContent = "⛰ Too steep (> " + MAX_SLOPE_DEG + "°) — find another way around";
    }

    // goal reached
    if (gdist < 1.2 && elStatus) {
      elStatus.textContent = "🎯 Goal reached! Drive to explore further.";
    }

    // animate goal flag
    flag.rotation.z = Math.sin(simTime * 2) * 0.12;
    goalLight.intensity = 0.6 + Math.sin(simTime * 3) * 0.2;

    // update quantum AI bars with live sensor-driven values
    var qPath = document.getElementById("q-path");
    var qEnergy = document.getElementById("q-energy");
    var qHazard = document.getElementById("q-hazard");
    if (qPath) {
      var pv = 70 + Math.sin(simTime * 0.7) * 12;
      qPath.style.width = pv.toFixed(0) + "%";
      qPath.parentElement.nextElementSibling.textContent = pv.toFixed(0) + "%";
    }
    if (qEnergy) {
      var ev = Math.max(0, Math.min(100, sensors.energy.soc));
      qEnergy.style.width = ev.toFixed(0) + "%";
      qEnergy.parentElement.nextElementSibling.textContent = ev.toFixed(0) + "%";
    }
    if (qHazard) {
      // hazard detection confidence based on nearest LiDAR range
      var hv = sensors.lidar.nearestRange < 2.0
        ? Math.min(100, 95 + (2.0 - sensors.lidar.nearestRange) * 5)
        : Math.max(50, 85 - (sensors.lidar.nearestRange - 2) * 5);
      qHazard.style.width = hv.toFixed(0) + "%";
      qHazard.parentElement.nextElementSibling.textContent = hv.toFixed(0) + "%";
    }
  }

  // ── render loop ────────────────────────────────────────────────────────────
  var last = performance.now();
  function loop(now) {
    requestAnimationFrame(loop);
    var dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    step(dt);
  }

  function step(dt) {
    drive(dt);
    placeRover();
    updateCamera();
    skyGroup.position.copy(camera.position);   // sky is "at infinity"
    updateEnvironment(dt);
    updateDust(dt);
    updateTracks(dt);
    updateHUD(dt);
    renderer.render(scene, camera);
  }
  requestAnimationFrame(loop);
  // test / recording hook: advance the simulation n frames of dt seconds
  window.roverSim.stepFrames = function (n, dt) { for (var i = 0; i < n; i++) step(dt || 1 / 60); };

  // ── resize ─────────────────────────────────────────────────────────────────
  window.addEventListener("resize", function () {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
  });

  // init
  rv.wy = surfaceAt(rv.wx, rv.wz);
  sensors.encoders.prevH = rv.wy;
  if (elStatus) elStatus.textContent = "Click here, then press W to drive forward  ·  WASD / Arrow keys";
  document.body.setAttribute("tabindex", "0");
  document.body.focus();
})();
