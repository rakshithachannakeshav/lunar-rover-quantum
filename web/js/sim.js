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
  var seg = 320;
  var tGeo = new THREE.PlaneGeometry(200, 200, seg, seg);
  tGeo.rotateX(-Math.PI / 2);
  var tv = tGeo.attributes.position;
  tGeo.setAttribute("color", new THREE.Float32BufferAttribute(new Float32Array(tv.count * 3), 3));

  // lunar regolith color, tinted amber on steep ground and red where impassable
  var REGOLITH = new THREE.Color(0x9a927a);
  var STEEP = new THREE.Color(0xc9893f);
  var IMPASSABLE = new THREE.Color(0xa8443a);
  var tmpColor = new THREE.Color();

  function rebuildTerrainMesh() {
    var col = tGeo.attributes.color;
    for (var i = 0; i < tv.count; i++) {
      var x = tv.getX(i), z = tv.getZ(i);
      tv.setY(i, heightAt(x, z));
      var g = slopeGradeAt(x, z);
      if (g > MAX_GRADE) tmpColor.copy(IMPASSABLE);
      else tmpColor.copy(REGOLITH).lerp(STEEP, Math.min(1, g / MAX_GRADE) * 0.75);
      col.setXYZ(i, tmpColor.r, tmpColor.g, tmpColor.b);
    }
    tv.needsUpdate = true;
    col.needsUpdate = true;
    tGeo.computeVertexNormals();
  }
  rebuildTerrainMesh();

  var tMat = new THREE.MeshStandardMaterial({
    color: 0xffffff, vertexColors: true, roughness: 0.95, metalness: 0.02
  });
  var terrain = new THREE.Mesh(tGeo, tMat);
  terrain.receiveShadow = true;
  scene.add(terrain);

  // ── rocks (with collision) ─────────────────────────────────────────────────
  var rockMat = new THREE.MeshStandardMaterial({ color: 0x5a5248, roughness: 0.9, metalness: 0.05 });
  // seeded PRNG (mulberry32) so the rock field — and every benchmark — is identical on each load
  var rockSeed = 20240607;
  function rockRand() {
    rockSeed = (rockSeed + 0x6D2B79F5) | 0;
    var t = Math.imul(rockSeed ^ (rockSeed >>> 15), 1 | rockSeed);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  function addRock(x, z, rx, ry, rz) {
    var baseRadius = 0.4 + rockRand() * 0.5;
    var g = new THREE.DodecahedronGeometry(baseRadius, 0);
    var m = new THREE.Mesh(g, rockMat);
    m.scale.set(rx, ry, rz);
    var y = heightAt(x, z);
    m.position.set(x, y + ry * 0.3, z);
    m.rotation.set(rockRand(), rockRand() * 6, rockRand());
    m.castShadow = true;
    scene.add(m);

    // register obstacle for collision detection
    // collision radius = average XZ scale * base geometry radius
    var collisionRadius = baseRadius * Math.max(rx, rz) * 0.85;
    obstacles.push({ x: x, z: z, radius: collisionRadius, mesh: m, lift: ry * 0.3, ring: null });
  }
  var rockDefs = [
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
  rockDefs.forEach(function (r) { addRock(r[0], r[1], r[2], r[3], r[4]); });

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
  var GRID_RES = 0.5;       // meters per cell
  var GRID_MIN = -30;
  var GRID_MAX = 30;

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

  // ── QAOA quantum-inspired path optimisation ─────────────────────────────────
  // Mirrors quantum_optimizer.py: build a quantum corridor from the A* backbone,
  // generate segment alternatives, construct a QUBO, simulate QAOA probability
  // distribution, and decode the lowest-energy feasible path.
  var qaoaState = {
    running: false,
    progress: 0,        // 0-100
    phase: "",           // status text
    qubits: 0,
    shots: 0,
    reps: 0,
    expectation: 0,
    validSamples: 0,
    corridorEdges: 0
  };

  // Generate an alternative A* path with perturbed costs (simulates corridor)
  function astarPathPerturbed(startX, startZ, goalX, goalZ, seed) {
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
                g.gx += ddx; g.gz += ddz; found = true;
              }
            }
          }
        }
      }
    }

    var dirs = [[1,0],[0,1],[-1,0],[0,-1],[1,1],[1,-1],[-1,1],[-1,-1]];
    var open = {}, closed = {}, heap = new MinHeap();
    var startKey = gridKey(s.gx, s.gz);
    var goalKey = gridKey(g.gx, g.gz);

    function heuristic(ax, az) {
      var dx = ax - g.gx, dz = az - g.gz;
      return Math.sqrt(dx * dx + dz * dz) * GRID_RES * MIN_GRADE_FACTOR;
    }
    // simple seeded PRNG for cost perturbation
    function noise(x, z) {
      var v = Math.sin(x * 127.1 + z * 311.7 + seed * 43758.5453) * 43758.5453;
      return (v - Math.floor(v)) * 0.3;
    }

    open[startKey] = { gx: s.gx, gz: s.gz, f: heuristic(s.gx, s.gz), g: 0, parent: null };

    heap.push(startKey, open[startKey].f);
    var iterations = 0, maxIter = gridSize * gridSize;

    while (true) {
      var bestKey = popOpen(heap, open);
      if (bestKey === null || iterations++ > maxIter) return null;

      var current = open[bestKey];
      delete open[bestKey];
      closed[bestKey] = current;

      if (bestKey === goalKey) {
        var path = [];
        var node = current;
        while (node) { var w = gridToWorld(node.gx, node.gz); path.unshift([w.wx, w.wz]); node = node.parent; }
        path.expanded = iterations;
        return path;
      }

      for (var d = 0; d < dirs.length; d++) {
        var ngx = current.gx + dirs[d][0], ngz = current.gz + dirs[d][1];
        if (ngx < 0 || ngx >= gridSize || ngz < 0 || ngz >= gridSize) continue;
        var nk = gridKey(ngx, ngz);
        if (closed[nk] || isBlocked(ngx, ngz)) continue;

        var cw = gridToWorld(current.gx, current.gz);
        var nw = gridToWorld(ngx, ngz);
        var baseCost = edgeCost(cw.wx, cw.wz, nw.wx, nw.wz);
        if (baseCost === Infinity) continue;
        // add terrain-roughness perturbation (simulates alternative terrain scoring)
        var moveCost = baseCost * (1.0 + noise(ngx, ngz));
        var tentG = current.g + moveCost;
        if (open[nk] && open[nk].g <= tentG) continue;
        open[nk] = { gx: ngx, gz: ngz, f: tentG + heuristic(ngx, ngz), g: tentG, parent: current };
        heap.push(nk, open[nk].f);
      }
    }
  }

  // QAOA-style path selection: build corridor, evaluate segment alternatives,
  // simulate QUBO probability distribution, select lowest-energy feasible path
  function qaoaPath(startX, startZ, goalX, goalZ) {
    var t0 = performance.now();

    // Phase 1: Build A* backbone (like classical_planning.run_astar)
    qaoaState.phase = "Building A* backbone...";
    qaoaState.progress = 10;
    var backbone = astarPath(startX, startZ, goalX, goalZ);
    if (!backbone) return null;
    var expanded = backbone.expanded || 0;  // total search effort, all sub-searches

    // Phase 2: Build quantum corridor — generate alternative segments
    // (mirrors quantum_optimizer.build_quantum_corridor)
    qaoaState.phase = "Building quantum corridor...";
    qaoaState.progress = 25;

    var NUM_ANCHORS = Math.min(6, Math.max(3, Math.floor(backbone.length / 8)));
    var anchorIndices = [0];
    for (var a = 1; a < NUM_ANCHORS - 1; a++) {
      anchorIndices.push(Math.round(a * (backbone.length - 1) / (NUM_ANCHORS - 1)));
    }
    anchorIndices.push(backbone.length - 1);

    // For each segment between anchors, find alternative paths
    var segments = [];  // each: { baseline: [...], alternatives: [[...], ...] }
    for (var si = 0; si < anchorIndices.length - 1; si++) {
      var ai = anchorIndices[si], bi = anchorIndices[si + 1];
      var segStart = backbone[ai], segEnd = backbone[bi];
      var baselineSegment = backbone.slice(ai, bi + 1);

      var alts = [];
      // generate 2-3 alternatives with perturbed costs (like corridor segment candidates)
      for (var alt = 0; alt < 3; alt++) {
        var altPath = astarPathPerturbed(segStart[0], segStart[1], segEnd[0], segEnd[1], alt * 17 + si * 7 + 42);
        if (altPath) expanded += altPath.expanded || 0;
        if (altPath && altPath.length > 1) {
          alts.push(altPath);
        }
      }
      segments.push({ baseline: baselineSegment, alternatives: alts });
    }

    var totalEdges = segments.length + segments.reduce(function(s, seg) { return s + seg.alternatives.length; }, 0);
    qaoaState.corridorEdges = totalEdges;

    // Phase 3: Build QUBO — edge-selection binary variables
    // (mirrors quantum_optimizer.build_qubo)
    qaoaState.phase = "Building QUBO...";
    qaoaState.progress = 40;

    // Each segment has 1 baseline + N alternative edges = binary variables
    var numQubits = 0;
    var edgeList = [];  // { segIdx, edgeIdx, path, energy }
    for (var si = 0; si < segments.length; si++) {
      var seg = segments[si];
      var baseE = computePathEnergy(seg.baseline);
      edgeList.push({ segIdx: si, edgeIdx: 0, path: seg.baseline, energy: baseE.energy });
      numQubits++;
      for (var ai = 0; ai < seg.alternatives.length; ai++) {
        var altE = computePathEnergy(seg.alternatives[ai]);
        edgeList.push({ segIdx: si, edgeIdx: ai + 1, path: seg.alternatives[ai], energy: altE.energy });
        numQubits++;
      }
    }

    // add activation qubits for internal anchors (like quantum_optimizer)
    var activationQubits = Math.max(0, NUM_ANCHORS - 2);
    numQubits += activationQubits;

    qaoaState.qubits = numQubits;
    qaoaState.reps = Math.min(3, Math.max(1, Math.ceil(numQubits / 6)));

    // Phase 4: Simulate QAOA — evaluate all feasible bitstrings
    // (mirrors optimize_qaoa + sampling)
    qaoaState.phase = "Running QAOA simulation (p=" + qaoaState.reps + ")...";
    qaoaState.progress = 55;
    qaoaState.shots = 2048;

    // Enumerate feasible combinations: pick exactly one edge per segment
    var feasiblePaths = [];
    var edgesBySegment = [];
    for (var si = 0; si < segments.length; si++) {
      var segEdges = edgeList.filter(function(e) { return e.segIdx === si; });
      edgesBySegment.push(segEdges);
    }

    // Enumerate all combinations (product of segment choices)
    function enumerate(segIdx, currentPath, currentEnergy) {
      if (segIdx >= edgesBySegment.length) {
        feasiblePaths.push({ path: currentPath.slice(), energy: currentEnergy });
        return;
      }
      var choices = edgesBySegment[segIdx];
      for (var c = 0; c < choices.length; c++) {
        var segPath = choices[c].path;
        // stitch: drop first point of segment (it's the last of previous)
        var appended = segIdx === 0 ? segPath : segPath.slice(1);
        enumerate(segIdx + 1, currentPath.concat(appended), currentEnergy + choices[c].energy);
      }
    }
    enumerate(0, [], 0);

    if (feasiblePaths.length === 0) return null;  // no classical fallback

    // Phase 5: QAOA probability model — simulate quantum interference
    // Each feasible path gets a probability proportional to QAOA amplitudes
    // |ψ⟩ = Σ α_x |x⟩  where amplitudes are biased toward low-energy states
    qaoaState.phase = "Decoding measurement outcomes...";
    qaoaState.progress = 75;

    // Compute QAOA-like probabilities using Boltzmann-style distribution
    // This mirrors how trained QAOA concentrates probability on low-cost states
    var energies = feasiblePaths.map(function(fp) { return fp.energy; });
    var minE = Math.min.apply(null, energies);
    var maxE = Math.max.apply(null, energies);
    var range = Math.max(maxE - minE, 0.01);

    // Temperature parameter — lower = more "quantum-optimised" concentration
    // p layers of QAOA ≈ more concentration on optimal
    var temperature = range / (2.5 * qaoaState.reps);

    var boltzmannWeights = energies.map(function(e) {
      return Math.exp(-(e - minE) / Math.max(temperature, 0.001));
    });
    var totalWeight = boltzmannWeights.reduce(function(a, b) { return a + b; }, 0);
    var probs = boltzmannWeights.map(function(w) { return w / totalWeight; });

    // Simulate shots — sample from probability distribution
    var shotCounts = new Array(feasiblePaths.length).fill(0);
    // Deterministic sampling based on probabilities
    var cumProbs = [];
    var cumSum = 0;
    for (var i = 0; i < probs.length; i++) {
      cumSum += probs[i];
      cumProbs.push(cumSum);
    }

    // Use a seeded LCG for reproducible "measurement" outcomes
    var rngState = 12345;
    function nextRand() {
      rngState = (rngState * 1103515245 + 12345) & 0x7fffffff;
      return rngState / 0x7fffffff;
    }

    var validSamples = 0;
    for (var shot = 0; shot < qaoaState.shots; shot++) {
      var r = nextRand();
      for (var j = 0; j < cumProbs.length; j++) {
        if (r <= cumProbs[j]) {
          shotCounts[j]++;
          validSamples++;
          break;
        }
      }
    }
    qaoaState.validSamples = validSamples;

    // Decode the MOST-SAMPLED feasible bitstring, as a real QAOA run does.
    // (Picking the lowest-energy candidate instead would be classical post-selection.)
    // Shallow circuits don't always concentrate on the optimum, so this can miss it.
    var bestIdx = 0;
    for (var i = 1; i < feasiblePaths.length; i++) {
      if (shotCounts[i] > shotCounts[bestIdx] ||
          (shotCounts[i] === shotCounts[bestIdx] && feasiblePaths[i].energy < feasiblePaths[bestIdx].energy)) {
        bestIdx = i;
      }
    }

    // Compute the QAOA expectation value ⟨C⟩ = Σ p_x · C(x)
    var expectation = 0;
    for (var i = 0; i < probs.length; i++) {
      expectation += probs[i] * feasiblePaths[i].energy;
    }
    qaoaState.expectation = expectation;

    qaoaState.phase = "Path decoded";
    qaoaState.progress = 100;
    qaoaState.running = false;

    var chosen = feasiblePaths[bestIdx].path;
    chosen.expanded = expanded;
    return chosen;
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
      var py = heightAt(px, pz) + 0.15;
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
      var py = heightAt(path[i][0], path[i][1]) + 0.25;
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
  var gly = heightAt(GOAL_X, GOAL_Z);
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
    ring.position.set(o.x, heightAt(o.x, o.z) + 0.05, o.z);
    scene.add(ring);
    o.ring = ring;
  });

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
      if (elQVs) elQVs.textContent = qr.validSamples + " / " + qr.shots;
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
      var oy = heightAt(o.x, o.z);
      o.mesh.position.y = oy + o.lift;
      if (o.ring) o.ring.position.y = oy + 0.05;
    });
    var gy = heightAt(GOAL_X, GOAL_Z);
    goalGroup.position.y = gy;
    goalLight.position.y = gy + 1;

    // every terrain run starts from spawn with fresh odometry/energy counters
    rv.wx = START_X; rv.wz = START_Z; rv.yaw = 0; rv.speed = 0; rv.turn = 0;
    rv.wy = heightAt(rv.wx, rv.wz);
    var enc = sensors.encoders;
    enc.odomX = enc.prevX = START_X; enc.odomZ = enc.prevZ = START_Z; enc.odomTheta = 0;
    enc.totalDistance = 0; enc.climb = 0; enc.descent = 0;
    enc.prevH = rv.wy;
    sensors.energy.consumedJ = 0;
    patrolIdx = 0;

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
    rover: function () { return [rv.wx, rv.wz]; }
  };

  document.querySelectorAll(".cam-btn[data-cam]").forEach(function (btn) {
    btn.addEventListener("click", function () {
      document.querySelectorAll(".cam-btn[data-cam]").forEach(function (b) { b.classList.remove("active"); });
      btn.classList.add("active");
      camMode = btn.dataset.cam;
    });
  });

  // stop HUD scroll from zooming the 3D camera
  document.querySelectorAll(".hud").forEach(function (hud) {
    hud.addEventListener("wheel", function (e) { e.stopPropagation(); });
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
  // grade (rise/run) along the rover's heading, sampled ±0.3 m around it
  function headingGrade() {
    var sx = Math.sin(rv.yaw) * 0.3, sz = Math.cos(rv.yaw) * 0.3;
    return (heightAt(rv.wx + sx, rv.wz + sz) - heightAt(rv.wx - sx, rv.wz - sz)) / 0.6;
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
        rv.yaw += rv.turn * dt;
        rv.wx += Math.sin(rv.yaw) * rv.speed * dt;
        rv.wz += Math.cos(rv.yaw) * rv.speed * dt;
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

        rv.yaw += rv.turn * dt;
        rv.wx += Math.sin(rv.yaw) * rv.speed * dt;
        rv.wz += Math.cos(rv.yaw) * rv.speed * dt;
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

      rv.yaw += rv.turn * dt;
      rv.wx += Math.sin(rv.yaw) * rv.speed * dt;
      rv.wz += Math.cos(rv.yaw) * rv.speed * dt;
    } else {
      var fwd = (keys.w ? 1 : 0) - (keys.s ? 1 : 0);
      var trn = (keys.a ? 1 : 0) - (keys.d ? 1 : 0);
      if (keys[" "]) { fwd = 0; trn = 0; }
      var tspd = fwd > 0 ? capFwd : fwd < 0 ? -capRev : 0;
      var ttrn = trn * 1.6;
      rv.speed += (tspd - rv.speed) * Math.min(1, dt * 7);
      rv.turn += (ttrn - rv.turn) * Math.min(1, dt * 8);

      rv.yaw += rv.turn * dt;
      rv.wx += Math.sin(rv.yaw) * rv.speed * dt;
      rv.wz += Math.cos(rv.yaw) * rv.speed * dt;
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
      var moveGrade = (heightAt(oldX + ux * probe, oldZ + uz * probe) - heightAt(oldX, oldZ)) / probe;
      // 10% headroom over the planning limit so planned paths never stall between grid points
      if (Math.abs(moveGrade) > MAX_GRADE * 1.1) {
        rv.wx = oldX;
        rv.wz = oldZ;
        rv.speed *= 0.3;
        slopeBlocked = true;
      }
    }

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
        var dx = heightAt(rv.wx + 0.3, rv.wz) - heightAt(rv.wx - 0.3, rv.wz);
        var dz = heightAt(rv.wx, rv.wz + 0.3) - heightAt(rv.wx, rv.wz - 0.3);
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
      odomX: -3, odomZ: 3,    // dead-reckoned position
      odomTheta: 0,            // dead-reckoned heading
      linearVel: 0,            // estimated linear velocity (m/s)
      angularVel: 0,           // estimated angular velocity (rad/s)
      totalDistance: 0,         // cumulative odometry distance (m)
      climb: 0, descent: 0,     // cumulative elevation gained / lost (m)
      prevX: -3, prevZ: 3, prevH: 0,

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
        var h = heightAt(rv.wx, rv.wz);
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

      update: function (dt) {
        // P_grav = m·g·v·sinθ along the heading (v is signed, so reversing flips it)
        var pg = this.mass * 1.62 * rv.speed * Math.sin(Math.atan(sensors.imu.grade || 0));
        this.gravityPower = pg < 0 ? pg * this.regenEff : pg;
        this.currentPower = Math.max(this.pIdle * 0.5, this.pIdle +
          this.kLinear * Math.abs(rv.speed) +
          this.kAngular * Math.abs(rv.turn) +
          this.gravityPower);
        this.consumedJ += this.currentPower * dt;
        this.currentA = this.currentPower / this.voltage;
        var capacityJ = this.capacityWh * 3600;
        this.soc = Math.max(0, (1 - this.consumedJ / capacityJ) * 100);
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

    var spd = Math.abs(rv.speed);
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
    if (elGrav) elGrav.textContent = "1.62 m/s²";
    if (elTemp) elTemp.textContent = "−153 °C";

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
  sensors.encoders.prevH = rv.wy;
  if (elStatus) elStatus.textContent = "Click here, then press W to drive forward  ·  WASD / Arrow keys";
  document.body.setAttribute("tabindex", "0");
  document.body.focus();
})();
