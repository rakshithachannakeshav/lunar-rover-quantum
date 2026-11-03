/*
 * qaoa.js - real QAOA path planner for the web viewer (no A*, no Dijkstra).
 *
 * Mirrors src/planning_pkg/planning_pkg/quantum_optimizer.py:
 *   1. corridor: a rows x cols lattice of anchor cells laid out geometrically
 *      between start and goal; neighbouring anchors are joined by a straight
 *      walk (always step to the neighbour closest to the target; edge weights
 *      are ignored), and each walk becomes one macro-edge weighted by its energy;
 *   2. QUBO: one binary per macro-edge plus one activation binary per internal
 *      anchor; start/goal degree exactly 1, internal anchors degree 0 or 2;
 *   3. QAOA: exact statevector simulation of p layers (cost phase + X mixer),
 *      angles trained to minimise <C>, then `shots` measurements;
 *   4. answer: the valid path QAOA sampled MOST OFTEN (QAOA's own answer, not
 *      best-of-shots post-selection). If no shot is a valid path: null.
 *      There is no classical fallback.
 *
 * Usage: QaoaCore.plan({ start:[x,z], goal:[x,z], gridRes, gridMin, gridMax,
 *                        isBlocked(gx,gz), edgeCost(x1,z1,x2,z2), reps, shots, seed })
 */
(function (root) {
  "use strict";

  var DIRS = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]];

  function mulberry32(seed) {
    var a = seed >>> 0;
    return function () {
      a = (a + 0x6D2B79F5) >>> 0;
      var t = a;
      t = Math.imul(t ^ (t >>> 15), t | 1);
      t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  // Largest rows x cols lattice (rows odd, prefer >= 3) with at most maxEdges edges.
  function latticeShape(maxEdges) {
    var best = null;
    for (var rows = 1; rows <= maxEdges + 1; rows += 2) {
      for (var cols = 2; cols <= maxEdges + 1; cols++) {
        var edges = rows * (cols - 1) + cols * (rows - 1);
        if (edges > maxEdges) break;
        var cand = [rows >= 3 ? 1 : 0, edges, rows, cols];
        if (!best || cmp(cand, best) > 0) best = cand;
      }
    }
    if (!best) throw new Error("maxEdges must be at least 1");
    return { rows: best[2], cols: best[3] };
  }
  function cmp(a, b) {
    for (var i = 0; i < a.length; i++) if (a[i] !== b[i]) return a[i] - b[i];
    return 0;
  }

  function Corridor(o) {
    this.o = o;
    this.size = Math.round((o.gridMax - o.gridMin) / o.gridRes);
  }
  Corridor.prototype.world = function (c) {
    return [this.o.gridMin + c[0] * this.o.gridRes, this.o.gridMin + c[1] * this.o.gridRes];
  };
  Corridor.prototype.inside = function (gx, gz) {
    return gx >= 0 && gz >= 0 && gx < this.size && gz < this.size;
  };
  // nearest free cell to a world point (pure geometry)
  Corridor.prototype.snap = function (x, z) {
    var gx = Math.round((x - this.o.gridMin) / this.o.gridRes);
    var gz = Math.round((z - this.o.gridMin) / this.o.gridRes);
    for (var r = 0; r <= 8; r++) {
      for (var dx = -r; dx <= r; dx++) {
        for (var dz = -r; dz <= r; dz++) {
          if (Math.max(Math.abs(dx), Math.abs(dz)) !== r) continue;
          var cx = gx + dx, cz = gz + dz;
          if (this.inside(cx, cz) && !this.o.isBlocked(cx, cz)) return [cx, cz];
        }
      }
    }
    return null;
  };
  // straight walk a -> b: always step to the free neighbour geometrically closest to b
  Corridor.prototype.walk = function (a, b) {
    var cells = [a], cur = a, energy = 0, self = this;
    function dist(c) { return Math.hypot(c[0] - b[0], c[1] - b[1]); }
    for (var guard = 0; guard < 4 * this.size; guard++) {
      if (cur[0] === b[0] && cur[1] === b[1]) return { cells: cells, energy: energy };
      var best = null, bestD = Infinity, bestCost = 0;
      for (var d = 0; d < DIRS.length; d++) {
        var n = [cur[0] + DIRS[d][0], cur[1] + DIRS[d][1]];
        if (!self.inside(n[0], n[1]) || self.o.isBlocked(n[0], n[1])) continue;
        var w0 = self.world(cur), w1 = self.world(n);
        var c = self.o.edgeCost(w0[0], w0[1], w1[0], w1[1]);
        if (!isFinite(c)) continue;
        var dn = dist(n);
        if (dn < bestD) { bestD = dn; best = n; bestCost = c; }
      }
      if (!best || bestD >= dist(cur)) return null;   // blocked: no step gets closer
      cells.push(best); energy += bestCost; cur = best;
    }
    return null;
  };

  function buildCorridor(o, maxEdges, widthRatio) {
    var cor = new Corridor(o);
    var s = cor.snap(o.start[0], o.start[1]);
    var g = cor.snap(o.goal[0], o.goal[1]);
    if (!s || !g) return { error: "start or goal is inside an obstacle" };
    if (s[0] === g[0] && s[1] === g[1]) return { error: "start and goal are the same cell" };
    var sw = cor.world(s), gw = cor.world(g);
    var ax = gw[0] - sw[0], az = gw[1] - sw[1], len = Math.hypot(ax, az);
    var px = -az / len, pz = ax / len;
    var shape = latticeShape(maxEdges), rows = shape.rows, cols = shape.cols;
    var mid = Math.floor(rows / 2), half = widthRatio * len;

    var anchors = [], grid = {}, used = {};
    function key(c) { return c[0] + "," + c[1]; }
    function add(i, j, c) { grid[i + ":" + j] = anchors.length; anchors.push(c); used[key(c)] = true; }
    add(mid, 0, s);
    add(mid, cols - 1, g);
    for (var i = 0; i < rows; i++) {
      for (var j = 0; j < cols; j++) {
        if (i === mid && (j === 0 || j === cols - 1)) continue;
        var off = mid ? (i - mid) / mid * half : 0;
        var t = j / (cols - 1);
        var c = cor.snap(sw[0] + ax * t + px * off, sw[1] + az * t + pz * off);
        if (!c || used[key(c)]) continue;
        add(i, j, c);
      }
    }
    var edges = [], walkCells = 0;
    for (i = 0; i < rows; i++) {
      for (j = 0; j < cols; j++) {
        var a = grid[i + ":" + j];
        if (a === undefined) continue;
        [[i + 1, j], [i, j + 1]].forEach(function (nb) {
          var b = grid[nb[0] + ":" + nb[1]];
          if (b === undefined) return;
          var w = cor.walk(anchors[a], anchors[b]);
          if (!w) return;
          walkCells += w.cells.length;
          for (var k = 1; k < w.cells.length - 1; k++) if (used[key(w.cells[k])]) return; // cuts another anchor
          edges.push({ u: a, v: b, w: w.energy, cells: w.cells });
        });
      }
    }
    // connectivity start(0) -> goal(1) by BFS over macro-edges (no cost-based search)
    var seen = { 0: true }, queue = [0];
    while (queue.length) {
      var x = queue.shift();
      edges.forEach(function (e) {
        var y = e.u === x ? e.v : (e.v === x ? e.u : -1);
        if (y >= 0 && !seen[y]) { seen[y] = true; queue.push(y); }
      });
    }
    if (!seen[1]) return { error: "obstacles block the quantum corridor between start and goal" };
    return { cor: cor, anchors: anchors, edges: edges, walkCells: walkCells };
  }

  // Edge-selection QUBO, as quantum_optimizer.build_qubo
  function buildQubo(corr) {
    var E = corr.edges.length, incident = {};
    corr.edges.forEach(function (e, idx) {
      (incident[e.u] = incident[e.u] || []).push(idx);
      (incident[e.v] = incident[e.v] || []).push(idx);
    });
    var sumW = corr.edges.reduce(function (s, e) { return s + e.w; }, 0);
    var P = Math.max(1, 2 * sumW);
    var lin = {}, quad = [], constant = 0, nVar = E;
    function L(i, v) { lin[i] = (lin[i] || 0) + v; }
    function Q(i, j, v) { quad.push([i, j, v]); }
    corr.edges.forEach(function (e, idx) { L(idx, e.w); });
    [0, 1].forEach(function (endpoint) {          // P * (deg - 1)^2
      var vs = incident[endpoint] || [];
      constant += P;
      vs.forEach(function (i) { L(i, -P); });
      for (var a = 0; a < vs.length; a++) for (var b = a + 1; b < vs.length; b++) Q(vs[a], vs[b], 2 * P);
    });
    for (var node = 2; node < corr.anchors.length; node++) {   // P * (deg - 2y)^2
      var vs = incident[node];
      if (!vs || !vs.length) continue;
      var y = nVar++;
      vs.forEach(function (i) { L(i, P); Q(i, y, -4 * P); });
      for (var a = 0; a < vs.length; a++) for (var b = a + 1; b < vs.length; b++) Q(vs[a], vs[b], 2 * P);
      L(y, 4 * P);
    }
    return { nVar: nVar, nEdges: E, lin: lin, quad: quad, constant: constant, penalty: P };
  }

  // QUBO value of every basis state (bit i of the index = variable i)
  function diagonal(qubo) {
    var N = 1 << qubo.nVar, C = new Float64Array(N).fill(qubo.constant);
    Object.keys(qubo.lin).forEach(function (k) {
      var i = +k, v = qubo.lin[k], m = 1 << i;
      for (var s = 0; s < N; s++) if (s & m) C[s] += v;
    });
    qubo.quad.forEach(function (t) {
      var m = (1 << t[0]) | (1 << t[1]), v = t[2];
      for (var s = 0; s < N; s++) if ((s & m) === m) C[s] += v;
    });
    return C;
  }

  function Simulator(Cn, n) {
    this.C = Cn; this.n = n; this.N = 1 << n;
    this.re = new Float64Array(this.N); this.im = new Float64Array(this.N);
    this.evals = 0;
  }
  Simulator.prototype.run = function (params) {
    var p = params.length / 2, N = this.N, re = this.re, im = this.im, C = this.C;
    var a0 = 1 / Math.sqrt(N);
    re.fill(a0); im.fill(0);
    for (var l = 0; l < p; l++) {
      var g = params[l];
      for (var s = 0; s < N; s++) {               // exp(-i g C)
        var ang = -g * C[s], c = Math.cos(ang), sn = Math.sin(ang);
        var r = re[s], i = im[s];
        re[s] = r * c - i * sn; im[s] = r * sn + i * c;
      }
      var cb = Math.cos(params[p + l]), sb = Math.sin(params[p + l]);
      for (var k = 0; k < this.n; k++) {           // exp(-i b X) on qubit k
        var st = 1 << k;
        for (var base = 0; base < N; base += st << 1) {
          for (var a = base, end = base + st; a < end; a++) {
            var b = a + st;
            var ar = re[a], ai = im[a], br = re[b], bi = im[b];
            re[a] = cb * ar + sb * bi; im[a] = cb * ai - sb * br;
            re[b] = cb * br + sb * ai; im[b] = cb * bi - sb * ar;
          }
        }
      }
    }
  };
  Simulator.prototype.expect = function (params) {
    this.evals++;
    this.run(params);
    var e = 0, re = this.re, im = this.im, C = this.C;
    for (var s = 0; s < this.N; s++) e += (re[s] * re[s] + im[s] * im[s]) * C[s];
    return e;
  };

  function nelderMead(f, x0, step, maxEvals) {
    var n = x0.length, pts = [x0.slice()], vals, evals = 0;
    for (var i = 0; i < n; i++) { var x = x0.slice(); x[i] += step; pts.push(x); }
    vals = pts.map(function (x) { evals++; return f(x); });
    function comb(a, b, t) { return a.map(function (v, i) { return v + t * (b[i] - v); }); }
    while (evals < maxEvals) {
      var idx = vals.map(function (v, i) { return i; }).sort(function (a, b) { return vals[a] - vals[b]; });
      pts = idx.map(function (i) { return pts[i]; }); vals = idx.map(function (i) { return vals[i]; });
      var cen = new Array(n).fill(0);
      for (var j = 0; j < n; j++) for (var d = 0; d < n; d++) cen[d] += pts[j][d] / n;
      var xr = comb(cen, pts[n], -1), fr = f(xr); evals++;
      if (fr < vals[0]) {
        var xe = comb(cen, pts[n], -2), fe = f(xe); evals++;
        if (fe < fr) { pts[n] = xe; vals[n] = fe; } else { pts[n] = xr; vals[n] = fr; }
      } else if (fr < vals[n - 1]) {
        pts[n] = xr; vals[n] = fr;
      } else {
        var xc = comb(cen, pts[n], 0.5), fc = f(xc); evals++;
        if (fc < vals[n]) { pts[n] = xc; vals[n] = fc; }
        else {
          for (var k = 1; k <= n; k++) { pts[k] = comb(pts[0], pts[k], 0.5); vals[k] = f(pts[k]); evals++; }
        }
      }
    }
    var b = 0;
    for (var m = 1; m < vals.length; m++) if (vals[m] < vals[b]) b = m;
    return { x: pts[b], f: vals[b] };
  }

  // INTERP initialisation for depth p+1 from depth-p angles (Zhou et al. 2020)
  function interp(params) {
    var p = params.length / 2, out = [];
    [params.slice(0, p), params.slice(p)].forEach(function (v) {
      var ext = [0].concat(v, [0]);
      for (var i = 1; i <= p + 1; i++) out.push(((i - 1) / p) * ext[i - 1] + ((p - i + 1) / p) * ext[i]);
    });
    return out;
  }

  function train(sim, reps, budget) {
    var best = null;
    var gammas = [0.05, 0.12, 0.3, 0.7, 1.5, 3.0], betas = [0.15, 0.4, 0.7, 1.0, 1.3];
    gammas.forEach(function (g) {
      betas.forEach(function (b) {
        var v = sim.expect([g, b]);
        if (!best || v < best.f) best = { x: [g, b], f: v };
      });
    });
    best = nelderMead(function (x) { return sim.expect(x); }, best.x, 0.1, budget);
    for (var p = 2; p <= reps; p++) {
      best = nelderMead(function (x) { return sim.expect(x); }, interp(best.x), 0.08, budget + 10 * p);
    }
    return best;
  }

  // Valid start->goal path encoded by basis state s, or null
  function decode(s, corr, qubo) {
    var deg = new Array(corr.anchors.length).fill(0), adj = {}, sel = [];
    for (var i = 0; i < qubo.nEdges; i++) {
      if (!((s >> i) & 1)) continue;
      var e = corr.edges[i];
      sel.push(i); deg[e.u]++; deg[e.v]++;
      (adj[e.u] = adj[e.u] || []).push(i);
      (adj[e.v] = adj[e.v] || []).push(i);
    }
    if (!sel.length || deg[0] !== 1 || deg[1] !== 1) return null;
    for (var a = 2; a < deg.length; a++) if (deg[a] !== 0 && deg[a] !== 2) return null;
    var node = 0, prevEdge = -1, cells = [], energy = 0, visited = { 0: true };
    while (node !== 1) {
      var nxt = adj[node].filter(function (ei) { return ei !== prevEdge; });
      if (nxt.length !== 1) return null;
      var e = corr.edges[nxt[0]], other = e.u === node ? e.v : e.u;
      if (visited[other]) return null;
      var seg = e.u === node ? e.cells : e.cells.slice().reverse();
      cells = cells.length ? cells.concat(seg.slice(1)) : seg.slice();
      energy += e.w; visited[other] = true; prevEdge = nxt[0]; node = other;
    }
    var seen = {};
    for (var c = 0; c < cells.length; c++) {
      var k = cells[c][0] + "," + cells[c][1];
      if (seen[k]) return null;                     // must be a simple path
      seen[k] = true;
    }
    return { cells: cells, energy: energy };
  }

  function sampleStates(probs, shots, rng) {
    var N = probs.length, cum = new Float64Array(N), acc = 0, out = [];
    for (var s = 0; s < N; s++) { acc += probs[s]; cum[s] = acc; }
    for (var k = 0; k < shots; k++) {
      var r = rng() * acc, lo = 0, hi = N - 1;
      while (lo < hi) { var m = (lo + hi) >> 1; if (cum[m] < r) lo = m + 1; else hi = m; }
      out.push(lo);
    }
    return out;
  }

  function plan(o) {
    var reps = o.reps || 2, shots = o.shots || 2048, seed = o.seed || 7;
    var stats = { qubits: 0, reps: reps, shots: shots, validSamples: 0, distinctValid: 0,
      expectation: 0, corridorEdges: 0, walkCells: 0, selectedCount: 0,
      randomValidSamples: 0, bestOfShotsEnergy: null, mostLikelyEnergy: null, trainEvals: 0, error: null };
    var corr = buildCorridor(o, o.maxEdges || 12, o.widthRatio || 0.5);
    if (corr.error) { stats.error = corr.error; return { path: null, stats: stats }; }
    stats.corridorEdges = corr.edges.length;
    stats.walkCells = corr.walkCells;
    var qubo = buildQubo(corr);
    stats.qubits = qubo.nVar;
    var C = diagonal(qubo);
    var mean = 0, min = Infinity;
    for (var s = 0; s < C.length; s++) { mean += C[s]; if (C[s] < min) min = C[s]; }
    mean /= C.length;
    var v = 0;
    for (s = 0; s < C.length; s++) v += (C[s] - mean) * (C[s] - mean);
    var sd = Math.sqrt(v / C.length) || 1;
    var Cn = new Float64Array(C.length);
    for (s = 0; s < C.length; s++) Cn[s] = (C[s] - min) / sd;   // rescale only: same ground states

    var sim = new Simulator(Cn, qubo.nVar);
    var best = train(sim, reps, o.budget || 30);
    stats.trainEvals = sim.evals;
    sim.run(best.x);
    var probs = new Float64Array(sim.N), ex = 0;
    for (s = 0; s < sim.N; s++) { probs[s] = sim.re[s] * sim.re[s] + sim.im[s] * sim.im[s]; ex += probs[s] * C[s]; }
    stats.expectation = ex;

    var rng = mulberry32(seed), counts = {};
    sampleStates(probs, shots, rng).forEach(function (st) { counts[st] = (counts[st] || 0) + 1; });
    var chosen = null, bestE = null;
    Object.keys(counts).forEach(function (k) {
      var d = decode(+k, corr, qubo);
      if (!d) return;
      stats.validSamples += counts[k]; stats.distinctValid++;
      if (bestE === null || d.energy < bestE) bestE = d.energy;
      if (!chosen || counts[k] > chosen.count || (counts[k] === chosen.count && d.energy < chosen.d.energy)) {
        chosen = { count: counts[k], d: d };
      }
    });
    var rrng = mulberry32(seed + 1), cache = {};
    for (var r = 0; r < shots; r++) {               // uniform-random baseline, same shots
      var st = Math.floor(rrng() * sim.N);
      if (cache[st] === undefined) cache[st] = !!decode(st, corr, qubo);
      if (cache[st]) stats.randomValidSamples++;
    }
    stats.bestOfShotsEnergy = bestE;
    if (!chosen) {
      stats.error = "QAOA sampled no valid path in " + shots + " shots (no classical fallback)";
      return { path: null, stats: stats };
    }
    stats.selectedCount = chosen.count;
    stats.mostLikelyEnergy = chosen.d.energy;
    var path = chosen.d.cells.map(function (c) { return corr.cor.world(c); });
    return { path: path, stats: stats };
  }

  var api = { plan: plan, latticeShape: latticeShape, interp: interp, _decode: decode,
    _buildCorridor: buildCorridor, _buildQubo: buildQubo, _diagonal: diagonal, Simulator: Simulator };
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  else root.QaoaCore = api;
})(this);
