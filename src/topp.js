// Time-optimal path parameterization by reachability analysis (TOPP-RA, Pham & Pham, IEEE T-RO 2018).
// Variables on a grid s_0..s_N: x_i = sdot_i^2, u_i = sddot_i, with x_{i+1} = x_i + 2 * delta_i * u_i.
// Each stage i has
//   0 <= x_i <= xmax_i                    (velocity-type constraints: joint speed, contact-safe speed, TCP cap)
//   lo_ik <= a_ik u + b_ik x + c_ik <= hi_ik   (acceleration-type: joint torque, joint acceleration)
// Backward pass computes the controllable sets K_i; the forward pass picks the largest admissible u.
// The result is time-optimal for the discretized problem, and infeasibility is detected exactly.

const EPS = 1e-10;

// Solve min and max of x over the polygon { (x,u): A [x,u] <= b }. Returns null if empty.
// Vertex enumeration: fine for the ~30 half-planes per stage used here.
export function lp2Range(rows) {
  let lo = Infinity, hi = -Infinity;
  const n = rows.length;
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      const [a1, b1, c1] = rows[i], [a2, b2, c2] = rows[j];
      const det = a1 * b2 - a2 * b1;
      if (Math.abs(det) < 1e-14) continue;
      const x = (c1 * b2 - c2 * b1) / det;
      const u = (a1 * c2 - a2 * c1) / det;
      let ok = true;
      for (let k = 0; k < n; k++) {
        const [a, b, c] = rows[k];
        if (a * x + b * u > c + EPS * (1 + Math.abs(c))) { ok = false; break; }
      }
      if (ok) { if (x < lo) lo = x; if (x > hi) hi = x; }
    }
  }
  return lo === Infinity ? null : [Math.max(0, lo), hi];
}

// Stage constraints as half-planes over (x, u): rows [ax, au, rhs] meaning ax*x + au*u <= rhs.
function stageRows(stage) {
  const rows = [[-1, 0, 0], [1, 0, Math.min(stage.xmax, 1e6)]];
  for (const k of stage.acc) {
    // lo <= a u + b x + c <= hi
    rows.push([k.b, k.a, k.hi - k.c]);
    rows.push([-k.b, -k.a, -(k.lo - k.c)]);
  }
  return rows;
}

// stages: array of N+1 { xmax, acc: [{a,b,c,lo,hi}] }, grid: s values (length N+1).
// xStart, xEnd: boundary squared speeds (0 for rest-to-rest).
export function toppra(grid, stages, { xStart = 0, xEnd = 0 } = {}) {
  const N = grid.length - 1;
  const K = new Array(N + 1);
  const xe = Math.min(xEnd, stages[N].xmax, 1e6);
  K[N] = [xe, xe];
  for (let i = N - 1; i >= 0; i--) {
    const d = grid[i + 1] - grid[i];
    const rows = stageRows(stages[i]);
    // K_{i+1}.lo <= x + 2 d u <= K_{i+1}.hi
    rows.push([1, 2 * d, K[i + 1][1]]);
    rows.push([-1, -2 * d, -K[i + 1][0]]);
    const r = lp2Range(rows);
    if (!r || r[1] < r[0] - 1e-12) return { ok: false, failedAt: i, K };
    K[i] = r;
  }
  if (xStart < K[0][0] - 1e-9 || xStart > K[0][1] + 1e-9) return { ok: false, failedAt: 0, K };
  const x = new Array(N + 1), u = new Array(N);
  x[0] = xStart;
  for (let i = 0; i < N; i++) {
    const d = grid[i + 1] - grid[i];
    // feasible u interval given x_i
    let uLo = -Infinity, uHi = Infinity;
    for (const k of stages[i].acc) {
      const base = k.b * x[i] + k.c;
      if (Math.abs(k.a) < 1e-14) continue;
      const l = (k.lo - base) / k.a, h = (k.hi - base) / k.a;
      uLo = Math.max(uLo, Math.min(l, h));
      uHi = Math.min(uHi, Math.max(l, h));
    }
    uLo = Math.max(uLo, (K[i + 1][0] - x[i]) / (2 * d));
    uHi = Math.min(uHi, (K[i + 1][1] - x[i]) / (2 * d));
    if (uHi < uLo - 1e-6) return { ok: false, failedAt: i, K };
    u[i] = Math.max(uLo, Math.min(uHi, uHi));
    x[i + 1] = Math.max(0, x[i] + 2 * d * u[i]);
  }
  // time stamps
  const t = [0];
  for (let i = 0; i < N; i++) {
    const v0 = Math.sqrt(x[i]), v1 = Math.sqrt(x[i + 1]);
    const dt = v0 + v1 > 1e-12 ? (2 * (grid[i + 1] - grid[i])) / (v0 + v1) : Infinity;
    t.push(t[i] + dt);
  }
  return { ok: Number.isFinite(t[N]), x, u, t, K, duration: t[N] };
}
