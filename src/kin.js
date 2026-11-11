// Kinematics and rigid-body dynamics for a serial revolute arm described URDF-style.
// Pure functions, no dependencies. Matrices are row-major arrays of length 9.

export const v3 = {
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  scale: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: (a) => Math.hypot(a[0], a[1], a[2]),
};

export const m3 = {
  mul: (A, B) => {
    const C = new Array(9);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
    return C;
  },
  vec: (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]],
  T: (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]],
  I: () => [1, 0, 0, 0, 1, 0, 0, 0, 1],
  col: (A, j) => [A[j], A[3 + j], A[6 + j]],
};

export function rpyToR([r, p, y]) {
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
  // URDF: R = Rz(y) Ry(p) Rx(r)
  return [
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr,
  ];
}

export function axisAngle(k, t) {
  const c = Math.cos(t), s = Math.sin(t), v = 1 - c;
  const [x, y, z] = k;
  return [
    x * x * v + c, x * y * v - z * s, x * z * v + y * s,
    x * y * v + z * s, y * y * v + c, y * z * v - x * s,
    x * z * v - y * s, y * z * v + x * s, z * z * v + c,
  ];
}

const symI = ([xx, xy, xz, yy, yz, zz]) => [xx, xy, xz, xy, yy, yz, xz, yz, zz];

// Composite rigid body for link 7 + tool + payload (point masses at local points).
export function compositeLink(link, extras) {
  let m = link.mass;
  let c = v3.scale(link.com, link.mass);
  for (const e of extras) { m += e.mass; c = v3.add(c, v3.scale(e.p, e.mass)); }
  c = v3.scale(c, 1 / m);
  const I = symI(link.inertia).slice();
  const addPA = (mass, r) => {
    const d = v3.sub(r, c), dd = v3.dot(d, d);
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) I[i * 3 + j] += mass * ((i === j ? dd : 0) - d[i] * d[j]);
  };
  addPA(link.mass, link.com);
  for (const e of extras) addPA(e.mass, e.p);
  return { mass: m, com: c, I };
}

// Build a robot instance: tool length (m), tool mass (kg), payload (kg) at the TCP.
export function buildRobot(model, { toolLength = 0.16, toolMass = 1.0, payload = 0 } = {}) {
  const Rf = rpyToR(model.flange.rpy);
  // tool points in link-7 frame: flange origin + Rf * [0,0,z]
  const inL7 = (z) => v3.add(model.flange.xyz, m3.vec(Rf, [0, 0, z]));
  const links = model.links.map((l) => ({ mass: l.mass, com: l.com, I: symI(l.inertia) }));
  links[6] = compositeLink(model.links[6], [
    { mass: toolMass, p: inL7(toolLength / 2) },
    { mass: payload, p: inL7(toolLength) },
  ].filter((e) => e.mass > 0));
  return {
    model, toolLength, toolMass, payload, links, n: model.joints.length,
    Rroot: rpyToR(model.root.rpy),
    Rorig: model.joints.map((j) => rpyToR(j.rpy)),
    Rflange: Rf,
    qdmax: model.joints.map((j) => j.velocity),
    taumax: model.joints.map((j) => j.effort),
    lower: model.joints.map((j) => j.lower),
    upper: model.joints.map((j) => j.upper),
    movingMass: model.links.reduce((a, l) => a + l.mass, 0),
  };
}

// Forward kinematics. Returns world rotation/position of each link frame 0..7,
// joint axes z[i] and origins o[i] (i = 0..6 for joints A1..A7), flange and TCP frames.
export function fk(robot, q) {
  const R = [robot.Rroot], P = [[0, 0, 0]], z = [], o = [];
  for (let i = 0; i < robot.n; i++) {
    const j = robot.model.joints[i];
    const Rp = R[i], pp = P[i];
    const oi = v3.add(pp, m3.vec(Rp, j.xyz));
    const Ro = m3.mul(Rp, robot.Rorig[i]);
    const zi = m3.vec(Ro, j.axis);
    z.push(zi); o.push(oi);
    R.push(m3.mul(Ro, axisAngle(j.axis, q[i])));
    P.push(oi);
  }
  const Rf = m3.mul(R[7], robot.Rflange);
  const pf = v3.add(P[7], m3.vec(R[7], robot.model.flange.xyz));
  const ptcp = v3.add(pf, m3.vec(Rf, [0, 0, robot.toolLength]));
  return { R, P, z, o, Rf, pf, ptcp };
}

// World position of a contact point (see CONTACT_POINTS in model.js).
export function pointWorld(robot, F, cp) {
  if (cp.link === 8) {
    const zl = cp.p[2] === 'tool' ? robot.toolLength : cp.p[2];
    return v3.add(F.pf, m3.vec(F.Rf, [cp.p[0], cp.p[1], zl]));
  }
  return v3.add(F.P[cp.link], m3.vec(F.R[cp.link], cp.p));
}

// Linear Jacobian (3 x n, as array of columns) of a world point rigidly attached to link `link`.
export function jacPoint(robot, F, p, link) {
  const nAct = Math.min(link, robot.n);
  const cols = [];
  for (let i = 0; i < robot.n; i++) cols.push(i < nAct ? v3.cross(F.z[i], v3.sub(p, F.o[i])) : [0, 0, 0]);
  return cols;
}
export const jacApply = (cols, qd) => cols.reduce((acc, c, i) => v3.add(acc, v3.scale(c, qd[i])), [0, 0, 0]);

// Joint-space mass matrix via M = sum m J_v^T J_v + J_w^T I_w J_w (independent of rnea()).
export function massMatrix(robot, q, F = fk(robot, q)) {
  const n = robot.n, M = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let k = 1; k <= n; k++) {
    const L = robot.links[k - 1];
    const c = v3.add(F.P[k], m3.vec(F.R[k], L.com));
    const Jv = jacPoint(robot, F, c, k);
    const Iw = m3.mul(m3.mul(F.R[k], L.I), m3.T(F.R[k]));
    for (let a = 0; a < n; a++) {
      const wa = a < k ? F.z[a] : [0, 0, 0];
      const Iwa = m3.vec(Iw, wa);
      for (let b = 0; b < n; b++) {
        const wb = b < k ? F.z[b] : [0, 0, 0];
        M[a][b] += L.mass * v3.dot(Jv[a], Jv[b]) + v3.dot(wb, Iwa);
      }
    }
  }
  return M;
}

// Recursive Newton-Euler inverse dynamics in the world frame.
// tau = M(q) qdd + C(q, qd) qd + g(q). Pass g = 0 to drop gravity.
export function rnea(robot, q, qd, qdd, g = 9.81, F = fk(robot, q)) {
  const n = robot.n;
  const w = [], al = [], ap = [], ac = [], cw = [];
  let wPrev = [0, 0, 0], aPrev = [0, 0, 0], apPrev = [0, 0, g], pPrev = [0, 0, 0];
  for (let i = 0; i < n; i++) {
    const zi = F.z[i], oi = F.o[i];
    const r = v3.sub(oi, pPrev);
    const api = v3.add(apPrev, v3.add(v3.cross(aPrev, r), v3.cross(wPrev, v3.cross(wPrev, r))));
    const wi = v3.add(wPrev, v3.scale(zi, qd[i]));
    const ali = v3.add(v3.add(aPrev, v3.scale(zi, qdd[i])), v3.cross(wPrev, v3.scale(zi, qd[i])));
    const L = robot.links[i];
    const ci = v3.add(F.P[i + 1], m3.vec(F.R[i + 1], L.com));
    const rc = v3.sub(ci, oi);
    const aci = v3.add(api, v3.add(v3.cross(ali, rc), v3.cross(wi, v3.cross(wi, rc))));
    w.push(wi); al.push(ali); ap.push(api); ac.push(aci); cw.push(ci);
    wPrev = wi; aPrev = ali; apPrev = api; pPrev = oi;
  }
  const tau = new Array(n).fill(0);
  let fNext = [0, 0, 0], nNext = [0, 0, 0];
  for (let i = n - 1; i >= 0; i--) {
    const L = robot.links[i];
    const Iw = m3.mul(m3.mul(F.R[i + 1], L.I), m3.T(F.R[i + 1]));
    const fi = v3.add(v3.scale(ac[i], L.mass), fNext);
    let ni = v3.add(m3.vec(Iw, al[i]), v3.cross(w[i], m3.vec(Iw, w[i])));
    ni = v3.add(ni, nNext);
    ni = v3.add(ni, v3.cross(v3.sub(cw[i], F.o[i]), v3.scale(ac[i], L.mass)));
    if (i < n - 1) ni = v3.add(ni, v3.cross(v3.sub(F.o[i + 1], F.o[i]), fNext));
    tau[i] = v3.dot(ni, F.z[i]);
    fNext = fi; nNext = ni;
  }
  return tau;
}

// Small dense linear algebra helpers.
export function solve(A, b) {
  const n = A.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    [M[c], M[p]] = [M[p], M[c]];
    const d = M[c][c];
    if (Math.abs(d) < 1e-14) throw new Error('singular matrix');
    for (let r = 0; r < n; r++) if (r !== c) {
      const f = M[r][c] / d;
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((r, i) => r[n] / r[i]);
}

// Reflected (effective) mass of the arm at a point moving in unit direction u:
// m_u = 1 / (u^T J M^-1 J^T u)  (Khatib 1995). Exact for this model.
export function reflectedMass(robot, q, cp, F = fk(robot, q), u = null, qd = null) {
  const p = pointWorld(robot, F, cp);
  const J = jacPoint(robot, F, p, cp.link === 8 ? 7 : cp.link);
  let dir = u;
  if (!dir) {
    const v = jacApply(J, qd);
    const s = v3.norm(v);
    if (s < 1e-12) return Infinity;
    dir = v3.scale(v, 1 / s);
  }
  const M = massMatrix(robot, q, F);
  const jtU = J.map((c) => v3.dot(c, dir)); // J^T u
  const x = solve(M, jtU);
  const denom = jtU.reduce((a, v, i) => a + v * x[i], 0);
  return denom > 1e-15 ? 1 / denom : Infinity;
}

// Damped-least-squares IK for TCP position + tool axis pointing along `dirDown`,
// with a null-space pull toward qRest. Returns { q, err }.
export function ik(robot, target, q0, { qRest = null, iters = 200, tol = 1e-6, dir = [0, 0, -1], active = null } = {}) {
  let q = q0.slice();
  const n = robot.n;
  for (let it = 0; it < iters; it++) {
    const F = fk(robot, q);
    const ep = v3.sub(target, F.ptcp);
    const zt = m3.col(F.Rf, 2);
    const eo = v3.cross(zt, dir); // rotation error (axis only, yaw free)
    const e = [...ep, ...eo];
    const errN = Math.hypot(...e);
    if (errN < tol) return { q, err: errN, iters: it };
    const Jp = jacPoint(robot, F, F.ptcp, 7);
    const J = [0, 1, 2].map((r) => Jp.map((c) => c[r])).concat([0, 1, 2].map((r) => F.z.map((z) => z[r])));
    if (active) for (const row of J) for (let k = 0; k < n; k++) if (!active[k]) row[k] = 0;
    const lam2 = 1e-4;
    const JJt = J.map((ri) => J.map((rj) => ri.reduce((a, v, k) => a + v * rj[k], 0)));
    for (let i = 0; i < 6; i++) JJt[i][i] += lam2;
    const y = solve(JJt, e);
    const dq = new Array(n).fill(0);
    for (let k = 0; k < n; k++) for (let i = 0; i < 6; i++) dq[k] += J[i][k] * y[i];
    if (qRest) {
      // null-space projection of (qRest - q)
      const g = qRest.map((r, k) => 0.1 * (r - q[k]));
      const Jg = J.map((ri) => ri.reduce((a, v, k) => a + v * g[k], 0));
      const z2 = solve(JJt, Jg);
      for (let k = 0; k < n; k++) {
        let pr = g[k];
        for (let i = 0; i < 6; i++) pr -= J[i][k] * z2[i];
        dq[k] += pr;
      }
    }
    const step = Math.max(...dq.map(Math.abs));
    const s = step > 0.2 ? 0.2 / step : 1;
    q = q.map((v, k) => Math.min(robot.upper[k], Math.max(robot.lower[k], v + s * dq[k])));
  }
  const F = fk(robot, q);
  const e = [...v3.sub(target, F.ptcp), ...v3.cross(m3.col(F.Rf, 2), dir)];
  return { q, err: Math.hypot(...e), iters };
}
