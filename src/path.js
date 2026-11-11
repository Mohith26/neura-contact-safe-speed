// Geometric path: Cartesian polyline with blended corners -> dense IK samples -> C2 joint spline q(s),
// where s is the TCP arc length in metres (so |dp_tcp/ds| = 1 along the path).
import { ik, fk, v3 } from './kin.js';

// A3 (upper-arm roll) and A7 (flange roll) are held at their start values: the gripper is
// symmetric about its axis, and fixing the redundancy keeps q(s) smooth and unique.
export const ACTIVE = [true, true, false, true, true, true, false];

// Polyline with quadratic-Bezier corner blends, resampled uniformly by arc length.
export function blendedPolyline(pts, blend, ds = 0.005) {
  const curve = [];
  const push = (p) => curve.push(p);
  push(pts[0]);
  for (let i = 1; i < pts.length - 1; i++) {
    const a = pts[i - 1], b = pts[i], c = pts[i + 1];
    const lin = v3.norm(v3.sub(b, a)), lout = v3.norm(v3.sub(c, b));
    const d = Math.min(blend, 0.45 * lin, 0.45 * lout);
    const din = v3.scale(v3.sub(b, a), 1 / lin), dout = v3.scale(v3.sub(c, b), 1 / lout);
    const p0 = v3.sub(b, v3.scale(din, d)), p2 = v3.add(b, v3.scale(dout, d));
    push(p0);
    for (let k = 1; k < 24; k++) {
      const t = k / 24;
      push(v3.add(v3.add(v3.scale(p0, (1 - t) ** 2), v3.scale(b, 2 * (1 - t) * t)), v3.scale(p2, t * t)));
    }
    push(p2);
  }
  push(pts[pts.length - 1]);
  // dense linear interpolation, then uniform arc-length resampling
  const dense = [curve[0]];
  for (let i = 1; i < curve.length; i++) {
    const a = curve[i - 1], b = curve[i];
    const L = v3.norm(v3.sub(b, a));
    const m = Math.max(1, Math.ceil(L / 0.001));
    for (let k = 1; k <= m; k++) dense.push(v3.add(a, v3.scale(v3.sub(b, a), k / m)));
  }
  const cum = [0];
  for (let i = 1; i < dense.length; i++) cum.push(cum[i - 1] + v3.norm(v3.sub(dense[i], dense[i - 1])));
  const total = cum[cum.length - 1];
  const n = Math.max(2, Math.round(total / ds) + 1);
  const out = [];
  let j = 0;
  for (let i = 0; i < n; i++) {
    const s = (total * i) / (n - 1);
    while (j < cum.length - 2 && cum[j + 1] < s) j++;
    const t = cum[j + 1] > cum[j] ? (s - cum[j]) / (cum[j + 1] - cum[j]) : 0;
    out.push({ s, p: v3.add(dense[j], v3.scale(v3.sub(dense[j + 1], dense[j]), t)) });
  }
  return { samples: out, length: total };
}

// Natural cubic spline through (x_i, y_i); returns evaluator for value, 1st and 2nd derivative.
export function cubicSpline(xs, ys) {
  const n = xs.length;
  const h = xs.slice(1).map((x, i) => x - xs[i]);
  const al = new Array(n).fill(0);
  for (let i = 1; i < n - 1; i++) al[i] = (3 / h[i]) * (ys[i + 1] - ys[i]) - (3 / h[i - 1]) * (ys[i] - ys[i - 1]);
  const l = new Array(n).fill(1), mu = new Array(n).fill(0), z = new Array(n).fill(0);
  for (let i = 1; i < n - 1; i++) {
    l[i] = 2 * (xs[i + 1] - xs[i - 1]) - h[i - 1] * mu[i - 1];
    mu[i] = h[i] / l[i];
    z[i] = (al[i] - h[i - 1] * z[i - 1]) / l[i];
  }
  const c = new Array(n).fill(0), b = new Array(n).fill(0), d = new Array(n).fill(0);
  for (let j = n - 2; j >= 0; j--) {
    c[j] = z[j] - mu[j] * c[j + 1];
    b[j] = (ys[j + 1] - ys[j]) / h[j] - (h[j] * (c[j + 1] + 2 * c[j])) / 3;
    d[j] = (c[j + 1] - c[j]) / (3 * h[j]);
  }
  const seg = (x) => {
    let lo = 0, hi = n - 2;
    while (lo < hi) { const m = (lo + hi + 1) >> 1; if (xs[m] <= x) lo = m; else hi = m - 1; }
    return lo;
  };
  return (x) => {
    const i = seg(Math.min(Math.max(x, xs[0]), xs[n - 1]));
    const t = x - xs[i];
    return [ys[i] + b[i] * t + c[i] * t * t + d[i] * t * t * t, b[i] + 2 * c[i] * t + 3 * d[i] * t * t, 2 * c[i] + 6 * d[i] * t];
  };
}

// Build a joint-space path for one motion. Returns { length, q(s) -> {q, dq, ddq}, knots }.
export function buildMotionPath(robot, pts, { blend = 0.06, ds = 0.005, qSeed, qRest } = {}) {
  const { samples, length } = blendedPolyline(pts, blend, ds);
  let q = qSeed.slice();
  const qs = [];
  let maxErr = 0;
  for (const smp of samples) {
    const r = ik(robot, smp.p, q, { iters: 300, tol: 1e-9, active: ACTIVE });
    maxErr = Math.max(maxErr, r.err);
    q = r.q;
    qs.push(q);
  }
  const xs = samples.map((s) => s.s);
  const splines = robot.model.joints.map((_, j) => cubicSpline(xs, qs.map((qq) => qq[j])));
  const evalS = (s) => {
    const r = splines.map((f) => f(s));
    return { q: r.map((v) => v[0]), dq: r.map((v) => v[1]), ddq: r.map((v) => v[2]) };
  };
  return { length, eval: evalS, ikMaxErr: maxErr, qStart: qs[0], qEnd: qs[qs.length - 1], cart: samples.map((s) => s.p) };
}

export function buildTaskPaths(robot, task, qRest) {
  let seed = qRest.slice();
  const out = [];
  for (const m of task.motions) {
    // converge the first point from the previous configuration
    const first = ik(robot, m.pts[0], seed, { iters: 800, tol: 1e-10, active: ACTIVE });
    const path = buildMotionPath(robot, m.pts, { blend: task.blend, qSeed: first.q, qRest });
    out.push({ name: m.name, ...path });
    seed = path.qEnd;
  }
  return out;
}

export const tcpOf = (robot, q) => fk(robot, q).ptcp;
