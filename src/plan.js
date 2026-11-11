// Contact-safe speed planning: per-point, per-body-region ISO/TS 15066 speed limits along the path,
// fed into TOPP-RA together with MAiRA's joint speed / torque limits, then checked by an
// independent verifier at 4x the planning resolution.
import { fk, rnea, jacPoint, jacApply, pointWorld, reflectedMass, v3, massMatrix, solve } from './kin.js';
import { BODY, maxSpeed, contactForce, permissibleForce, reducedMass, isoRobotMass } from './iso15066.js';
import { allZones, inZone } from './cell.js';
import { CONTACT_POINTS } from './model.js';
import { toppra } from './topp.js';

export const DEFAULTS = {
  gridStep: 0.01, // m of TCP arc length between TOPP stages
  torqueFraction: 0.9, // plan to 90% of published joint torque limits
  // Not published for MAiRA; assumed values, labelled as such in the UI and README.
  jointAccel: [6, 6, 8, 8, 12, 12, 16],
  planMargin: 0.97, // plan to 97% of each contact speed limit
  zoneMargin: 0.01, // plan with zones inflated by 1 cm
  clampGap: 0.06, // a body part can be pinned if the robot surface is within 6 cm of the bench and moving down
  massModel: 'max', // 'iso' | 'reflected' | 'max' (default: the larger of ISO A.3.2 and the model-based reflected mass)
  contactArea: { shoulder: 10, upperarm: 10, elbow: 8, forearm: 8, wrist: 6, flange: 4, tcp: 1.5 }, // cm^2
};

export const STRATEGIES = {
  contactSafe: { label: 'Contact-safe speed', short: 'Contact-safe' },
  singleLimit: { label: 'Single collaborative limit', short: 'Single limit' },
  cobot250: { label: 'Cobot speed (key switch I, 250 mm/s)', short: 'Cobot speed' },
  fullSpeed: { label: 'Full speed (key switch II)', short: 'Full speed' },
};

function robotMass(robot, q, cp, F, dir, model) {
  const iso = isoRobotMass(robot.movingMass, robot.toolMass + robot.payload);
  const refl = reflectedMass(robot, q, cp, F, dir);
  if (model === 'iso') return { mR: iso, iso, refl };
  if (model === 'reflected') return { mR: refl, iso, refl };
  return { mR: Math.max(iso, refl), iso, refl };
}

// Analyse one configuration on the path: contact-point speeds per unit sdot and their limits.
export function analysePoint(robot, zones, q, dq, opts, { margin = 0, withMass = true } = {}) {
  const F = fk(robot, q);
  const pts = [];
  for (const cp of CONTACT_POINTS) {
    const p = pointWorld(robot, F, cp);
    const J = jacPoint(robot, F, p, cp.link === 8 ? 7 : cp.link);
    const w = jacApply(J, dq);
    const wn = v3.norm(w);
    const dir = wn > 1e-12 ? v3.scale(w, 1 / wn) : [0, 0, 0];
    let best = null;
    for (const z of zones) {
      if (!inZone(z, p, cp.r + margin)) continue;
      const area = opts.contactArea[cp.id];
      if (z.keepOut) { best = { zone: z, region: z.region, contact: 'keep-out', vlim: 0, mR: null, Flim: 0 }; break; }
      const clamp = z.clampSurfaceZ !== undefined && p[2] - cp.r - z.clampSurfaceZ <= opts.clampGap && dir[2] < -0.5;
      const contact = clamp ? 'quasi-static' : 'transient';
      const m = withMass ? robotMass(robot, q, cp, F, dir, opts.massModel) : { mR: isoRobotMass(robot.movingMass, robot.toolMass + robot.payload) };
      for (const region of [z.region, ...(z.alsoRegions || [])]) {
        const vlim = maxSpeed(region, contact, m.mR, area);
        if (!best || vlim < best.vlim) best = { zone: z, region, contact, vlim, mR: m.mR, mIso: m.iso, mRefl: m.refl, Flim: permissibleForce(region, contact, area), area };
      }
    }
    pts.push({ cp, p, w, wn, dir, lim: best });
  }
  return { F, pts };
}

function stageFor(robot, path, s, zones, strategy, opts, vSingle) {
  const { q, dq, ddq } = path.eval(s);
  let xmax = Infinity;
  const bounds = [];
  robot.qdmax.forEach((m, k) => { if (Math.abs(dq[k]) > 1e-9) { const b = (0.99 * m / Math.abs(dq[k])) ** 2; bounds.push(['joint ' + robot.model.joints[k].name, b]); xmax = Math.min(xmax, b); } });
  const A = analysePoint(robot, zones, q, dq, opts, { margin: opts.zoneMargin, withMass: strategy === 'contactSafe' || strategy === 'zones' });
  const tcp = A.pts[A.pts.length - 1];
  const cap = (v, wn) => (wn > 1e-9 ? (v / wn) ** 2 : Infinity);
  if (strategy === 'cobot250') { const b = cap(robot.model.cobotSpeed, tcp.wn); bounds.push(['TCP 250 mm/s', b]); xmax = Math.min(xmax, b); }
  else { const b = cap(robot.model.fullSpeed, tcp.wn); bounds.push(['TCP 4.5 m/s', b]); xmax = Math.min(xmax, b); }
  let binding = null;
  if (strategy === 'contactSafe') {
    for (const pt of A.pts) {
      if (!pt.lim) continue;
      const b = cap(opts.planMargin * pt.lim.vlim, pt.wn);
      if (b < xmax) { xmax = b; binding = pt; }
      bounds.push([pt.cp.id + '@' + pt.lim.zone.id, b]);
    }
  }
  if (strategy === 'singleLimit') {
    for (const pt of A.pts) { const b = cap(opts.planMargin * vSingle, pt.wn); if (b < xmax) xmax = b; }
  }
  const acc = [];
  const z0 = new Array(robot.n).fill(0);
  const a = rnea(robot, q, z0, dq, 0, A.F);
  const b = rnea(robot, q, dq, ddq, 0, A.F);
  const c = rnea(robot, q, z0, z0, 9.81, A.F);
  for (let k = 0; k < robot.n; k++) {
    const T = opts.torqueFraction * robot.taumax[k];
    acc.push({ a: a[k], b: b[k], c: c[k], lo: -T, hi: T, kind: 'torque', k });
    acc.push({ a: dq[k], b: ddq[k], c: 0, lo: -opts.jointAccel[k], hi: opts.jointAccel[k], kind: 'accel', k });
  }
  return { s, xmax, acc, A, binding };
}

// Plan one motion with one strategy.
export function planMotion(robot, path, zones, strategy, opts, vSingle) {
  const N = Math.max(2, Math.ceil(path.length / opts.gridStep));
  const grid = Array.from({ length: N + 1 }, (_, i) => (path.length * i) / N);
  const stages = grid.map((s) => stageFor(robot, path, s, zones, strategy, opts, vSingle));
  const keepOut = stages.findIndex((st) => st.A.pts.some((p) => p.lim && p.lim.contact === 'keep-out'));
  if (strategy === 'contactSafe' && keepOut >= 0) {
    const pt = stages[keepOut].A.pts.find((p) => p.lim && p.lim.contact === 'keep-out');
    return { ok: false, reason: 'keep-out', s: grid[keepOut], point: pt.cp.label, zone: pt.lim.zone.label || pt.lim.zone.id, grid, stages };
  }
  const r = toppra(grid, stages);
  if (!r.ok) return { ok: false, reason: 'infeasible', s: grid[r.failedAt ?? 0], grid, stages };
  return { ok: true, grid, stages, x: r.x, u: r.u, t: r.t, duration: r.duration };
}

// Independent verifier: re-evaluates every limit on the exact zones (no planning margin)
// at `sub` points per grid interval, using the closed-form s(t) of the TOPP-RA solution.
export function verifyMotion(robot, path, zones, plan, opts, sub = 4) {
  const viol = [];
  let maxForceRatio = 0, maxTorqueRatio = 0, maxJointSpeedRatio = 0, checked = 0, inZoneSamples = 0;
  const { grid, x, u } = plan;
  for (let i = 0; i < grid.length - 1; i++) {
    for (let m = 0; m < sub; m++) {
      const f = m / sub;
      const s = grid[i] + f * (grid[i + 1] - grid[i]);
      const xs = Math.max(0, x[i] + 2 * u[i] * (s - grid[i]));
      const sd = Math.sqrt(xs), sdd = u[i];
      const { q, dq, ddq } = path.eval(s);
      const qd = dq.map((v) => v * sd);
      const qdd = dq.map((v, k) => v * sdd + ddq[k] * xs);
      const tau = rnea(robot, q, qd, qdd);
      for (let k = 0; k < robot.n; k++) {
        maxTorqueRatio = Math.max(maxTorqueRatio, Math.abs(tau[k]) / robot.taumax[k]);
        maxJointSpeedRatio = Math.max(maxJointSpeedRatio, Math.abs(qd[k]) / robot.qdmax[k]);
        if (Math.abs(tau[k]) > robot.taumax[k] * (1 + 1e-6)) viol.push({ s, kind: 'torque', joint: k, value: tau[k] });
        if (Math.abs(qd[k]) > robot.qdmax[k] * (1 + 1e-6)) viol.push({ s, kind: 'joint speed', joint: k, value: qd[k] });
      }
      const A = analysePoint(robot, zones, q, dq, { ...opts, massModel: opts.massModel }, { margin: 0 });
      for (const pt of A.pts) {
        checked++;
        if (!pt.lim) continue;
        inZoneSamples++;
        const v = pt.wn * sd;
        if (pt.lim.contact === 'keep-out') { if (v > 1e-9) viol.push({ s, kind: 'keep-out', point: pt.cp.id, zone: pt.lim.zone.id, v }); continue; }
        const Fc = contactForce(pt.lim.region, pt.lim.mR, v);
        const ratio = Fc / pt.lim.Flim;
        maxForceRatio = Math.max(maxForceRatio, ratio);
        if (v > pt.lim.vlim * (1 + 1e-9)) viol.push({ s, kind: 'contact', point: pt.cp.id, zone: pt.lim.zone.id, region: pt.lim.region, contact: pt.lim.contact, v, vlim: pt.lim.vlim, F: Fc, Flim: pt.lim.Flim });
      }
    }
  }
  return { violations: viol, maxForceRatio, maxTorqueRatio, maxJointSpeedRatio, checked, inZoneSamples };
}

// Group contiguous stages by their binding contact constraint (for the segment table).
export function segmentsOf(plan, motionIndex, motionName, opts) {
  const segs = [];
  let cur = null;
  plan.stages.forEach((st, i) => {
    // the most restrictive contact point at this stage (smallest allowed sdot)
    let key = 'free', best = null;
    for (const pt of st.A.pts) {
      if (!pt.lim || pt.wn < 1e-9) continue;
      const sd = (opts.planMargin * pt.lim.vlim) / pt.wn;
      if (!best || sd < best.sd) best = { sd, pt };
    }
    if (best) key = best.pt.lim.zone.id + '|' + best.pt.lim.region + '|' + best.pt.lim.contact;
    const sd = Math.sqrt(plan.x[i]);
    if (!cur || cur.key !== key) {
      if (cur) segs.push(cur);
      cur = { key, motion: motionIndex, motionName, i0: i, i1: i, s0: plan.grid[i], s1: plan.grid[i], t0: plan.t[i], t1: plan.t[i], best: best ? best.pt : null, worst: null, vPointMax: 0, vlimMin: Infinity, tcpMax: 0, bindingFrac: 0, n: 0, points: {} };
    }
    cur.i1 = i; cur.s1 = plan.grid[i]; cur.t1 = plan.t[i]; cur.n++;
    cur.tcpMax = Math.max(cur.tcpMax, sd);
    if (best) {
      const v = best.pt.wn * sd;
      cur.points[best.pt.cp.id] = (cur.points[best.pt.cp.id] || 0) + 1;
      if (!cur.worst || v / best.pt.lim.vlim > cur.worst.ratio) cur.worst = { ratio: v / best.pt.lim.vlim, pt: best.pt, v, s: plan.grid[i] };
      cur.vPointMax = Math.max(cur.vPointMax, v);
      cur.vlimMin = Math.min(cur.vlimMin, best.pt.lim.vlim);
      if (v >= 0.995 * opts.planMargin * best.pt.lim.vlim) cur.bindingFrac++;
    }
  });
  if (cur) segs.push(cur);
  for (const g of segs) if (g.worst) g.best = g.worst.pt;
  return segs.filter((g) => g.t1 - g.t0 > 1e-3 || g.best);
}

// Full plan for a task: all strategies, verification, headline numbers.
export function planTask(robot, paths, task, operator, userOpts = {}) {
  const opts = { ...DEFAULTS, ...userOpts, contactArea: { ...DEFAULTS.contactArea, ...(userOpts.contactArea || {}) } };
  const zones = allZones(operator);
  const out = { task: task.id, operator, opts, strategies: {}, zones };
  // contact-safe first (also yields the most conservative single limit seen on this path)
  const results = {};
  const cs = paths.map((p) => planMotion(robot, p, zones, 'contactSafe', opts));
  results.contactSafe = cs;
  let vSingle = Infinity;
  if (cs.every((r) => r.ok)) {
    for (const r of cs) for (const st of r.stages) for (const pt of st.A.pts) if (pt.lim && pt.lim.vlim < vSingle) vSingle = pt.lim.vlim;
  }
  out.vSingle = vSingle;
  results.singleLimit = Number.isFinite(vSingle) ? paths.map((p) => planMotion(robot, p, zones, 'singleLimit', opts, vSingle)) : null;
  results.cobot250 = paths.map((p) => planMotion(robot, p, zones, 'cobot250', opts));
  results.fullSpeed = paths.map((p) => planMotion(robot, p, zones, 'fullSpeed', opts));
  const stops = paths.length; // one gripper action at the end of every motion
  for (const [k, res] of Object.entries(results)) {
    if (!res) { out.strategies[k] = { ok: false, reason: 'no limit' }; continue; }
    const ok = res.every((r) => r.ok);
    const entry = { ok, motions: [] };
    if (!ok) {
      const bad = res.find((r) => !r.ok);
      Object.assign(entry, { reason: bad.reason, s: bad.s, point: bad.point, zone: bad.zone, motion: res.indexOf(bad) });
      out.strategies[k] = entry;
      continue;
    }
    let total = 0, viol = 0, maxF = 0, maxTau = 0;
    res.forEach((r, mi) => {
      const ver = verifyMotion(robot, paths[mi], zones, r, opts);
      total += r.duration;
      viol += ver.violations.length;
      maxF = Math.max(maxF, ver.maxForceRatio);
      maxTau = Math.max(maxTau, ver.maxTorqueRatio);
      entry.motions.push({ name: paths[mi].name, duration: r.duration, violations: ver.violations.length, firstViolations: ver.violations.slice(0, 5), maxForceRatio: ver.maxForceRatio, maxTorqueRatio: ver.maxTorqueRatio, checked: ver.checked, inZoneSamples: ver.inZoneSamples });
    });
    entry.cycleTime = total + stops * task.dwell;
    entry.moveTime = total;
    entry.violations = viol;
    entry.maxForceRatio = maxF;
    entry.maxTorqueRatio = maxTau;
    out.strategies[k] = entry;
  }
  out.raw = results;
  if (out.strategies.contactSafe.ok) {
    out.segments = results.contactSafe.flatMap((r, mi) => segmentsOf(r, mi, paths[mi].name, opts));
  }
  return out;
}

export { BODY, reducedMass, massMatrix, solve };
