import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import { MAIRA7M } from '../src/model.js';
import * as K from '../src/kin.js';
import { BODY, maxSpeed, permissibleForce, contactForce } from '../src/iso15066.js';
import { toppra, lp2Range } from '../src/topp.js';
import { cubicSpline } from '../src/path.js';
import { TASKS, Q_REST, OPERATOR_DEFAULT } from '../src/cell.js';
import { runScenario, DEFAULT_SCENARIO, scenarioKey, getPaths } from '../src/report.js';

// deterministic PRNG for the randomized tests
function rng(seed) { let s = seed >>> 0; return () => ((s = (s * 1664525 + 1013904223) >>> 0) / 4294967296); }
const robot = K.buildRobot(MAIRA7M, { payload: 2 });
const randQ = (r) => robot.lower.map((lo, k) => 0.6 * (lo + (robot.upper[k] - lo) * r()));

test('FK: straight-up pose puts the TCP at the sum of the published link offsets', () => {
  const F = K.fk(robot, [0, 0, 0, 0, 0, 0, 0]);
  assert.ok(Math.abs(F.ptcp[2] - (0.438 + 0.7 + 0.7 + 0.115 + 0.16)) < 1e-9);
  assert.ok(Math.hypot(F.ptcp[0], F.ptcp[1]) < 1e-9);
});

test('dynamics (randomized): RNEA columns reproduce the independently built mass matrix', () => {
  const r = rng(7);
  for (let t = 0; t < 25; t++) {
    const q = randQ(r);
    const M = K.massMatrix(robot, q);
    for (let j = 0; j < 7; j++) {
      const e = new Array(7).fill(0); e[j] = 1;
      const col = K.rnea(robot, q, new Array(7).fill(0), e, 0);
      for (let i = 0; i < 7; i++) assert.ok(Math.abs(col[i] - M[i][j]) < 1e-9, `M[${i}][${j}]`);
    }
  }
});

test('dynamics (randomized): power balance qd . tau = d/dt kinetic energy without gravity', () => {
  const r = rng(11);
  const KE = (q, qd) => { const M = K.massMatrix(robot, q); let s = 0; for (let i = 0; i < 7; i++) for (let j = 0; j < 7; j++) s += qd[i] * M[i][j] * qd[j]; return s / 2; };
  for (let t = 0; t < 15; t++) {
    const q = randQ(r), qd = q.map(() => 2 * r() - 1), qdd = q.map(() => 2 * r() - 1);
    const h = 1e-6;
    const dKE = (KE(q.map((v, i) => v + qd[i] * h), qd.map((v, i) => v + qdd[i] * h)) - KE(q.map((v, i) => v - qd[i] * h), qd.map((v, i) => v - qdd[i] * h))) / (2 * h);
    const p = K.rnea(robot, q, qd, qdd, 0).reduce((a, x, i) => a + x * qd[i], 0);
    assert.ok(Math.abs(dKE - p) < 1e-5 * (1 + Math.abs(p)), `${dKE} vs ${p}`);
  }
});

test('reflected mass is positive, direction-symmetric and below the full arm mass for the wrist', () => {
  const r = rng(3);
  const cp = { link: 5, p: [0, 0.7, 0] };
  for (let t = 0; t < 10; t++) {
    const q = randQ(r);
    const u = [r() - 0.5, r() - 0.5, r() - 0.5]; const n = Math.hypot(...u); const d = u.map((x) => x / n);
    const a = K.reflectedMass(robot, q, cp, undefined, d), b = K.reflectedMass(robot, q, cp, undefined, d.map((x) => -x));
    assert.ok(a > 0 && Math.abs(a - b) < 1e-9 * a);
  }
});

test('ISO/TS 15066 energy model reproduces Annex A transfer energies (chest 1.6 J, hand 0.52 J)', () => {
  const E = (region) => (2 * BODY[region].F) ** 2 / (2 * BODY[region].k * 1000);
  assert.ok(Math.abs(E('chest') - 1.568) < 1e-3);
  assert.ok(Math.abs(E('hand') - 0.5227) < 1e-3);
  // v = F / sqrt(mu k) and back again
  const v = maxSpeed('hand', 'transient', 30.716, 5);
  assert.ok(Math.abs(contactForce('hand', 30.716, v) - 280) < 1e-9);
});

test('ISO/TS 15066: head and face are keep-out for transient contact; pressure binds for sharp edges', () => {
  assert.equal(maxSpeed('face', 'transient', 20, 5), 0);
  assert.equal(maxSpeed('skull', 'transient', 20, 5), 0);
  assert.equal(permissibleForce('hand', 'transient', 0.5), 2 * 95); // 190 N/cm^2 x 0.5 cm^2 < 140 N
  assert.equal(permissibleForce('hand', 'quasi-static', 5), 140);
});

test('TOPP-RA: acceleration-only problem gives the bang-bang optimum T = 2 sqrt(L/a)', () => {
  const L = 1, a = 2, N = 400;
  const grid = Array.from({ length: N + 1 }, (_, i) => (L * i) / N);
  const stages = grid.map(() => ({ xmax: Infinity, acc: [{ a: 1, b: 0, c: 0, lo: -a, hi: a }] }));
  const r = toppra(grid, stages);
  assert.ok(r.ok);
  assert.ok(Math.abs(r.duration - 2 * Math.sqrt(L / a)) < 0.01);
});

test('TOPP-RA: speed cap gives the trapezoid optimum T = L/v + v/a', () => {
  const L = 2, a = 4, v = 0.5, N = 800;
  const grid = Array.from({ length: N + 1 }, (_, i) => (L * i) / N);
  const stages = grid.map(() => ({ xmax: v * v, acc: [{ a: 1, b: 0, c: 0, lo: -a, hi: a }] }));
  const r = toppra(grid, stages);
  assert.ok(Math.abs(r.duration - (L / v + v / a)) < 0.01, String(r.duration));
});

test('TOPP-RA: a zero-speed stretch of path (keep-out) is reported as infeasible, not silently skipped', () => {
  const grid = Array.from({ length: 101 }, (_, i) => i / 100);
  const stages = grid.map((_, i) => ({ xmax: i >= 40 && i <= 60 ? 0 : 1, acc: [{ a: 1, b: 0, c: 0, lo: -1, hi: 1 }] }));
  const r = toppra(grid, stages);
  assert.equal(r.ok, false);
});

test('2-D LP (randomized): vertex enumeration matches a dense brute-force search', () => {
  const r = rng(42);
  for (let t = 0; t < 30; t++) {
    const rows = [[-1, 0, 0], [1, 0, 1 + r()], [0, 1, 2], [0, -1, 2]];
    for (let k = 0; k < 4; k++) rows.push([r() - 0.5, r() - 0.5, r() * 0.5 + 0.1]);
    const res = lp2Range(rows);
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i <= 600; i++) for (let j = 0; j <= 600; j++) {
      const x = (2 * i) / 600, u = -2 + (4 * j) / 600;
      if (rows.every(([a, b, c]) => a * x + b * u <= c + 1e-12)) { lo = Math.min(lo, x); hi = Math.max(hi, x); }
    }
    if (lo === Infinity) { assert.equal(res, null); continue; }
    assert.ok(Math.abs(res[0] - lo) < 0.01 && Math.abs(res[1] - hi) < 0.01, JSON.stringify({ res, lo, hi }));
  }
});

test('natural cubic spline interpolates and differentiates a smooth function', () => {
  const xs = Array.from({ length: 101 }, (_, i) => (i * Math.PI) / 100);
  const f = cubicSpline(xs, xs.map(Math.sin));
  for (const x of [0.3, 1.1, 2.0, 2.9]) {
    const [v, d1, d2] = f(x);
    assert.ok(Math.abs(v - Math.sin(x)) < 1e-6 && Math.abs(d1 - Math.cos(x)) < 1e-3 && Math.abs(d2 + Math.sin(x)) < 2e-2);
  }
});

test('path is parameterized by TCP arc length (|dp/ds| = 1) and keeps the tool pointing down', () => {
  const paths = getPaths(robot, 'kitting');
  for (const p of paths) {
    for (let s = 0.02; s < p.length - 0.02; s += 0.05) {
      const a = K.fk(robot, p.eval(s).q), b = K.fk(robot, p.eval(s + 1e-4).q);
      const sp = Math.hypot(...K.v3.sub(b.ptcp, a.ptcp)) / 1e-4;
      assert.ok(Math.abs(sp - 1) < 0.02, `speed ${sp} at s=${s}`);
      assert.ok(K.m3.col(a.Rf, 2)[2] < -0.999);
    }
  }
});

test('plan (randomized operator positions): contact-safe trajectories pass the 4x verifier with zero violations', () => {
  const r = rng(99);
  for (let t = 0; t < 4; t++) {
    const sc = { ...DEFAULT_SCENARIO, opX: 1.3 + 0.4 * r(), opY: -0.3 + 0.6 * r(), payload: Math.round(10 * r()) };
    const out = runScenario(sc);
    assert.ok(out.strategies.contactSafe.ok, JSON.stringify(sc));
    assert.equal(out.strategies.contactSafe.violations, 0, JSON.stringify(sc));
    assert.ok(out.strategies.contactSafe.maxTorqueRatio <= 1);
  }
});

test('verifier catches unsafe plans: full speed exceeds the ISO/TS 15066 force limit near the operator', () => {
  const out = runScenario(DEFAULT_SCENARIO);
  assert.ok(out.strategies.fullSpeed.violations > 0);
  assert.ok(out.strategies.fullSpeed.maxForceRatio > 1);
});

test('ordering: full speed <= contact-safe <= single limit <= cobot speed (constraint sets are nested)', () => {
  for (const task of ['kitting', 'handover']) {
    const h = runScenario({ ...DEFAULT_SCENARIO, task }).headline;
    assert.ok(h.fullSpeed <= h.contactSafe + 1e-9 && h.contactSafe <= h.singleLimit + 1e-9 && h.singleLimit <= h.cobot250 + 1e-9, JSON.stringify(h));
  }
});

test('cobot-speed baseline never moves the TCP faster than 250 mm/s', () => {
  const out = runScenario(DEFAULT_SCENARIO);
  const mx = Math.max(...out.series.cobot250.rows.map((r) => r[3]));
  assert.ok(mx <= 0.25 + 1e-9, String(mx));
});

test('headline numbers in data/results.json are reproduced exactly by the code', () => {
  const saved = JSON.parse(fs.readFileSync(new URL('../data/results.json', import.meta.url)));
  const fresh = runScenario(DEFAULT_SCENARIO);
  const s = saved.scenarios[scenarioKey(DEFAULT_SCENARIO)];
  assert.deepEqual(fresh.headline, s.headline);
  assert.equal(s.headline.violations, 0);
  assert.ok(s.headline.speedupVsCobot > 2.5);
});
