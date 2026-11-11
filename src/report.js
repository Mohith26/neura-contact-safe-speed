// Turn a planTask() result into the plain JSON the app renders (and that data/results.json stores).
import { BODY, reducedMass, contactForce } from './iso15066.js';
import { CONTACT_POINTS } from './model.js';
import { buildRobot } from './kin.js';
import { MAIRA7M } from './model.js';
import { TASKS, Q_REST, OPERATOR_DEFAULT } from './cell.js';
import { buildTaskPaths } from './path.js';
import { planTask, STRATEGIES } from './plan.js';

const r4 = (x) => Math.round(x * 1e4) / 1e4;
const r3 = (x) => Math.round(x * 1e3) / 1e3;

export function scenarioKey(sc) {
  return `${sc.task}|${sc.payload}|${sc.opX}|${sc.opY}|${sc.massModel}|${sc.clamp ? 1 : 0}`;
}

export const DEFAULT_SCENARIO = { task: 'kitting', payload: 2, opX: OPERATOR_DEFAULT.x, opY: OPERATOR_DEFAULT.y, massModel: 'max', clamp: true };

const pathCache = new Map();
export function getPaths(robot, taskId) {
  const k = taskId + '|' + robot.payload;
  if (!pathCache.has(k)) pathCache.set(k, buildTaskPaths(robot, TASKS[taskId], Q_REST));
  return pathCache.get(k);
}

export function runScenario(sc = DEFAULT_SCENARIO) {
  const robot = buildRobot(MAIRA7M, { payload: sc.payload });
  const task = TASKS[sc.task];
  const paths = getPaths(robot, sc.task);
  const operator = { ...OPERATOR_DEFAULT, x: sc.opX, y: sc.opY };
  const opts = { massModel: sc.massModel };
  if (!sc.clamp) opts.clampGap = -1;
  const res = planTask(robot, paths, task, operator, opts);
  return serialize(res, paths, robot, task, sc);
}

export function serialize(res, paths, robot, task, sc) {
  const S = res.strategies;
  const out = {
    scenario: sc,
    task: { id: task.id, name: task.name, description: task.description, dwell: task.dwell, motions: paths.map((p) => ({ name: p.name, length: r4(p.length) })) },
    robot: { name: robot.model.name, payload: robot.payload, toolMass: robot.toolMass, movingMass: r3(robot.movingMass), isoMass: r3(robot.movingMass / 2 + robot.toolMass + robot.payload) },
    vSingle: Number.isFinite(res.vSingle) ? r4(res.vSingle) : null,
    strategies: {},
    zones: res.zones.map((z) => ({ id: z.id, kind: z.kind, region: z.region, regionLabel: BODY[z.region].label, alsoRegions: z.alsoRegions || [], label: z.label || BODY[z.region].label, keepOut: !!z.keepOut, min: z.min, max: z.max, cs: z.cs, R: z.R })),
    cart: paths.map((p) => p.cart.filter((_, i) => i % 4 === 0).map((pt) => pt.map(r4))),
    series: {},
    segments: [],
  };
  for (const [k, v] of Object.entries(S)) {
    out.strategies[k] = {
      label: STRATEGIES[k].label, short: STRATEGIES[k].short, ok: v.ok,
      ...(v.ok ? { cycleTime: r3(v.cycleTime), moveTime: r3(v.moveTime), violations: v.violations, maxForceRatio: r3(v.maxForceRatio), maxTorqueRatio: r3(v.maxTorqueRatio), motions: v.motions.map((m) => ({ name: m.name, duration: r3(m.duration), violations: m.violations, checked: m.checked, inZoneSamples: m.inZoneSamples, firstViolations: m.firstViolations.map((x) => ({ ...x, s: r4(x.s), v: x.v != null ? r4(x.v) : undefined, vlim: x.vlim != null ? r4(x.vlim) : undefined })) })) }
        : { reason: v.reason, s: v.s != null ? r3(v.s) : null, point: v.point || null, zone: v.zone || null, motion: v.motion ?? null }),
    };
  }
  // time series (with dwell between motions) for playback and the speed chart
  for (const k of Object.keys(S)) {
    const raw = res.raw[k];
    if (!raw || !S[k].ok) continue;
    const ser = [];
    let t0 = 0;
    raw.forEach((plan, mi) => {
      plan.grid.forEach((s, i) => {
        const st = plan.stages[i];
        const sd = Math.sqrt(plan.x[i]);
        // the TCP speed at which the most restrictive contact point reaches its limit
        let env = null, bind = null, contact = null;
        for (const pt of st.A.pts) {
          if (!pt.lim || pt.wn < 1e-9) continue;
          const e = pt.lim.vlim / pt.wn;
          if (env === null || e < env) { env = e; bind = pt.cp.id; contact = pt.lim.contact; }
        }
        if (k !== 'contactSafe') { env = null; bind = null; }
        const tcpW = st.A.pts[st.A.pts.length - 1].wn;
        ser.push([r4(t0 + plan.t[i]), mi, r4(s), r4(Math.floor(tcpW * sd * 1e4) / 1e4), env === null ? null : r4(env * tcpW), bind, contact]);
      });
      t0 += plan.duration + task.dwell;
    });
    // joint positions for playback, evaluated at the grid
    const qs = [];
    raw.forEach((plan, mi) => plan.grid.forEach((s) => qs.push(paths[mi].eval(s).q.map(r4))));
    out.series[k] = { cols: ['t', 'motion', 's', 'tcpSpeed', 'tcpEnvelope', 'binding', 'contact'], rows: ser, q: qs, duration: r3(t0) };
  }
  if (res.segments) {
    out.segments = res.segments.map((g, i) => {
      const pt = g.best;
      const base = { id: i + 1, motion: g.motion, motionName: g.motionName, s0: r3(g.s0), s1: r3(g.s1), t0: r3(g.t0), t1: r3(g.t1), tcpMax: r3(g.tcpMax), samples: g.n, points: g.points, worstS: g.worst ? r3(g.worst.s) : null };
      if (!pt) return { ...base, free: true };
      const L = pt.lim, b = BODY[L.region];
      const mu = reducedMass(b.mH, L.mR);
      return {
        ...base, free: false,
        point: pt.cp.id, pointLabel: CONTACT_POINTS.find((c) => c.id === pt.cp.id).label,
        zone: L.zone.id, zoneLabel: L.zone.label || BODY[L.zone.region].label,
        region: L.region, regionLabel: b.label, contact: L.contact, area: L.area,
        Fqs: b.F, p: b.p, k: b.k, mH: b.mH, mult: L.contact === 'transient' ? b.transient : 1,
        Flim: r3(L.Flim), mR: r3(L.mR), mIso: L.mIso != null ? r3(L.mIso) : null, mRefl: L.mRefl != null && Number.isFinite(L.mRefl) ? r3(L.mRefl) : null,
        mu: r4(mu), vlim: r4(L.vlim), vPlanned: r4(g.vPointMax), Fpred: r3(contactForce(L.region, L.mR, g.vPointMax)),
        margin: r3(1 - g.vPointMax / L.vlim), bindingShare: r3(g.bindingFrac / g.n),
      };
    });
  }
  const cs = out.strategies.contactSafe, cb = out.strategies.cobot250, sl = out.strategies.singleLimit, fs = out.strategies.fullSpeed;
  out.headline = cs.ok ? {
    contactSafe: cs.cycleTime, cobot250: cb.ok ? cb.cycleTime : null, singleLimit: sl && sl.ok ? sl.cycleTime : null, fullSpeed: fs.ok ? fs.cycleTime : null,
    speedupVsCobot: cb.ok ? r3(cb.cycleTime / cs.cycleTime) : null,
    savedVsCobot: cb.ok ? r3(cb.cycleTime - cs.cycleTime) : null,
    savedVsSingle: sl && sl.ok ? r3(sl.cycleTime - cs.cycleTime) : null,
    violations: cs.violations, fullSpeedViolations: fs.ok ? fs.violations : null, fullSpeedMaxForceRatio: fs.ok ? fs.maxForceRatio : null,
    cyclesPerHour: r3(3600 / cs.cycleTime), cyclesPerHourCobot: cb.ok ? r3(3600 / cb.cycleTime) : null,
  } : null;
  return out;
}
