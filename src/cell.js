// Cell layout: MAiRA on a workbench (bench top = z 0, floor = z -0.9), an operator standing
// across the bench, the body-region zones derived from the operator's position, and the tasks.
// All geometry in metres, world frame at the robot base.

export const BENCH = { min: [-0.45, -0.95, -0.9], max: [1.15, 0.95, 0.0], topZ: 0.0 };

// Operator anthropometry (approx. 50th percentile male, standing; heights relative to bench top 0.9 m above floor).
export const OPERATOR_DEFAULT = { x: 1.40, y: 0.0, shoulderZ: 0.50, shoulderHalfWidth: 0.20, reach: 0.72, reachMargin: 0.10 };

// Body-region zones (boxes) for an operator standing at (x, y), facing the robot (-x direction).
export function bodyZones(op = OPERATOR_DEFAULT) {
  const { x, y } = op;
  const box = (id, region, min, max, extra = {}) => ({ id, region, kind: 'box', min, max, ...extra });
  return [
    box('head', 'face', [x - 0.11, y - 0.10, 0.60], [x + 0.11, y + 0.10, 0.85], { keepOut: true, label: 'Head (keep-out)' }),
    box('neck', 'neck', [x - 0.07, y - 0.07, 0.50], [x + 0.07, y + 0.07, 0.60]),
    box('chest', 'chest', [x - 0.14, y - 0.18, 0.25], [x + 0.14, y + 0.18, 0.50]),
    box('abdomen', 'abdomen', [x - 0.14, y - 0.17, 0.10], [x + 0.14, y + 0.17, 0.25]),
    box('pelvis', 'pelvis', [x - 0.13, y - 0.18, 0.00], [x + 0.13, y + 0.18, 0.10]),
    box('upperArmL', 'upperArm', [x - 0.22, y + 0.18, 0.12], [x + 0.08, y + 0.30, 0.48]),
    box('upperArmR', 'upperArm', [x - 0.22, y - 0.30, 0.12], [x + 0.08, y - 0.18, 0.48]),
    box('lowerArms', 'lowerArm', [x - 0.52, y - 0.34, 0.03], [x - 0.12, y + 0.34, 0.30]),
    box('hands', 'hand', [x - 0.66, y - 0.32, 0.0], [x - 0.30, y + 0.32, 0.09], { clampSurfaceZ: BENCH.topZ }),
    box('thighs', 'thigh', [x - 0.16, y - 0.20, -0.42], [x + 0.14, y + 0.20, 0.0]),
    box('lowerLegs', 'lowerLeg', [x - 0.14, y - 0.20, -0.90], [x + 0.12, y + 0.20, -0.42]),
  ];
}

// Reach envelope: anything within arm's reach (+ margin for leaning, ISO 13857 style) of either
// shoulder can be touched by a hand or forearm, so it carries the stricter of the two limits.
export function reachZones(op = OPERATOR_DEFAULT) {
  const R = op.reach + op.reachMargin;
  const sh = (sy) => [op.x, op.y + sy * op.shoulderHalfWidth, op.shoulderZ];
  return [{
    id: 'reach', kind: 'spheres', region: 'hand', alsoRegions: ['lowerArm'],
    cs: [sh(1), sh(-1)], R, clampSurfaceZ: BENCH.topZ,
    label: 'Operator reach envelope',
  }];
}

export function allZones(op = OPERATOR_DEFAULT) {
  return [...bodyZones(op), ...reachZones(op)];
}

// Distance-based membership: point p with housing radius r touches zone z if the
// (inflated) zone contains p.
export function inZone(z, p, r) {
  if (z.kind === 'box') {
    for (let k = 0; k < 3; k++) if (p[k] < z.min[k] - r || p[k] > z.max[k] + r) return false;
    return true;
  }
  return z.cs.some((c) => Math.hypot(p[0] - c[0], p[1] - c[1], p[2] - c[2]) <= z.R + r);
}

// Tasks: Cartesian TCP waypoints (tool pointing down). Each motion runs between two stops
// (gripper actions); corners inside a motion are blended with radius `blend`.
export const TASKS = {
  kitting: {
    id: 'kitting',
    name: 'Gearbox kitting',
    description: 'Pick bearing from infeed tray, place into the shared fixture where the operator presses it in, pick the finished subassembly from the outfeed.',
    dwell: 0.4,
    blend: 0.06,
    motions: [
      { name: 'Infeed to fixture', pts: [[0.15, -0.62, 0.06], [0.15, -0.62, 0.30], [0.80, 0.0, 0.32], [0.80, 0.0, 0.08]] },
      { name: 'Fixture to outfeed', pts: [[0.80, 0.0, 0.08], [0.80, 0.0, 0.32], [0.18, 0.62, 0.30], [0.18, 0.62, 0.06]] },
      { name: 'Outfeed to infeed', pts: [[0.18, 0.62, 0.06], [0.18, 0.62, 0.30], [0.55, 0.0, 0.40], [0.15, -0.62, 0.30], [0.15, -0.62, 0.06]] },
    ],
  },
  handover: {
    id: 'handover',
    name: 'Hand-over to operator',
    description: 'Pick a housing from the infeed tray and hand it to the operator at chest height, then return.',
    dwell: 0.6,
    blend: 0.06,
    motions: [
      { name: 'Infeed to hand-over', pts: [[0.15, -0.62, 0.06], [0.15, -0.62, 0.32], [0.55, -0.15, 0.40], [0.92, 0.0, 0.36]] },
      { name: 'Hand-over to infeed', pts: [[0.92, 0.0, 0.36], [0.55, -0.15, 0.40], [0.15, -0.62, 0.32], [0.15, -0.62, 0.06]] },
    ],
  },
};

export const Q_REST = [0, 0.55, 0, 1.75, 0, 0.85, 0];
