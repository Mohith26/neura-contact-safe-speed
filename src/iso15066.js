// ISO/TS 15066:2016 Annex A body model (power and force limiting).
// Table A.2: maximum permissible quasi-static force F (N) and the lowest maximum permissible
// quasi-static pressure p (N/cm^2) among the specific areas listed for the region; transient
// multiplier (2, or null where transient contact is not permitted: skull/forehead, face).
// Table A.3: effective spring constant k (N/mm) and effective mass m_H (kg).
// Transcribed from ISO/TS 15066:2016(E) Annex A (copy used for teaching at diag.uniroma1.it/deluca/pHRI_elective)
// and cross-checked against arXiv:2311.13814 Table 1 and the COBOTTA manual excerpt of A.3.2.

export const BODY = {
  skull: { label: 'Skull and forehead', F: 130, p: 110, k: 150, mH: 4.4, transient: null },
  face: { label: 'Face', F: 65, p: 110, k: 75, mH: 4.4, transient: null },
  neck: { label: 'Neck', F: 150, p: 140, k: 50, mH: 1.2, transient: 2 },
  back: { label: 'Back and shoulders', F: 210, p: 160, k: 35, mH: 40, transient: 2 },
  chest: { label: 'Chest', F: 140, p: 120, k: 25, mH: 40, transient: 2 },
  abdomen: { label: 'Abdomen', F: 110, p: 140, k: 10, mH: 40, transient: 2 },
  pelvis: { label: 'Pelvis', F: 180, p: 210, k: 25, mH: 40, transient: 2 },
  upperArm: { label: 'Upper arms and elbow joints', F: 150, p: 190, k: 30, mH: 3, transient: 2 },
  lowerArm: { label: 'Lower arms and wrist joints', F: 160, p: 180, k: 40, mH: 2, transient: 2 },
  hand: { label: 'Hands and fingers', F: 140, p: 190, k: 75, mH: 0.6, transient: 2 },
  thigh: { label: 'Thighs and knees', F: 220, p: 220, k: 50, mH: 75, transient: 2 },
  lowerLeg: { label: 'Lower legs', F: 130, p: 210, k: 60, mH: 75, transient: 2 },
};

// Reduced mass of the two-body contact model (A.3.2): mu = (1/m_H + 1/m_R)^-1.
export const reducedMass = (mH, mR) => (Number.isFinite(mR) ? 1 / (1 / mH + 1 / mR) : mH);

// ISO/TS 15066 A.3.2 simplified robot effective mass: m_R = M/2 + m_L.
export const isoRobotMass = (movingMass, effectivePayload) => movingMass / 2 + effectivePayload;

// Permissible contact force (N) for a region, contact type and robot contact area (cm^2).
// Pressure limit converts to force through the area; the smaller one binds.
export function permissibleForce(region, contact, areaCm2) {
  const b = BODY[region];
  const mult = contact === 'transient' ? b.transient : 1;
  if (mult == null) return 0; // transient contact with head/face is not permitted
  return mult * Math.min(b.F, b.p * areaCm2);
}

// Maximum permissible relative speed (m/s) from the energy model E = F^2/(2k) = mu v^2 / 2
// => v = F / sqrt(mu k), with k converted from N/mm to N/m.
export function maxSpeed(region, contact, mR, areaCm2) {
  const b = BODY[region];
  const F = permissibleForce(region, contact, areaCm2);
  if (F <= 0) return 0;
  const mu = reducedMass(b.mH, mR);
  return F / Math.sqrt(mu * b.k * 1000);
}

// Peak contact force predicted by the same model for a contact at relative speed v (m/s).
export function contactForce(region, mR, v) {
  const b = BODY[region];
  return v * Math.sqrt(reducedMass(b.mH, mR) * b.k * 1000);
}
