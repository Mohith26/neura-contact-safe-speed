// Precompute the default scenarios shown on first load and the headline numbers quoted in the README.
//   node scripts/build-results.mjs
import fs from 'node:fs';
import { runScenario, DEFAULT_SCENARIO, scenarioKey } from '../src/report.js';

const scenarios = [DEFAULT_SCENARIO, { ...DEFAULT_SCENARIO, task: 'handover' }];
const out = { generatedBy: 'scripts/build-results.mjs', model: 'MAiRA 7M nominal (neura-rtb-data 0.0.1)', scenarios: {} };
for (const sc of scenarios) {
  const t0 = Date.now();
  const r = runScenario(sc);
  out.scenarios[scenarioKey(sc)] = r;
  const h = r.headline;
  console.log(sc.task, `${Date.now() - t0} ms`, h ? `contact-safe ${h.contactSafe}s | cobot ${h.cobot250}s | single ${h.singleLimit}s | full ${h.fullSpeed}s (${h.fullSpeedViolations} violations, ${h.fullSpeedMaxForceRatio}x force) | contact-safe violations ${h.violations}` : 'infeasible');
}
out.defaultKey = scenarioKey(DEFAULT_SCENARIO);
fs.writeFileSync(new URL('../data/results.json', import.meta.url), JSON.stringify(out));
console.log('wrote data/results.json', fs.statSync(new URL('../data/results.json', import.meta.url)).size, 'bytes');
