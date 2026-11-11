import { icon } from './src/icons.js';
import { scenarioKey, DEFAULT_SCENARIO } from './src/report.js';
import { buildRobot } from './src/kin.js';
import { MAIRA7M } from './src/model.js';

const $ = (s, el = document) => el.querySelector(s);
const $$ = (s, el = document) => [...el.querySelectorAll(s)];
const fmt = (x, d = 2) => (x == null || !Number.isFinite(x) ? '–' : x.toFixed(d));
const STRATS = ['contactSafe', 'singleLimit', 'cobot250', 'fullSpeed'];
const LS = 'neuron-suite.contact-safe-speed.v1';

// ---------- icons ----------
$$('[data-ic]').forEach((el) => { el.insertAdjacentHTML('afterbegin', icon(el.dataset.ic, Number(el.dataset.s || 20))); });

// ---------- state: URL hash > localStorage > defaults ----------
const saved = (() => { try { return JSON.parse(localStorage.getItem(LS)) || {}; } catch { return {}; } })();
const state = {
  sc: { ...DEFAULT_SCENARIO, ...(saved.sc || {}) },
  strat: 'contactSafe', t: 0, playing: false, seg: null, filter: 'all',
  applied: saved.applied || {}, configs: saved.configs || [],
  result: null, pre: null, mesh: null, reqId: 0,
};
function readHash() {
  const h = new URLSearchParams(location.hash.replace(/^#\/?speed\??/, '').replace(/^#/, ''));
  const num = (k, d) => (h.has(k) && Number.isFinite(Number(h.get(k))) ? Number(h.get(k)) : d);
  if (h.has('task') && ['kitting', 'handover'].includes(h.get('task'))) state.sc.task = h.get('task');
  state.sc.opX = num('opx', state.sc.opX); state.sc.opY = num('opy', state.sc.opY); state.sc.payload = num('payload', state.sc.payload);
  if (['max', 'iso', 'reflected'].includes(h.get('mass'))) state.sc.massModel = h.get('mass');
  if (h.has('clamp')) state.sc.clamp = h.get('clamp') !== '0';
  if (STRATS.includes(h.get('strat'))) state.strat = h.get('strat');
  if (h.has('seg')) state.seg = num('seg', null);
  if (h.has('t')) state.t = num('t', 0);
}
function writeHash() {
  const p = new URLSearchParams({ task: state.sc.task, opx: state.sc.opX.toFixed(2), opy: state.sc.opY.toFixed(2), payload: String(state.sc.payload), mass: state.sc.massModel, clamp: state.sc.clamp ? '1' : '0', strat: state.strat });
  if (state.seg) p.set('seg', state.seg);
  history.replaceState(null, '', '#/speed?' + p.toString());
  localStorage.setItem(LS, JSON.stringify({ sc: state.sc, applied: state.applied, configs: state.configs }));
}
readHash();

// ---------- worker ----------
let worker = null;
try { worker = new Worker('./worker.js', { type: 'module' }); } catch { worker = null; }
const pending = new Map();
if (worker) worker.onmessage = (e) => { const cb = pending.get(e.data.id); if (cb) { pending.delete(e.data.id); cb(e.data); } };
function plan(sc) {
  const key = scenarioKey(sc);
  if (state.pre && state.pre.scenarios[key]) return Promise.resolve({ ok: true, result: state.pre.scenarios[key], ms: 0 });
  if (!worker) return import('./src/report.js').then((m) => ({ ok: true, result: m.runScenario(sc) }));
  const id = ++state.reqId;
  return new Promise((res) => { pending.set(id, res); worker.postMessage({ id, scenario: sc }); });
}

// ---------- UI helpers ----------
function toast(msg, ok = true) {
  const t = $('#toast'); t.innerHTML = icon(ok ? 'tick' : 'warning', 16) + `<span>${msg}</span>`; t.classList.add('show');
  clearTimeout(toast.h); toast.h = setTimeout(() => t.classList.remove('show'), 2200);
}
function busy(on) { ['#busyK', '#busyW', '#busyC'].forEach((s, i) => { const el = $(s); el.innerHTML = i === 0 ? '<div><div class="spin"></div>Re-planning…</div>' : ''; el.classList.toggle('show', on); }); }
function download(name, text, type) { const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([text], { type })); a.download = name; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 1000); }

// ---------- controls ----------
function syncControls() {
  $('#task').value = state.sc.task;
  $('#opx').value = state.sc.opX; $('#opxO').textContent = state.sc.opX.toFixed(2) + ' m';
  $('#opy').value = state.sc.opY; $('#opyO').textContent = (state.sc.opY >= 0 ? '+' : '') + state.sc.opY.toFixed(2) + ' m';
  $('#payload').value = state.sc.payload; $('#payloadO').textContent = state.sc.payload + ' kg';
  $$('#massSeg button').forEach((b) => { b.classList.toggle('on', b.dataset.v === state.sc.massModel); b.setAttribute('aria-checked', b.dataset.v === state.sc.massModel); b.setAttribute('role', 'radio'); });
  $('#clamp').classList.toggle('on', state.sc.clamp); $('#clamp').setAttribute('aria-checked', state.sc.clamp);
}
let deb = null;
function changed() { syncControls(); writeHash(); clearTimeout(deb); deb = setTimeout(recompute, 220); }
$('#task').onchange = (e) => { state.sc.task = e.target.value; state.seg = null; changed(); };
$('#opx').oninput = (e) => { state.sc.opX = Number(e.target.value); changed(); };
$('#opy').oninput = (e) => { state.sc.opY = Number(e.target.value); changed(); };
$('#payload').oninput = (e) => { state.sc.payload = Number(e.target.value); changed(); };
$$('#massSeg button').forEach((b) => (b.onclick = () => { state.sc.massModel = b.dataset.v; changed(); }));
$('#clamp').onclick = () => { state.sc.clamp = !state.sc.clamp; changed(); };
$$('#tfilters .chip').forEach((b) => (b.onclick = () => { state.filter = b.dataset.f; $$('#tfilters .chip').forEach((x) => x.classList.toggle('on', x === b)); renderTable(); }));

// ---------- rendering ----------
let twin = null, robotView = null;
function render() {
  const r = state.result;
  const cs = r.strategies.contactSafe;
  const task = r.task;
  $('#title').textContent = task.id === 'kitting' ? 'Gearbox kitting cell' : 'Hand-over station';
  $('#crumbTask').textContent = $('#title').textContent;
  document.title = `Contact-safe speed · ${$('#title').textContent} · Neuron Suite`;
  const banner = $('#banner');
  if (!cs.ok) {
    $('#bannerText').textContent = cs.reason === 'keep-out'
      ? `No contact-safe speed exists for this layout: the ${cs.point} enters the operator's ${cs.zone.toLowerCase()} ${fmt(cs.s)} m into "${task.motions[cs.motion].name}". ISO/TS 15066 does not permit transient contact with the head or face. Move the operator or re-route the path.`
      : `The planner could not find a feasible speed profile for "${task.motions[cs.motion]?.name || ''}" (${cs.reason}).`;
    banner.classList.add('show');
  } else banner.classList.remove('show');
  renderKPI(); renderChart(); renderTable(); renderTwin(); renderApplyState();
  if (state.seg) openSeg(state.seg, false);
}

function renderKPI() {
  const r = state.result, S = r.strategies, h = r.headline;
  $('#kBig').textContent = S.contactSafe.ok ? fmt(S.contactSafe.cycleTime, 2) : '–';
  $('#kDelta').innerHTML = h ? `${fmt(h.speedupVsCobot, 1)}× faster than cobot speed · ${fmt(h.savedVsSingle, 2)} s less than a single collaborative limit` : '<span style="color:var(--red)">No compliant plan for this layout</span>';
  const max = Math.max(...STRATS.map((k) => (S[k] && S[k].ok ? S[k].cycleTime : 0)));
  $('#bars').innerHTML = STRATS.map((k) => {
    const s = S[k]; if (!s) return '';
    const v = s.ok ? s.cycleTime : null;
    const bad = s.ok && s.violations > 0;
    return `<div class="bar ${k === 'contactSafe' ? 'cs' : ''}" data-k="${k}" title="${s.label}${bad ? ` · ${s.violations} contact-limit violations, peak ${fmt(s.maxForceRatio, 2)}× permissible force` : ''}">
      <span>${s.short}</span><div class="track"><div class="fill" style="width:${v ? (100 * v) / max : 0}%;${bad ? 'background:var(--red)' : ''}"></div></div>
      <span class="v ${bad ? 'bad' : ''}">${v ? fmt(v, 2) + ' s' : 'n/a'}</span></div>`;
  }).join('');
  $$('#bars .bar').forEach((b) => (b.onclick = () => setStrat(b.dataset.k)));
  const fv = $('#fViol');
  if (S.contactSafe.ok) {
    const checked = S.contactSafe.motions.reduce((a, m) => a + m.inZoneSamples, 0);
    fv.className = 'fact ' + (S.contactSafe.violations === 0 ? 'ok' : 'bad');
    fv.innerHTML = `<b>${S.contactSafe.violations}</b><span>contact-limit violations · ${checked.toLocaleString()} in-zone checks</span>`;
    $('#fRate').innerHTML = `<b>${Math.round(h.cyclesPerHour)}</b><span>cycles/h (cobot speed: ${Math.round(h.cyclesPerHourCobot)})</span>`;
  } else { fv.className = 'fact bad'; fv.innerHTML = '<b>–</b><span>no compliant plan</span>'; $('#fRate').innerHTML = '<b>–</b><span>cycles per hour</span>'; }
  const fs = S.fullSpeed;
  $('#kSub').textContent = fs && fs.ok ? `full speed: ${fs.violations} violations, ${fmt(fs.maxForceRatio, 2)}× force` : 'per kit, incl. gripper actions';
}

function setStrat(k) {
  if (!state.result.series[k]) return;
  state.strat = k; state.t = Math.min(state.t, state.result.series[k].duration);
  renderStratSeg(); renderChart(); renderTwin(); writeHash();
}
function renderStratSeg() {
  $('#stratSeg').innerHTML = STRATS.map((k, i) => `<button role="radio" aria-checked="${state.strat === k}" class="${state.strat === k ? 'on' : ''}" data-k="${k}" title="Key ${i + 1}" ${state.result.series[k] ? '' : 'disabled'}>${state.result.strategies[k]?.short || k}</button>`).join('');
  $$('#stratSeg button').forEach((b) => (b.onclick = () => setStrat(b.dataset.k)));
}

// ---------- chart ----------
function renderChart() {
  const svg = $('#chartSvg'), r = state.result, ser = r.series[state.strat];
  const W = svg.clientWidth || 1000, H = svg.clientHeight || 180;
  const pad = { l: 40, r: 10, t: 10, b: 22 };
  if (!ser) { svg.innerHTML = `<text x="${W / 2}" y="${H / 2}" fill="#7f8690" font-size="12" text-anchor="middle">No trajectory for this strategy</text>`; $('#cSub').textContent = ''; return; }
  const T = ser.duration;
  const vmax = Math.max(1.6, ...ser.rows.map((x) => x[3])) * 1.05;
  const X = (t) => pad.l + ((W - pad.l - pad.r) * t) / T, Y = (v) => H - pad.b - ((H - pad.t - pad.b) * Math.min(v, vmax)) / vmax;
  let g = '';
  // contact bands from the contact-safe series
  const csr = r.series.contactSafe ? r.series.contactSafe.rows : [];
  if (state.strat === 'contactSafe') {
    let st = null;
    const flush = (end) => { if (st && st.c && end > st.t) g += `<rect x="${X(st.t)}" y="${pad.t}" width="${Math.max(1, X(end) - X(st.t))}" height="${H - pad.t - pad.b}" fill="${st.c === 'quasi-static' ? 'rgba(229,113,131,.13)' : 'rgba(242,169,59,.08)'}"/>`; };
    let prevT = 0;
    csr.forEach((row) => { const c = row[6] || null; if (!st || st.c !== c || st.m !== row[1]) { if (st) flush(st.m !== row[1] ? prevT : row[0]); st = { t: row[0], c, m: row[1] }; } prevT = row[0]; });
    if (st) flush(csr[csr.length - 1][0]);
  }
  for (let v = 0; v <= vmax; v += 0.5) g += `<line x1="${pad.l}" x2="${W - pad.r}" y1="${Y(v)}" y2="${Y(v)}" stroke="#363a41" stroke-dasharray="${v ? '2 4' : ''}"/><text x="${pad.l - 8}" y="${Y(v) + 3.5}" fill="#7f8690" font-size="10" text-anchor="end">${v.toFixed(1)}</text>`;
  const step = T > 12 ? 2 : 1;
  for (let t = 0; t <= T + 1e-9; t += step) g += `<text x="${X(t)}" y="${H - 6}" fill="#7f8690" font-size="10" text-anchor="middle">${t.toFixed(0)} s</text>`;
  g += `<text x="${pad.l + 2}" y="${pad.t + 9}" fill="#7f8690" font-size="10">m/s</text>`;
  g += `<line x1="${pad.l}" x2="${W - pad.r}" y1="${Y(0.25)}" y2="${Y(0.25)}" stroke="#f2c12e" stroke-width="1.2" stroke-dasharray="5 4" opacity=".85"/>`;
  // path per motion (break at dwell)
  const poly = (rows, idx, color, extra = '') => {
    let d = '', last = -1;
    rows.forEach((row) => { const v = row[idx]; if (v == null) { last = -1; return; } d += (last === row[1] ? 'L' : 'M') + X(row[0]).toFixed(1) + ' ' + Y(v).toFixed(1); last = row[1]; });
    return `<path d="${d}" fill="none" stroke="${color}" ${extra}/>`;
  };
  if (state.strat === 'contactSafe') g += poly(ser.rows, 4, '#9aa1ab', 'stroke-width="1.2" stroke-dasharray="3 3"');
  g += poly(ser.rows, 3, state.strat === 'fullSpeed' ? '#e57183' : '#6d87eb', 'stroke-width="2"');
  g += `<line id="cursor" x1="${X(state.t)}" x2="${X(state.t)}" y1="${pad.t}" y2="${H - pad.b}" stroke="#e6e8eb" opacity=".5"/>`;
  g += `<rect id="hit" x="${pad.l}" y="${pad.t}" width="${W - pad.l - pad.r}" height="${H - pad.t - pad.b}" fill="transparent"/>`;
  svg.setAttribute('viewBox', `0 0 ${W} ${H}`);
  svg.innerHTML = g;
  const s = r.strategies[state.strat];
  $('#cSub').textContent = `${s.label} · ${fmt(ser.duration, 2)} s incl. ${r.task.motions.length} × ${r.task.dwell} s gripper actions`;
  const tip = $('#tip');
  $('#hit').onmousemove = (e) => {
    const bb = svg.getBoundingClientRect();
    const t = ((e.clientX - bb.left - pad.l) / (W - pad.l - pad.r)) * T;
    let best = ser.rows[0]; for (const row of ser.rows) if (Math.abs(row[0] - t) < Math.abs(best[0] - t)) best = row;
    tip.style.display = 'block'; tip.style.left = Math.min(e.clientX - bb.left + 14, W - 200) + 'px'; tip.style.top = '14px';
    const ctx = best[6] ? `${best[6] === 'quasi-static' ? 'Clamping' : 'Transient'} zone` : 'Outside operator reach';
    tip.innerHTML = `<b>${fmt(best[0], 2)} s</b> · ${r.task.motions[best[1]].name}<br>TCP speed <b>${fmt(best[3], 3)} m/s</b>` + (best[4] != null ? `<br>Contact limit at TCP <b>${fmt(best[4], 3)} m/s</b><br>Binding point: ${best[5]}` : '') + `<br>${ctx}`;
  };
  $('#hit').onmouseleave = () => (tip.style.display = 'none');
  $('#hit').onclick = (e) => { const bb = svg.getBoundingClientRect(); seek(((e.clientX - bb.left - pad.l) / (W - pad.l - pad.r)) * T); };
}

// ---------- table ----------
function segRows() {
  const segs = state.result.segments || [];
  return segs.filter((g) => state.filter === 'all' || (state.filter === 'free' ? g.free : g.contact === state.filter));
}
function renderTable() {
  const r = state.result, segs = segRows();
  $('#tSub').textContent = r.strategies.contactSafe.ok ? `${(r.segments || []).length} segments · binding ISO/TS 15066 limit per stretch of path` : '';
  if (!r.strategies.contactSafe.ok) { $('#tableWrap').innerHTML = '<div class="empty">No segments: the contact-safe plan is infeasible for this layout.</div>'; return; }
  if (!segs.length) { $('#tableWrap').innerHTML = `<div class="empty">No ${state.filter === 'quasi-static' ? 'clamping' : state.filter} segments in this plan.</div>`; return; }
  $('#tableWrap').innerHTML = `<table><thead><tr><th>#</th><th>Motion</th><th class="num">Time</th><th>Contact zone</th><th>Body region</th><th>Contact</th><th>Binding point</th><th class="num">m<sub>R</sub> kg</th><th class="num">Limit m/s</th><th class="num">Peak m/s</th><th class="num">Force / limit N</th><th>Margin</th></tr></thead><tbody>${segs.map((g) => g.free
    ? `<tr data-id="${g.id}" class="${state.seg === g.id ? 'sel' : ''}"><td>${g.id}</td><td>${g.motionName}</td><td class="num">${fmt(g.t0)}–${fmt(g.t1)}</td><td class="muted">Outside reach</td><td class="muted">–</td><td><span class="pill"><span class="d" style="background:#6687f4"></span>Robot-only</span></td><td class="muted">–</td><td class="num muted">–</td><td class="num muted">robot</td><td class="num">${fmt(g.tcpMax)}</td><td class="num muted">–</td><td class="muted">joint limits</td></tr>`
    : `<tr data-id="${g.id}" class="${state.seg === g.id ? 'sel' : ''}"><td>${g.id}</td><td>${g.motionName}</td><td class="num">${fmt(g.t0)}–${fmt(g.t1)}</td><td>${g.zoneLabel}</td><td>${g.regionLabel}</td><td><span class="pill"><span class="d" style="background:${g.contact === 'quasi-static' ? 'var(--red)' : 'var(--amber)'}"></span>${g.contact === 'quasi-static' ? 'Clamping' : 'Transient'}</span></td><td><span class="tag">${g.pointLabel}</span></td><td class="num">${fmt(g.mR, 1)}</td><td class="num">${fmt(g.vlim, 3)}</td><td class="num">${fmt(g.vPlanned, 3)}</td><td class="num">${Math.round(g.Fpred)} / ${Math.round(g.Flim)}</td><td><span class="marginbar"><i style="width:${Math.max(4, Math.min(100, g.margin * 100 * 2))}%"></i></span>${Math.round(g.margin * 100)}%</td></tr>`).join('')}</tbody></table>`;
  $$('#tableWrap tbody tr').forEach((tr) => (tr.onclick = () => openSeg(Number(tr.dataset.id))));
}

// ---------- drill-down ----------
function openSeg(id, focus = true) {
  const g = (state.result.segments || []).find((x) => x.id === id);
  if (!g) return;
  state.seg = id; writeHash();
  $$('#tableWrap tbody tr').forEach((tr) => tr.classList.toggle('sel', Number(tr.dataset.id) === id));
  const d = $('#drawer');
  const head = `<div class="dh"><div class="crumb">Segments <span>/</span> <b>#${g.id}</b></div><h3>${g.free ? 'Outside the operator\'s reach' : `${g.contact === 'quasi-static' ? 'Clamping' : 'Transient contact'} · ${g.regionLabel}`}</h3><div class="muted" style="margin-top:6px;font-size:12px">${g.motionName} · ${fmt(g.t0)}–${fmt(g.t1)} s · s = ${fmt(g.s0)}–${fmt(g.s1)} m</div></div>`;
  const body = g.free ? `<div class="db"><p class="muted" style="font-size:12.5px;line-height:1.6;margin:0">No body region of the operator can reach any of the seven watched points on the arm here (reach envelope ${fmt(0.82)} m from each shoulder, incl. 0.10 m for leaning). The planner leaves this stretch to MAiRA's own joint speed and torque limits.</p><div class="kv"><span>Peak TCP speed</span><b>${fmt(g.tcpMax, 3)} m/s</b><span>Duration</span><b>${fmt(g.t1 - g.t0, 2)} s</b></div></div>`
    : `<div class="db">
      <div class="sect">Binding contact</div>
      <div class="kv"><span>Point on the arm</span><b>${g.pointLabel}</b><span>Contact area (housing)</span><b>${g.area} cm²</b><span>Zone</span><b>${g.zoneLabel}</b><span>Body region (Table A.2)</span><b>${g.regionLabel}</b><span>Points that bound here</span><b>${Object.entries(g.points).map(([k, v]) => `${k} ×${v}`).join(', ')}</b></div>
      <div class="sect">Permissible force</div>
      <div class="formula">F<sub>qs</sub> = <em>${g.Fqs} N</em>, p<sub>qs</sub> = <em>${g.p} N/cm²</em> × ${g.area} cm²<br>F<sub>lim</sub> = ${g.mult} × min(${g.Fqs}, ${Math.round(g.p * g.area)}) = <em>${fmt(g.Flim, 0)} N</em> (${g.contact === 'quasi-static' ? 'quasi-static: no ×2 for clamping' : 'transient: ×2'})</div>
      <div class="sect">Effective masses</div>
      <div class="kv"><span>Robot, ISO A.3.2 (M/2 + m<sub>L</sub>)</span><b>${fmt(g.mIso, 2)} kg</b><span>Robot, reflected along motion</span><b>${g.mRefl != null ? fmt(g.mRefl, 2) + ' kg' : '–'}</b><span>Used (${state.sc.massModel === 'max' ? 'larger of the two' : state.sc.massModel === 'iso' ? 'ISO' : 'reflected'})</span><b>${fmt(g.mR, 2)} kg</b><span>Body region m<sub>H</sub>, k (Table A.3)</span><b>${g.mH} kg, ${g.k} N/mm</b></div>
      <div class="sect">Speed limit</div>
      <div class="formula">μ = (1/m<sub>H</sub> + 1/m<sub>R</sub>)⁻¹ = <em>${fmt(g.mu, 3)} kg</em><br>v<sub>lim</sub> = F<sub>lim</sub> / √(μ·k) = ${fmt(g.Flim, 0)} / √(${fmt(g.mu, 3)} × ${g.k * 1000}) = <em>${fmt(g.vlim, 3)} m/s</em><br>planned peak ${fmt(g.vPlanned, 3)} m/s → F = <em>${fmt(g.Fpred, 0)} N</em> (${Math.round(g.margin * 100)}% margin)</div>
      <p class="muted" style="font-size:11.5px;line-height:1.6;margin:0">Planned to 97% of the limit with zones inflated by 1 cm; the verifier then re-checks every point on exact zones at 4× the planning resolution.${g.mRefl != null && g.mRefl > g.mIso ? ` Here the model-based reflected mass (${fmt(g.mRefl, 1)} kg) is larger than ISO's simplified M/2 + m<sub>L</sub>, so the conservative setting uses it.` : ''}</p>
    </div>`;
  d.innerHTML = head + body + `<div class="df"><button class="btn" id="dClose">Close</button><button class="btn primary" id="dShow">Show in twin</button></div>`;
  d.classList.add('show'); d.setAttribute('aria-hidden', 'false'); $('#scrim').classList.add('show');
  $('#dClose').onclick = closeSeg; $('#dShow').onclick = () => { if (state.strat !== 'contactSafe') setStrat('contactSafe'); seek(g.t0 + segOffset(g)); closeSeg(); };
  if (focus) $('#dClose').focus();
}
function segOffset(g) { // global time offset of the motion within the series
  const rows = state.result.series.contactSafe.rows; const first = rows.find((r) => r[1] === g.motion); return first ? first[0] : 0;
}
function closeSeg() { $('#drawer').classList.remove('show'); $('#drawer').setAttribute('aria-hidden', 'true'); $('#scrim').classList.remove('show'); state.seg = null; writeHash(); $$('#tableWrap tbody tr').forEach((tr) => tr.classList.remove('sel')); }
$('#scrim').onclick = closeSeg;

// ---------- twin + playback ----------
function renderTwin() {
  if (!twin) return;
  const r = state.result;
  twin.setZones(r.zones, state.sc.opX, state.sc.opY);
  const ser = r.series[state.strat] || r.series.cobot250;
  if (ser) twin.setPath(ser.rows, ser.q);
  $('#twinSub').textContent = `${r.task.name} · operator at ${state.sc.opX.toFixed(2)} m · ${state.sc.payload} kg payload`;
  drawAt(state.t);
}
function drawAt(t) {
  const ser = state.result.series[state.strat] || state.result.series.cobot250;
  if (!ser) return;
  const rows = ser.rows; let i = 0;
  let lo = 0, hi = rows.length - 1; while (lo < hi) { const m = (lo + hi + 1) >> 1; if (rows[m][0] <= t) lo = m; else hi = m - 1; } i = lo;
  const j = Math.min(rows.length - 1, i + 1);
  let q = ser.q[i];
  if (j !== i && rows[j][1] === rows[i][1] && rows[j][0] > rows[i][0]) { const f = Math.min(1, Math.max(0, (t - rows[i][0]) / (rows[j][0] - rows[i][0]))); q = ser.q[i].map((v, k) => v + f * (ser.q[j][k] - v)); }
  twin && twin.pose(q);
  $('#scrub').value = Math.round((1000 * t) / ser.duration);
  $('#time').textContent = `${t.toFixed(2)} / ${ser.duration.toFixed(2)} s`;
  const c = $('#cursor'); if (c) { const svg = $('#chartSvg'); const W = svg.viewBox.baseVal.width || svg.clientWidth; const x = 40 + ((W - 50) * t) / ser.duration; c.setAttribute('x1', x); c.setAttribute('x2', x); }
}
function seek(t) { const ser = state.result.series[state.strat]; if (!ser) return; state.t = Math.max(0, Math.min(ser.duration, t)); drawAt(state.t); }
$('#scrub').oninput = (e) => { const ser = state.result.series[state.strat]; if (ser) seek((Number(e.target.value) / 1000) * ser.duration); };
let last = 0;
function tick(ts) {
  if (state.playing) { const dt = last ? (ts - last) / 1000 : 0; const ser = state.result.series[state.strat]; if (ser) { state.t += dt; if (state.t > ser.duration) state.t = 0; drawAt(state.t); } }
  last = ts; requestAnimationFrame(tick);
}
function setPlaying(p) { state.playing = p; $('#play').innerHTML = icon(p ? 'pause' : 'play', 16); $('#play').setAttribute('aria-label', p ? 'Pause (Space)' : 'Play (Space)'); }
$('#play').onclick = () => setPlaying(!state.playing);

// ---------- apply / save / export ----------
function renderApplyState() {
  const key = scenarioKey(state.sc), a = state.applied[key];
  const st = $('#status');
  st.className = 'status ' + (a ? 'applied' : '');
  $('#statusText').textContent = a ? 'Applied' : 'Draft';
  const ok = state.result.strategies.contactSafe.ok;
  $('#applyBtn').disabled = !ok || !!a;
  $('#applyBtn').textContent = a ? 'Applied to program' : 'Apply to program';
}
$('#applyBtn').onclick = () => openApply();
function openApply() {
  if ($('#applyBtn').disabled) return;
  const r = state.result, n = (r.segments || []).filter((g) => !g.free).length;
  $('#applyP').innerHTML = `Writes ${n} per-segment speed overrides into <b>${r.task.name} v3</b>: ${fmt(r.headline.contactSafe)} s per cycle instead of ${fmt(r.headline.cobot250)} s at cobot speed. The verifier found <b>${r.headline.violations}</b> contact-limit violations. Final validation still needs force/pressure measurements on the real cell (ISO/TS 15066 clause 5.5.5).`;
  const hist = Object.entries(state.applied).sort((a, b) => b[1].at - a[1].at).slice(0, 3);
  $('#history').innerHTML = hist.length ? '<div class="sect" style="margin-bottom:2px">Previously applied</div>' + hist.map(([k, v]) => `<div><span>${v.name} · ${v.desc}</span><span>${fmt(v.cycle)} s</span></div>`).join('') : '';
  $('#applyModal').classList.add('show'); $('#applyConfirm').focus();
}
$('#applyConfirm').onclick = () => {
  const key = scenarioKey(state.sc);
  state.applied[key] = { at: Date.now(), cycle: state.result.headline.contactSafe, name: state.result.task.name, desc: `op ${state.sc.opX.toFixed(2)} m, ${state.sc.payload} kg` };
  writeHash(); $('#applyModal').classList.remove('show'); renderApplyState(); toast('Speed overrides applied to the program');
};
$$('[data-close]').forEach((b) => (b.onclick = () => b.closest('.modal').classList.remove('show')));
$('#revertBtn').onclick = () => { state.configs.unshift({ ...state.sc, at: Date.now() }); state.configs = state.configs.slice(0, 10); writeHash(); toast('What-if configuration saved'); };
function exportCsv() {
  const r = state.result; if (!r.segments) return toast('Nothing to export for an infeasible plan', false);
  const head = ['segment', 'motion', 't0_s', 't1_s', 's0_m', 's1_m', 'zone', 'body_region', 'contact', 'binding_point', 'area_cm2', 'F_lim_N', 'm_R_kg', 'm_R_iso_kg', 'm_R_reflected_kg', 'mu_kg', 'k_N_per_mm', 'v_lim_m_s', 'v_planned_peak_m_s', 'F_predicted_N', 'margin'];
  const lines = [head.join(',')].concat(r.segments.map((g) => [g.id, `"${g.motionName}"`, g.t0, g.t1, g.s0, g.s1, g.free ? 'outside reach' : `"${g.zoneLabel}"`, g.free ? '' : `"${g.regionLabel}"`, g.free ? 'robot-only' : g.contact, g.point || '', g.area ?? '', g.Flim ?? '', g.mR ?? '', g.mIso ?? '', g.mRefl ?? '', g.mu ?? '', g.k ?? '', g.vlim ?? '', g.free ? g.tcpMax : g.vPlanned, g.Fpred ?? '', g.margin ?? ''].join(',')));
  lines.push('', 't_s,motion,s_m,tcp_speed_m_s,tcp_contact_limit_m_s,binding_point,contact');
  for (const row of r.series.contactSafe.rows) lines.push(row.map((x) => (x == null ? '' : x)).join(','));
  download(`contact-safe-speed-${r.task.id}.csv`, lines.join('\n'), 'text/csv'); toast('CSV exported');
}
function exportJson() { const r = { ...state.result }; download(`contact-safe-speed-${r.task.id}.json`, JSON.stringify(r, null, 1), 'application/json'); toast('JSON exported'); }
$('#exportCsv').onclick = exportCsv; $('#exportJson').onclick = exportJson;
$('#copyLink').onclick = async () => { try { await navigator.clipboard.writeText(location.href); toast('Deep link copied'); } catch { toast('Copy failed: ' + location.href, false); } };
$('#kbdBtn').onclick = () => $('#keysModal').classList.add('show');

// ---------- keyboard ----------
document.addEventListener('keydown', (e) => {
  if (e.target.matches('input, select, textarea') && e.key !== 'Escape') return;
  if (e.key === ' ') { e.preventDefault(); setPlaying(!state.playing); }
  else if (e.key === 'ArrowRight') seek(state.t + 0.1);
  else if (e.key === 'ArrowLeft') seek(state.t - 0.1);
  else if (['1', '2', '3', '4'].includes(e.key)) setStrat(STRATS[Number(e.key) - 1]);
  else if (e.key === ']' || e.key === '[') { const s = state.result.segments || []; if (!s.length) return; const cur = state.seg || 0; const nx = e.key === ']' ? Math.min(s.length, cur + 1) : Math.max(1, cur - 1); openSeg(nx); }
  else if (e.key.toLowerCase() === 'a') openApply();
  else if (e.key.toLowerCase() === 'e') exportCsv();
  else if (e.key.toLowerCase() === 'j') exportJson();
  else if (e.key === '?') $('#keysModal').classList.add('show');
  else if (e.key === 'Escape') { closeSeg(); $$('.modal').forEach((m) => m.classList.remove('show')); }
});

// ---------- boot ----------
async function recompute() {
  busy(true);
  const sc = { ...state.sc };
  const res = await plan(sc);
  if (scenarioKey(sc) !== scenarioKey(state.sc)) return; // superseded
  busy(false);
  if (!res.ok) { $('#bannerText').textContent = 'Planner error: ' + res.error; $('#banner').classList.add('show'); return; }
  state.result = res.result;
  robotView = buildRobot(MAIRA7M, { payload: sc.payload });
  twin && twin.setRobot(robotView);
  renderStratSeg(); render();
}
async function boot() {
  syncControls();
  ['#kBig'].forEach((s) => ($(s).innerHTML = '<span class="skel" style="display:inline-block;width:90px;height:32px"></span>'));
  busy(true);
  try {
    const [pre, mesh] = await Promise.all([fetch('data/results.json').then((r) => r.json()), fetch('data/maira7m-mesh.json').then((r) => r.json())]);
    state.pre = pre; state.mesh = mesh;
  } catch (err) {
    $('#bannerText').textContent = 'Could not load the robot model or precomputed plan (' + err.message + ').'; $('#banner').classList.add('show'); busy(false); return;
  }
  try {
    const { Twin } = await import('./twin.js');
    robotView = buildRobot(MAIRA7M, { payload: state.sc.payload });
    twin = new Twin($('#twin'), state.mesh, robotView);
  } catch (err) { $('#twin').innerHTML = `<div class="empty" style="padding-top:200px">3D view unavailable (${err.message}). Results below are unaffected.</div>`; }
  await recompute();
  writeHash();
  requestAnimationFrame(tick);
  window.addEventListener('resize', () => renderChart());
  window.__ready = true;
}
boot();
