// Decimate NEURA's published MAiRA 7M collision meshes (neura-rtb-data 0.0.1, meshes/Link*_body.stl)
// by vertex clustering so the digital twin loads fast. Usage:
//   node scripts/meshes.mjs <dir-with-Link*_body.stl> [cell-size-m]
import fs from 'node:fs';
import path from 'node:path';

const dir = process.argv[2];
const cell = Number(process.argv[3] || 0.009);
const out = { source: 'neura-rtb-data 0.0.1 (PyPI) neura_description/meshes/Link{0..7}_body.stl, vertex-clustered at ' + cell * 1000 + ' mm', links: [] };
for (let L = 0; L <= 7; L++) {
  const b = fs.readFileSync(path.join(dir, `Link${L}_body.stl`));
  const n = b.readUInt32LE(80);
  const key = new Map(), sums = [];
  const idx = [];
  const vid = (x, y, z) => {
    const k = `${Math.round(x / cell)},${Math.round(y / cell)},${Math.round(z / cell)}`;
    let id = key.get(k);
    if (id === undefined) { id = sums.length; key.set(k, id); sums.push([0, 0, 0, 0]); }
    const s = sums[id]; s[0] += x; s[1] += y; s[2] += z; s[3]++;
    return id;
  };
  const seen = new Set();
  for (let i = 0; i < n; i++) {
    const o = 84 + i * 50 + 12;
    const t = [0, 1, 2].map((v) => vid(b.readFloatLE(o + v * 12), b.readFloatLE(o + v * 12 + 4), b.readFloatLE(o + v * 12 + 8)));
    if (t[0] === t[1] || t[1] === t[2] || t[0] === t[2]) continue;
    const k = [...t].sort((a, c) => a - c).join(',');
    if (seen.has(k)) continue;
    seen.add(k);
    idx.push(...t);
  }
  const pos = new Float32Array(sums.length * 3);
  sums.forEach((s, i) => { pos[i * 3] = s[0] / s[3]; pos[i * 3 + 1] = s[1] / s[3]; pos[i * 3 + 2] = s[2] / s[3]; });
  const ind = new Uint32Array(idx);
  out.links.push({ link: L, tris: idx.length / 3, positions: Buffer.from(pos.buffer).toString('base64'), indices: Buffer.from(ind.buffer).toString('base64') });
  console.log(`Link${L}: ${n} -> ${idx.length / 3} triangles, ${sums.length} vertices`);
}
fs.writeFileSync(new URL('../data/maira7m-mesh.json', import.meta.url), JSON.stringify(out));
console.log('wrote data/maira7m-mesh.json', fs.statSync(new URL('../data/maira7m-mesh.json', import.meta.url)).size, 'bytes');
