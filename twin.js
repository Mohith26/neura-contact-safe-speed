// Three.js digital twin: MAiRA 7M (NEURA's published collision meshes), bench, operator, body-region zones,
// the planned TCP path coloured by contact type, and playback of the time-parameterized trajectory.
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { fk } from './src/kin.js';
import { BENCH } from './src/cell.js';

const COLORS = { free: 0x6687f4, transient: 0xf2a93b, 'quasi-static': 0xe57183 };

function b64(s, Type) { const bin = atob(s); const u = new Uint8Array(bin.length); for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return new Type(u.buffer); }
const setMat = (obj, R, P) => {
  obj.matrix.set(R[0], R[1], R[2], P[0], R[3], R[4], R[5], P[1], R[6], R[7], R[8], P[2], 0, 0, 0, 1);
  obj.matrixWorldNeedsUpdate = true;
};

export class Twin {
  constructor(el, meshData, robot) {
    this.el = el; this.robot = robot;
    const r = (this.renderer = new THREE.WebGLRenderer({ antialias: true, preserveDrawingBuffer: true }));
    r.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
    r.outputColorSpace = THREE.SRGBColorSpace;
    el.appendChild(r.domElement);
    const scene = (this.scene = new THREE.Scene());
    scene.background = new THREE.Color(0x24282e);
    const cam = (this.cam = new THREE.PerspectiveCamera(38, 1, 0.05, 50));
    cam.up.set(0, 0, 1);
    cam.position.set(2.25, -2.35, 1.7);
    this.controls = new OrbitControls(cam, r.domElement);
    this.controls.target.set(0.55, 0.0, 0.38);
    this.controls.enableDamping = true;
    this.controls.update();
    scene.add(new THREE.HemisphereLight(0xdfe6f2, 0x1a1d22, 1.15));
    const sun = new THREE.DirectionalLight(0xffffff, 1.5); sun.position.set(1.5, -2, 4); scene.add(sun);
    const fill = new THREE.DirectionalLight(0x8fa6ff, 0.45); fill.position.set(-2, 2, 1); scene.add(fill);
    // floor grid and bench
    const grid = new THREE.GridHelper(8, 32, 0x3a3f47, 0x30343a); grid.rotation.x = Math.PI / 2; grid.position.z = BENCH.min[2]; scene.add(grid);
    const bw = BENCH.max[0] - BENCH.min[0], bd = BENCH.max[1] - BENCH.min[1], bh = BENCH.max[2] - BENCH.min[2];
    const bench = new THREE.Mesh(new THREE.BoxGeometry(bw, bd, bh), new THREE.MeshStandardMaterial({ color: 0x3b4048, roughness: 0.9 }));
    bench.position.set((BENCH.max[0] + BENCH.min[0]) / 2, (BENCH.max[1] + BENCH.min[1]) / 2, (BENCH.max[2] + BENCH.min[2]) / 2); scene.add(bench);
    const top = new THREE.Mesh(new THREE.BoxGeometry(bw, bd, 0.012), new THREE.MeshStandardMaterial({ color: 0x4a505a, roughness: 0.7 }));
    top.position.set(bench.position.x, bench.position.y, -0.006); scene.add(top);
    // trays + fixture props
    const prop = (x, y, w, d, h, c) => { const m = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), new THREE.MeshStandardMaterial({ color: c, roughness: 0.8 })); m.position.set(x, y, h / 2); scene.add(m); };
    prop(0.15, -0.62, 0.30, 0.22, 0.03, 0x2f5a8a); prop(0.18, 0.62, 0.30, 0.22, 0.03, 0x2f5a8a); prop(0.80, 0.0, 0.16, 0.16, 0.05, 0x6b7280);
    // robot links
    const mat = new THREE.MeshStandardMaterial({ color: 0xe4e7ec, roughness: 0.42, metalness: 0.08 });
    const dark = new THREE.MeshStandardMaterial({ color: 0x2b2f36, roughness: 0.5, metalness: 0.2 });
    this.links = meshData.links.map((L) => {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(b64(L.positions, Float32Array), 3));
      g.setIndex(new THREE.BufferAttribute(b64(L.indices, Uint32Array), 1));
      g.computeVertexNormals();
      const m = new THREE.Mesh(g, L.link === 0 || L.link === 2 || L.link === 4 || L.link === 6 ? dark : mat);
      m.matrixAutoUpdate = false; scene.add(m); return m;
    });
    // gripper (flange frame, tool along +z)
    this.gripper = new THREE.Group(); this.gripper.matrixAutoUpdate = false;
    const body = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.045, 0.07, 24), dark); body.rotation.x = Math.PI / 2; body.position.z = 0.035; this.gripper.add(body);
    for (const sx of [-1, 1]) { const f = new THREE.Mesh(new THREE.BoxGeometry(0.014, 0.03, 0.09), mat); f.position.set(sx * 0.022, 0, 0.115); this.gripper.add(f); }
    scene.add(this.gripper);
    this.tcp = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), new THREE.MeshBasicMaterial({ color: 0xffffff }));
    scene.add(this.tcp);
    this.zoneGroup = new THREE.Group(); scene.add(this.zoneGroup);
    this.pathGroup = new THREE.Group(); scene.add(this.pathGroup);
    this.resize();
    new ResizeObserver(() => this.resize()).observe(el);
    const loop = () => { this.controls.update(); this.renderer.render(scene, cam); requestAnimationFrame(loop); };
    loop();
  }
  resize() {
    const w = this.el.clientWidth || 800, h = this.el.clientHeight || 450;
    this.renderer.setSize(w, h, false); this.cam.aspect = w / h; this.cam.updateProjectionMatrix();
  }
  setRobot(robot) { this.robot = robot; }
  pose(q) {
    const F = fk(this.robot, q);
    this.links.forEach((m, i) => setMat(m, F.R[i], F.P[i]));
    setMat(this.gripper, F.Rf, F.pf);
    this.tcp.position.set(F.ptcp[0], F.ptcp[1], F.ptcp[2]);
    return F;
  }
  setZones(zones, opX, opY) {
    const g = this.zoneGroup; while (g.children.length) g.remove(g.children[0]);
    // operator mannequin (neutral), body boxes tinted
    const skin = new THREE.MeshStandardMaterial({ color: 0x7c8592, roughness: 0.8, transparent: true, opacity: 0.9 });
    const limb = (a, b, r) => { const A = new THREE.Vector3(...a), B = new THREE.Vector3(...b); const L = A.distanceTo(B); const m = new THREE.Mesh(new THREE.CapsuleGeometry(r, L, 6, 12), skin); m.position.copy(A.clone().add(B).multiplyScalar(0.5)); m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), B.clone().sub(A).normalize()); g.add(m); };
    for (const sy of [-1, 1]) {
      const sh = [opX - 0.02, opY + sy * 0.21, 0.47], el = [opX - 0.12, opY + sy * 0.25, 0.20], wr = [opX - 0.42, opY + sy * 0.17, 0.06];
      limb(sh, el, 0.045); limb(el, wr, 0.037);
      const hand = new THREE.Mesh(new THREE.BoxGeometry(0.17, 0.08, 0.03), skin); hand.position.set(opX - 0.52, opY + sy * 0.14, 0.035); g.add(hand);
    }
    for (const z of zones) {
      if (z.kind === 'box') {
        const w = z.max[0] - z.min[0], d = z.max[1] - z.min[1], h = z.max[2] - z.min[2];
        const c = [(z.max[0] + z.min[0]) / 2, (z.max[1] + z.min[1]) / 2, (z.max[2] + z.min[2]) / 2];
        const isArm = z.region === 'hand' || z.region === 'lowerArm' || z.region === 'upperArm';
        if (!isArm) {
          let m;
          if (z.id === 'head') { m = new THREE.Mesh(new THREE.SphereGeometry(0.1, 24, 16), skin); m.scale.set(1, 0.9, 1.15); }
          else if (z.id === 'neck') { m = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.055, h, 16), skin); m.rotation.x = Math.PI / 2; }
          else { const rr = Math.min(w, d) * 0.45; m = new THREE.Mesh(new THREE.CapsuleGeometry(rr, Math.max(0.001, h - 2 * rr * 0.6), 6, 16), skin); m.rotation.x = Math.PI / 2; m.scale.set(w / (2 * rr) * 0.92, 1, d / (2 * rr) * 0.92); }
          m.position.set(c[0], c[1], c[2]); g.add(m);
        }
        const col = z.keepOut ? 0xe57183 : z.region === 'hand' ? 0xe57183 : 0xf2a93b;
        const edges = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(w, d, h)), new THREE.LineBasicMaterial({ color: col, transparent: true, opacity: z.region === 'hand' ? 0.85 : z.keepOut ? 0.5 : 0.1 }));
        edges.position.set(c[0], c[1], c[2]); g.add(edges);
        if (z.region === 'hand') { const f = new THREE.Mesh(new THREE.BoxGeometry(w, d, h), new THREE.MeshBasicMaterial({ color: 0xe57183, transparent: true, opacity: 0.12, depthWrite: false })); f.position.copy(edges.position); g.add(f); }
      } else if (z.kind === 'spheres') {
        for (const c of z.cs) {
          const rb = Math.sqrt(Math.max(0, z.R * z.R - c[2] * c[2])); // footprint on the bench top
          const disc = new THREE.Mesh(new THREE.CircleGeometry(rb, 96), new THREE.MeshBasicMaterial({ color: 0xf2a93b, transparent: true, opacity: 0.07, depthWrite: false }));
          disc.position.set(c[0], c[1], 0.003); g.add(disc);
          const ring = new THREE.Mesh(new THREE.RingGeometry(rb - 0.006, rb, 128), new THREE.MeshBasicMaterial({ color: 0xf2a93b, transparent: true, opacity: 0.6, side: THREE.DoubleSide }));
          ring.position.set(c[0], c[1], 0.004); g.add(ring);
        }
      }
    }
  }
  setPath(rows, qs) {
    const g = this.pathGroup; while (g.children.length) g.remove(g.children[0]);
    const pos = [], col = [];
    let prev = null;
    rows.forEach((r, i) => {
      const p = fk(this.robot, qs[i]).ptcp;
      const c = new THREE.Color(COLORS[r[6] || 'free']);
      if (prev && prev.m === r[1]) { pos.push(...prev.p, ...p); col.push(prev.c.r, prev.c.g, prev.c.b, c.r, c.g, c.b); }
      prev = { p, c, m: r[1] };
    });
    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    geo.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    g.add(new THREE.LineSegments(geo, new THREE.LineBasicMaterial({ vertexColors: true, linewidth: 2 })));
    // a thicker look: tube-like duplicates offset slightly
    const g2 = geo.clone(); const l2 = new THREE.LineSegments(g2, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.5 })); l2.position.z = 0.002; g.add(l2);
  }
  snapshot() { return this.renderer.domElement.toDataURL('image/png'); }
}
