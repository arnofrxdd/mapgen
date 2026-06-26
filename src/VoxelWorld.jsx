import { useEffect, useRef, useState } from "react";
import * as THREE from "three";

const TILE_SIZE       = 40;
const ROAD_WIDTH_MAIN = 22;
const ROAD_WIDTH_SIDE = 16;
const SIDEWALK_W      = 3.5;
const BUILDING_MIN_H  = 8;
const BUILDING_MAX_H  = 72;
const RENDER_RADIUS   = 7;
const LOAD_RADIUS     = 5;
const TREE_TRUNK_W    = 0.8;
const TREE_TRUNK_H    = 4.0;
const STREETLIGHT_H   = 7.8;
const ROAD_EVERY      = 4;

const hash = (x, z) => { const n = Math.sin(x * 127.1 + z * 311.7) * 43758.5453123; return n - Math.floor(n); };
const hash2 = (x, z, s) => { const n = Math.sin(x * 93.9898 + z * 67.233 + s * 17.1) * 43758.5453123; return n - Math.floor(n); };
const smoothNoise = (x, z) => {
  const ix = Math.floor(x), iz = Math.floor(z);
  const fx = x - ix, fz = z - iz;
  const ux = fx * fx * (3 - 2 * fx), uz = fz * fz * (3 - 2 * fz);
  const v1 = hash(ix, iz), v2 = hash(ix+1, iz), v3 = hash(ix, iz+1), v4 = hash(ix+1, iz+1);
  return v1 + (v2-v1)*ux + (v3-v1)*uz + (v4-v2-v3+v1)*ux*uz;
};
const fbm = (x, z) => {
  let v=0, amp=0.5, f=0.004;
  v += smoothNoise(x*f, z*f)*amp; f*=2; amp*=0.5;
  v += smoothNoise(x*f, z*f)*amp; f*=2; amp*=0.5;
  v += smoothNoise(x*f, z*f)*amp; f*=2; amp*=0.5;
  v += smoothNoise(x*f, z*f)*amp;
  return v;
};

const isRoadCol = (gx) => (((gx % ROAD_EVERY) + ROAD_EVERY) % ROAD_EVERY) === 0;
const isRoadRow = (gz) => (((gz % ROAD_EVERY) + ROAD_EVERY) % ROAD_EVERY) === 0;

const makeBox = (w, h, d, mat, x, y, z) => {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), mat);
  m.position.set(x, y, z);
  return m;
};

class MatCache {
  constructor() {
    this.asphalt    = new THREE.MeshLambertMaterial({ color: 0x1a1a1e });
    this.sidewalk   = new THREE.MeshLambertMaterial({ color: 0x3a3a42 });
    this.yellowLine = new THREE.MeshLambertMaterial({ color: 0xe5a93b });
    this.whiteLine  = new THREE.MeshLambertMaterial({ color: 0xcccccc });
    this.trunkMat   = new THREE.MeshLambertMaterial({ color: 0x5a3d28 });
    this.leafGreen  = new THREE.MeshLambertMaterial({ color: 0x2d5c1e });
    this.leafCherry = new THREE.MeshLambertMaterial({ color: 0xe07297 });
    this.leafAutumn = new THREE.MeshLambertMaterial({ color: 0xd47525 });
    this.poleMat    = new THREE.MeshLambertMaterial({ color: 0x22252c });
    this.bulbMat    = new THREE.MeshBasicMaterial({ color: 0xfffac8 });
    this.groundBase = new THREE.MeshLambertMaterial({ color: 0x111118 });
    this.roofMat    = new THREE.MeshLambertMaterial({ color: 0x0d0d14 });
    this.buildingMats = [
      new THREE.MeshLambertMaterial({ color: 0x23263a }),
      new THREE.MeshLambertMaterial({ color: 0x1d2229 }),
      new THREE.MeshLambertMaterial({ color: 0x2e2018 }),
      new THREE.MeshLambertMaterial({ color: 0x1e2b1e }),
      new THREE.MeshLambertMaterial({ color: 0x252030 }),
      new THREE.MeshLambertMaterial({ color: 0x1a2233 }),
    ];
    this.windowMats = [
      new THREE.MeshBasicMaterial({ color: 0xffcb66 }),
      new THREE.MeshBasicMaterial({ color: 0x3ac3ff }),
      new THREE.MeshBasicMaterial({ color: 0xff3380 }),
      new THREE.MeshBasicMaterial({ color: 0x66ff99 }),
      new THREE.MeshBasicMaterial({ color: 0xffaa33 }),
    ];
    this.neonColors = [0xff0055, 0x00ffcc, 0x9900ff, 0xffaa00, 0x00ff66, 0xff3300];
    this.neonMats   = this.neonColors.map(c => new THREE.MeshBasicMaterial({ color: c }));
  }
}

function buildChunk(gx, gz, mats) {
  const group  = new THREE.Group();
  const worldX = gx * TILE_SIZE;
  const worldZ = gz * TILE_SIZE;
  const roadCol = isRoadCol(gx);
  const roadRow = isRoadRow(gz);
  const intersection = roadCol && roadRow;
  const road = roadCol || roadRow;

  const ground = new THREE.Mesh(new THREE.PlaneGeometry(TILE_SIZE, TILE_SIZE), road ? mats.asphalt : mats.groundBase);
  ground.rotation.x = -Math.PI / 2;
  group.add(ground);

  if (road) {
    const rw = ROAD_WIDTH_SIDE;
    if (!intersection) {
      const swDir = roadCol;
      const swOffset = rw / 2 + SIDEWALK_W / 2;
      for (const sign of [-1, 1]) {
        const sw = new THREE.Mesh(new THREE.PlaneGeometry(swDir ? SIDEWALK_W : TILE_SIZE, swDir ? TILE_SIZE : SIDEWALK_W), mats.sidewalk);
        sw.rotation.x = -Math.PI / 2;
        sw.position.set(swDir ? sign*swOffset : 0, 0.03, swDir ? 0 : sign*swOffset);
        group.add(sw);
        group.add(makeBox(swDir ? SIDEWALK_W : TILE_SIZE, 0.18, swDir ? TILE_SIZE : SIDEWALK_W, mats.sidewalk, swDir ? sign*swOffset : 0, 0.09, swDir ? 0 : sign*swOffset));
      }
      const dashCount = 8;
      const dashLen = TILE_SIZE / (dashCount * 2);
      for (let i = 0; i < dashCount; i++) {
        const t = -TILE_SIZE/2 + dashLen*(i*2+0.5);
        const dash = new THREE.Mesh(new THREE.PlaneGeometry(roadCol ? 0.3 : dashLen*0.7, roadCol ? dashLen*0.7 : 0.3), mats.yellowLine);
        dash.rotation.x = -Math.PI / 2;
        dash.position.set(roadCol ? 0 : t, 0.04, roadCol ? t : 0);
        group.add(dash);
      }
      const poleOff = rw/2 + SIDEWALK_W*0.5;
      for (const sign of [-1, 1]) {
        group.add(makeBox(0.25, STREETLIGHT_H, 0.25, mats.poleMat, roadCol ? sign*poleOff : 0, STREETLIGHT_H/2, roadCol ? 0 : sign*poleOff));
        group.add(makeBox(roadCol ? 1.8 : 0.18, 0.18, roadCol ? 0.18 : 1.8, mats.poleMat, roadCol ? sign*(poleOff-0.9) : 0, STREETLIGHT_H, roadCol ? 0 : sign*(poleOff-0.9)));
        group.add(makeBox(0.6, 0.35, 0.6, mats.bulbMat, roadCol ? sign*(poleOff-1.8) : 0, STREETLIGHT_H-0.18, roadCol ? 0 : sign*(poleOff-1.8)));
      }
      const treeOff = rw/2 + SIDEWALK_W*0.85;
      for (let t = 0; t < 3; t++) {
        const pos = -TILE_SIZE/2 + (t+0.5)/3*TILE_SIZE;
        for (const sign of [-1, 1]) {
          const seed = hash2(gx+t, gz+sign, 99);
          if (seed > 0.4) {
            const lm = seed > 0.85 ? mats.leafCherry : seed > 0.7 ? mats.leafAutumn : mats.leafGreen;
            const tx = roadCol ? sign*treeOff : pos;
            const tz = roadCol ? pos : sign*treeOff;
            group.add(makeBox(TREE_TRUNK_W, TREE_TRUNK_H, TREE_TRUNK_W, mats.trunkMat, tx, TREE_TRUNK_H/2, tz));
            for (const c of [{s:3.2,y:TREE_TRUNK_H+1.5},{s:2.4,y:TREE_TRUNK_H+2.6},{s:1.5,y:TREE_TRUNK_H+3.4}]) {
              group.add(makeBox(c.s, c.s*0.75, c.s, lm, tx, c.y, tz));
            }
          }
        }
      }
    } else {
      for (const sign of [-1, 1]) {
        for (const axis of [0, 1]) {
          const bar = new THREE.Mesh(new THREE.PlaneGeometry(axis===0 ? ROAD_WIDTH_MAIN : 0.5, axis===0 ? 0.5 : ROAD_WIDTH_MAIN), mats.whiteLine);
          bar.rotation.x = -Math.PI / 2;
          const off = ROAD_WIDTH_MAIN/2 + 1.0;
          bar.position.set(axis===0 ? 0 : sign*off, 0.04, axis===0 ? sign*off : 0);
          group.add(bar);
        }
      }
    }
  } else {
    const dist = Math.sqrt(gx*gx + gz*gz);
    const density = Math.max(0, 1.0 - dist/80);
    
    // Divide tile into smaller plots, tightly packed
    const plots = [
      { px: -1, pz: -1 }, { px: 1, pz: -1 },
      { px: -1, pz: 1 }, { px: 1, pz: 1 }
    ];
    
    const plotSize = (TILE_SIZE / 2) - 0.2; // Tightly packed buildings
    
    plots.forEach((plot, i) => {
      const pSeed = hash2(gx, gz, i);
      // Sparsify outskirts
      if (pSeed > density * 1.5 && density < 0.4) return;

      const pxOffset = plot.px * (TILE_SIZE / 4);
      const pzOffset = plot.pz * (TILE_SIZE / 4);
      
      const localN = fbm(worldX + pxOffset, worldZ + pzOffset);
      const bh = BUILDING_MIN_H + Math.pow(localN, 0.8) * (0.4 + density*0.6) * (BUILDING_MAX_H - BUILDING_MIN_H) * (0.6 + pSeed*0.4);
      
      const bw = plotSize - 1.0;
      const bd = plotSize - 1.0;
      
      const bMat = mats.buildingMats[Math.floor(hash(gx*3+i, gz*7+i) * mats.buildingMats.length)];
      group.add(makeBox(bw, bh, bd, bMat, pxOffset, bh/2, pzOffset));
      group.add(makeBox(bw, 0.6, bd, mats.roofMat, pxOffset, bh+0.3, pzOffset));
      
      const winMat = mats.windowMats[Math.floor(hash2(gx+i, gz+i, 1) * mats.windowMats.length)];
      const wRows = Math.floor((bh-2)/3.5);
      const wColsX = Math.floor(bw/3.2);
      const wColsZ = Math.floor(bd/3.2);
      
      for (let r = 0; r < wRows; r++) {
        const wy = 2.0 + r*3.5 + 1.4;
        for (let c = 0; c < wColsX; c++) {
          if (hash2(gx*10+c, gz*10+r, 5+i) > 0.3) {
            const wx = pxOffset - bw/2 + 1.5 + c*(bw/wColsX);
            group.add(makeBox(1.4, 1.6, 0.12, winMat, wx, wy, pzOffset + bd/2+0.01));
            group.add(makeBox(1.4, 1.6, 0.12, winMat, wx, wy, pzOffset - bd/2-0.01));
          }
        }
        for (let c = 0; c < wColsZ; c++) {
          if (hash2(gx*10+c+50, gz*10+r+50, 6+i) > 0.3) {
            const wz = pzOffset - bd/2 + 1.5 + c*(bd/wColsZ);
            group.add(makeBox(0.12, 1.6, 1.4, winMat, pxOffset - bw/2-0.01, wy, wz));
            group.add(makeBox(0.12, 1.6, 1.4, winMat, pxOffset + bw/2+0.01, wy, wz));
          }
        }
      }
      
      if (bh > 25 && hash(gx*5+i, gz*3+i) > 0.6) {
        const ni = Math.floor(hash2(gx+i, gz+i, 2) * mats.neonMats.length);
        const sh = 3 + hash2(gx, gz, 8+i)*3;
        const sw = 4 + hash2(gx, gz, 9+i)*4;
        const sy = bh * (0.5 + hash2(gx, gz, 10+i)*0.4);
        group.add(makeBox(sw, sh, 0.4, mats.neonMats[ni], pxOffset, sy, pzOffset + bd/2+0.25));
        group.add(makeBox(sw+0.6, sh+0.6, 0.15, new THREE.MeshBasicMaterial({ color: mats.neonColors[ni], transparent: true, opacity: 0.25, depthWrite: false }), pxOffset, sy, pzOffset + bd/2+0.1));
      }
      
      if (bh > 15 && hash(gx*7+i, gz*11+i) > 0.4) {
        const bc = Math.floor(hash2(gx, gz, 3+i)*2)+1;
        for (let b = 0; b < bc; b++) {
          const rx = pxOffset + (hash2(gx, gz, b+20+i)-0.5)*(bw-4);
          const rz = pzOffset + (hash2(gx, gz, b+30+i)-0.5)*(bd-4);
          const rw2 = 1.5 + hash2(gx, gz, b+40+i)*2;
          const rh = 1.0 + hash2(gx, gz, b+50+i)*2;
          group.add(makeBox(rw2, rh, rw2, mats.roofMat, rx, bh+rh/2, rz));
        }
      }
    });
  }

  group.position.set(worldX, 0, worldZ);
  return group;
}

class ChunkManager {
  constructor(scene, mats) {
    this.scene = scene; this.mats = mats;
    this.loaded = new Map(); this.pending = new Set(); this.queue = [];
    this.PER_FRAME = 2;
  }
  update(cx, cz) {
    const cgx = Math.round(cx / TILE_SIZE);
    const cgz = Math.round(cz / TILE_SIZE);
    for (let dx = -LOAD_RADIUS; dx <= LOAD_RADIUS; dx++) {
      for (let dz = -LOAD_RADIUS; dz <= LOAD_RADIUS; dz++) {
        const gx = cgx+dx, gz = cgz+dz, key = `${gx},${gz}`;
        if (!this.loaded.has(key) && !this.pending.has(key)) {
          this.queue.push({ gx, gz, key, dist: Math.max(Math.abs(dx), Math.abs(dz)) });
          this.pending.add(key);
        }
      }
    }
    this.queue.sort((a,b) => a.dist-b.dist);
    let built = 0;
    while (this.queue.length > 0 && built < this.PER_FRAME) {
      const { gx, gz, key } = this.queue.shift();
      if (!this.loaded.has(key)) {
        const g = buildChunk(gx, gz, this.mats);
        this.scene.add(g); this.loaded.set(key, g);
      }
      this.pending.delete(key); built++;
    }
    for (const [key, g] of this.loaded) {
      const [kx, kz] = key.split(",").map(Number);
      if (Math.abs(kx-cgx) > RENDER_RADIUS || Math.abs(kz-cgz) > RENDER_RADIUS) {
        this.scene.remove(g);
        g.traverse(o => { if (o.geometry) o.geometry.dispose(); });
        this.loaded.delete(key);
        this.queue = this.queue.filter(q => q.key !== key);
        this.pending.delete(key);
      }
    }
  }
  loadedCount() { return this.loaded.size; }
  pendingCount() { return this.queue.length; }
}

class FreeCam {
  constructor(camera, canvas) {
    this.camera = camera; this.canvas = canvas;
    this.yaw = 0; this.pitch = -0.15;
    this.keys = {}; this.locked = false;
    this.speed = 20; this.fast = 90;
    this._k  = (e) => { this.keys[e.code] = e.type === "keydown"; };
    this._mv = (e) => this._move(e);
    this._lk = ()  => { this.locked = document.pointerLockElement === canvas; };
    this._cl = ()  => { if (!this.locked) canvas.requestPointerLock(); };
    this._wh = (e) => { this.camera.position.y = Math.max(1.5, this.camera.position.y - e.deltaY*0.05); };
    window.addEventListener("keydown", this._k);
    window.addEventListener("keyup",   this._k);
    document.addEventListener("pointerlockchange", this._lk);
    canvas.addEventListener("click", this._cl);
    canvas.addEventListener("wheel", this._wh, { passive: true });
    document.addEventListener("mousemove", this._mv);
    this._apply();
  }
  _move(e) {
    if (!this.locked) return;
    this.yaw   -= e.movementX * 0.002;
    this.pitch -= e.movementY * 0.002;
    this.pitch = Math.max(-Math.PI/2+0.05, Math.min(Math.PI/2-0.05, this.pitch));
    this._apply();
  }
  _apply() {
    const qY = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0), this.yaw);
    const qX = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0), this.pitch);
    this.camera.quaternion.copy(qY).multiply(qX);
  }
  update(dt) {
    const K = this.keys;
    const spd = (K["ShiftLeft"] || K["ShiftRight"]) ? this.fast : this.speed;
    const dir = new THREE.Vector3(); this.camera.getWorldDirection(dir);
    const right = new THREE.Vector3().crossVectors(dir, new THREE.Vector3(0,1,0)).normalize();
    const mv = new THREE.Vector3();
    if (K["KeyW"] || K["ArrowUp"])    mv.add(dir);
    if (K["KeyS"] || K["ArrowDown"])  mv.sub(dir);
    if (K["KeyA"] || K["ArrowLeft"])  mv.sub(right);
    if (K["KeyD"] || K["ArrowRight"]) mv.add(right);
    if (K["KeyE"] || K["PageUp"])     mv.add(new THREE.Vector3(0,1,0));
    if (K["KeyQ"] || K["PageDown"])   mv.sub(new THREE.Vector3(0,1,0));
    if (mv.lengthSq() > 0) this.camera.position.addScaledVector(mv.normalize(), spd*dt);
    this.camera.position.y = Math.max(1.5, this.camera.position.y);
  }
  dispose() {
    window.removeEventListener("keydown", this._k);
    window.removeEventListener("keyup",   this._k);
    document.removeEventListener("pointerlockchange", this._lk);
    this.canvas.removeEventListener("click",     this._cl);
    this.canvas.removeEventListener("wheel",     this._wh);
    document.removeEventListener("mousemove",    this._mv);
  }
}

export default function VoxelWorld() {
  const mountRef = useRef(null);
  const [stats, setStats]       = useState({ chunks:0, pending:0, x:"0", y:"0", z:"0", fps:0, tile:"0,0" });
  const [locked, setLocked]     = useState(false);
  const [showHelp, setShowHelp] = useState(true);

  useEffect(() => {
    const el = mountRef.current;
    if (!el) return;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(el.clientWidth, el.clientHeight);
    renderer.setClearColor(0x05050f);
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.2;
    el.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.fog = new THREE.Fog(0x05050f, TILE_SIZE*4, TILE_SIZE*(RENDER_RADIUS-1));
    scene.background = new THREE.Color(0x05050f);
    scene.add(new THREE.AmbientLight(0x0a0a1a, 0.8));
    const moon = new THREE.DirectionalLight(0x2233aa, 0.4);
    moon.position.set(200, 400, 100);
    scene.add(moon);

    const starCount = 2000;
    const sp = new Float32Array(starCount * 3);
    for (let i = 0; i < starCount*3; i+=3) {
      sp[i]   = (Math.random()-0.5)*3000;
      sp[i+1] = 200 + Math.random()*500;
      sp[i+2] = (Math.random()-0.5)*3000;
    }
    const sg = new THREE.BufferGeometry();
    sg.setAttribute("position", new THREE.BufferAttribute(sp, 3));
    const stars = new THREE.Points(sg, new THREE.PointsMaterial({ color: 0xffffff, size: 0.8 }));
    scene.add(stars);

    const camera = new THREE.PerspectiveCamera(75, el.clientWidth/el.clientHeight, 0.5, TILE_SIZE*RENDER_RADIUS*1.5);
    camera.position.set(0, 18, 0);

    const mats = new MatCache();
    const chunks = new ChunkManager(scene, mats);
    const freeCam = new FreeCam(camera, renderer.domElement);

    const onLC = () => setLocked(document.pointerLockElement === renderer.domElement);
    document.addEventListener("pointerlockchange", onLC);

    let lastT = performance.now(), fc = 0, ft = 0, fps = 0, aId;
    const loop = () => {
      aId = requestAnimationFrame(loop);
      const now = performance.now();
      const dt  = Math.min((now-lastT)/1000, 0.1); lastT = now;
      fc++; ft += dt;
      if (ft >= 0.5) { fps = Math.round(fc/ft); fc = 0; ft = 0; }
      freeCam.update(dt);
      const cx = camera.position.x, cy = camera.position.y, cz = camera.position.z;
      chunks.update(cx, cz);
      stars.position.set(cx, 0, cz);
      renderer.render(scene, camera);
      setStats({
        chunks: chunks.loadedCount(), pending: chunks.pendingCount(),
        x: cx.toFixed(1), y: cy.toFixed(1), z: cz.toFixed(1), fps,
        tile: `${Math.round(cx/TILE_SIZE)}, ${Math.round(cz/TILE_SIZE)}`
      });
    };
    loop();

    const onResize = () => {
      camera.aspect = el.clientWidth/el.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(el.clientWidth, el.clientHeight);
    };
    window.addEventListener("resize", onResize);
    const onH = (e) => { if (e.code === "KeyH") setShowHelp(v => !v); };
    window.addEventListener("keydown", onH);

    return () => {
      cancelAnimationFrame(aId); freeCam.dispose();
      document.removeEventListener("pointerlockchange", onLC);
      window.removeEventListener("resize", onResize);
      window.removeEventListener("keydown", onH);
      renderer.dispose();
      if (el.contains(renderer.domElement)) el.removeChild(renderer.domElement);
    };
  }, []);

  const hudStyle = {
    position:"absolute", background:"rgba(5,5,15,0.88)", backdropFilter:"blur(4px)",
    padding:"10px 14px", fontSize:11, lineHeight:2, letterSpacing:1, color:"#7a9aaa"
  };

  return (
    <div style={{ width:"100vw", height:"100vh", background:"#05050f", position:"relative", overflow:"hidden", fontFamily:"'Courier New',monospace" }}>
      <div ref={mountRef} style={{ width:"100%", height:"100%" }} />

      {!locked && (
        <div
          onClick={() => mountRef.current?.querySelector("canvas")?.requestPointerLock()}
          style={{
            position:"absolute", inset:0, display:"flex", flexDirection:"column",
            alignItems:"center", justifyContent:"center",
            background:"rgba(5,5,15,0.82)", backdropFilter:"blur(8px)",
            cursor:"pointer", zIndex:10
          }}
        >
          <div style={{
            fontSize:52, fontWeight:900, letterSpacing:6, marginBottom:8,
            background:"linear-gradient(135deg, #00f0ff 0%, #9900ff 100%)",
            WebkitBackgroundClip:"text", WebkitTextFillColor:"transparent"
          }}>VOXEL CITY</div>
          <div style={{ color:"#556677", fontSize:12, marginBottom:40, letterSpacing:4 }}>
            INFINITE PROCEDURAL WORLD &nbsp;�&nbsp; FREE CAM &nbsp;�&nbsp; ON-DEMAND STREAMING
          </div>
          <div style={{
            padding:"14px 48px", border:"1px solid rgba(0,240,255,0.6)",
            color:"#00f0ff", fontSize:13, letterSpacing:4,
            background:"rgba(0,240,255,0.06)",
            boxShadow:"0 0 32px rgba(0,240,255,0.2)",
            animation:"vPulse 2s ease-in-out infinite"
          }}>CLICK TO ENTER</div>
          <div style={{ marginTop:20, color:"#223344", fontSize:10, letterSpacing:3 }}>
            MIDNIGHT CLUB SCALE � TILE=40u � THREE.JS
          </div>
          <style>{`@keyframes vPulse{0%,100%{opacity:1}50%{opacity:0.55}}`}</style>
        </div>
      )}

      <div style={{ ...hudStyle, top:14, left:14, border:"1px solid rgba(0,240,255,0.2)", boxShadow:"0 0 16px rgba(0,240,255,0.08)" }}>
        <div style={{ color:"#00f0ff", fontWeight:700, letterSpacing:2, marginBottom:2 }}>? WORLD</div>
        <div>FPS&nbsp;&nbsp; <span style={{ color: stats.fps>50?"#00ff88":stats.fps>30?"#ffaa00":"#ff4444" }}>{stats.fps}</span></div>
        <div>CHUNKS&nbsp; <span style={{ color:"#fff" }}>{stats.chunks}</span> <span style={{ color:"#555" }}>+</span> <span style={{ color:"#ffaa00" }}>{stats.pending}</span></div>
        <div>TILE&nbsp;&nbsp;&nbsp; <span style={{ color:"#fff" }}>{stats.tile}</span></div>
        <div>POS&nbsp;&nbsp;&nbsp;&nbsp; <span style={{ color:"#ccc" }}>{stats.x} � {stats.y} � {stats.z}</span></div>
      </div>

      {showHelp && locked && (
        <div style={{ ...hudStyle, top:14, right:14, border:"1px solid rgba(153,0,255,0.25)", boxShadow:"0 0 16px rgba(153,0,255,0.1)" }}>
          <div style={{ color:"#9900ff", fontWeight:700, letterSpacing:2, marginBottom:2 }}>? CONTROLS</div>
          <div><span style={{ color:"#fff" }}>W A S D</span>&nbsp;&nbsp; Fly</div>
          <div><span style={{ color:"#fff" }}>MOUSE&nbsp;</span>&nbsp;&nbsp; Look</div>
          <div><span style={{ color:"#fff" }}>E / Q&nbsp;</span>&nbsp;&nbsp; Up / Down</div>
          <div><span style={{ color:"#fff" }}>SCROLL</span>&nbsp;&nbsp; Altitude</div>
          <div><span style={{ color:"#fff" }}>SHIFT&nbsp;</span>&nbsp;&nbsp; Fast (4x)</div>
          <div><span style={{ color:"#fff" }}>H&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</span>&nbsp;&nbsp; Toggle HUD</div>
          <div><span style={{ color:"#fff" }}>ESC&nbsp;&nbsp;&nbsp;</span>&nbsp;&nbsp; Release</div>
        </div>
      )}

      {locked && (
        <div style={{ position:"absolute", top:"50%", left:"50%", transform:"translate(-50%,-50%)", width:18, height:18, pointerEvents:"none" }}>
          <div style={{ position:"absolute", top:"50%", left:0, right:0, height:1, background:"rgba(255,255,255,0.55)", transform:"translateY(-50%)" }} />
          <div style={{ position:"absolute", left:"50%", top:0, bottom:0, width:1, background:"rgba(255,255,255,0.55)", transform:"translateX(-50%)" }} />
        </div>
      )}

      <div style={{ position:"absolute", bottom:16, left:"50%", transform:"translateX(-50%)", color:"rgba(0,240,255,0.22)", fontSize:9, letterSpacing:5, userSelect:"none", pointerEvents:"none" }}>
        VOXEL CITY � MIDNIGHT CLUB SCALE � TILE 40u � RENDER R{RENDER_RADIUS}
      </div>
    </div>
  );
}

