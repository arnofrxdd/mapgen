import { useEffect, useRef } from "react";
import * as THREE from "three";

// Road half-widths (full width = hw * 2)
const ROAD_HW = {
  highway:7.5,causeway:6.0,street:4.5,coast:4.0,
  suburb_road:3.5,ramp:4.0,alley:2.0,
};
const BRIDGE_ELEV=12.0,ROAD_H=0.30,CURB_H=0.36,CURB_W=1.1;
const FENCE_H=1.4,LAMP_H=8.5,LAMP_SPACING=28,CAR_W=1.8,CAR_D=3.5;
const CHUNK_SIZE=1200;

// Deterministic hash functions
const H1=(x,z)=>{const n=Math.sin(x*127.1+z*311.7)*43758.5453123;return n-Math.floor(n);};
const H2=(x,z,s)=>{const n=Math.sin(x*93.9898+z*67.233+s*17.1)*43758.5453123;return n-Math.floor(n);};

function findLineIntersection(ax, ay, ux, uy, bx, by, vx, vy) {
  const denom = -ux * vy + uy * vx;
  if (Math.abs(denom) < 0.0001) {
    return { x: (ax + bx) * 0.5, y: (ay + by) * 0.5 };
  }
  const t = (-(bx - ax) * vy + (by - ay) * vx) / denom;
  return { x: ax + t * ux, y: ay + t * uy };
}

function calculateJunctionPolygons(edges, nodes) {
  const nodeEdges = new Map();
  for (const e of edges) {
    if (!nodeEdges.has(e.n1.id)) nodeEdges.set(e.n1.id, []);
    if (!nodeEdges.has(e.n2.id)) nodeEdges.set(e.n2.id, []);
    nodeEdges.get(e.n1.id).push({ edge: e, isN1: true });
    nodeEdges.get(e.n2.id).push({ edge: e, isN1: false });
  }

  const junctionPolygons = new Map();

  for (const [nodeId, conn] of nodeEdges.entries()) {
    const n = nodes.find(nod => nod.id === nodeId);
    if (!n) continue;

    const roads = conn.map(c => {
      const edge = c.edge;
      const neighbor = c.isN1 ? edge.n2 : edge.n1;
      const angle = Math.atan2(neighbor.y - n.y, neighbor.x - n.x);
      const hw = ROAD_HW[edge.type] ?? 2.0;
      return { edge, angle, hw, neighbor, isN1: c.isN1 };
    });

    roads.sort((a, b) => a.angle - b.angle);
    const k = roads.length;
    const roadOffsets = new Map();
    let shapePts = [];

    if (k >= 2) {
      for (let i = 0; i < k; i++) {
        const r1 = roads[i];
        const r2 = roads[(i + 1) % k];

        // Ensure safe offset bounds to prevent polygon twisting
        const offset = Math.max(r1.hw, r2.hw) + 0.2;
        roadOffsets.set(r1.edge.id, offset);

        const cos1 = Math.cos(r1.angle), sin1 = Math.sin(r1.angle);
        const cos2 = Math.cos(r2.angle), sin2 = Math.sin(r2.angle);

        // R1 Right boundary
        const r1RightX = n.x + cos1 * offset + sin1 * r1.hw;
        const r1RightY = n.y + sin1 * offset - cos1 * r1.hw;
        
        // R2 Left boundary
        const r2LeftX = n.x + cos2 * offset - sin2 * r2.hw;
        const r2LeftY = n.y + sin2 * offset + cos2 * r2.hw;

        shapePts.push({ type: 'line', x: n.x + cos1 * offset - sin1 * r1.hw, y: n.y + sin1 * offset + cos1 * r1.hw }); // R1 Left
        shapePts.push({ type: 'line', x: r1RightX, y: r1RightY }); // R1 Right
        
        // Find intersection of boundaries to use as Bezier control point for smooth filleted corner
        const cp = findLineIntersection(
          r1RightX, r1RightY, cos1, sin1,
          r2LeftX, r2LeftY, -cos2, -sin2
        );

        const maxCpDist = offset * 2.0;
        const cpDist = Math.hypot(cp.x - n.x, cp.y - n.y);
        if (cpDist > maxCpDist) {
          cp.x = n.x + ((cp.x - n.x) / cpDist) * maxCpDist;
          cp.y = n.y + ((cp.y - n.y) / cpDist) * maxCpDist;
        }

        shapePts.push({ type: 'quad', cpX: cp.x, cpY: cp.y, x: r2LeftX, y: r2LeftY });
      }
    } else if (k === 1) {
      const r = roads[0];
      const offset = r.hw;
      roadOffsets.set(r.edge.id, offset);
      
      const cos = Math.cos(r.angle), sin = Math.sin(r.angle);
      const lx = n.x + cos * offset - sin * r.hw, ly = n.y + sin * offset + cos * r.hw;
      const rx = n.x + cos * offset + sin * r.hw, ry = n.y + sin * offset - cos * r.hw;
      
      shapePts.push({ type: 'line', x: lx, y: ly });
      shapePts.push({ type: 'line', x: rx, y: ry });
      // Semi-circle end cap
      shapePts.push({ type: 'quad', cpX: n.x + cos * (offset + r.hw), cpY: n.y + sin * (offset + r.hw), x: lx, y: ly });
    }

    junctionPolygons.set(nodeId, { shapePts, roadOffsets });
  }

  return junctionPolygons;
}

function bldgHeight(b,seed){
  const h=H1(b.x+seed*0.01,b.y+seed*0.07);
  const snap = v => Math.round(v / 4) * 4; // snap to voxel grid of 4 units
  if(b.type==="PARKING_LOT")return 0.5;
  if(b.type==="HOUSE")return snap(4+h*8);    // 4, 8, 12
  if(b.type==="TREE")return 4+Math.round(h*2)*2; // 4, 6, 8
  const dist=Math.sqrt(b.x*b.x+b.y*b.y);
  const cf=Math.max(0,1-dist/2800);
  if(cf>0.7)return snap(24+h*40);  // 24 - 64  downtown towers
  if(cf>0.4)return snap(12+h*20);  // 12 - 32  mid-rise
  return snap(8+h*12);             // 8  - 20  low-rise suburbs
}

const WATER_LVL = 0.35;
const CITY_LVL = 0.52;
const SUBURB_LVL = 0.65;

const randomNoise = (x, y) => {
  let n = Math.sin(x * 12.9898 + y * 78.233) * 43758.5453123;
  return n - Math.floor(n);
};

const smoothNoise2D = (x, y) => {
  const ix = Math.floor(x); const iy = Math.floor(y);
  const fx = x - ix; const fy = y - iy;
  const v1 = randomNoise(ix, iy);
  const v2 = randomNoise(ix + 1, iy);
  const v3 = randomNoise(ix, iy + 1);
  const v4 = randomNoise(ix + 1, iy + 1);
  const i1 = v1 + (v2 - v1) * fx;
  const i2 = v3 + (v4 - v3) * fx;
  return i1 + (i2 - i1) * fy;
};

const fbmNoise = (x, y, scale = 0.003) => {
  let v = 0, amp = 0.5, f = scale;
  v += smoothNoise2D(x * f, y * f) * amp; f *= 2; amp *= 0.5;
  v += smoothNoise2D(x * f, y * f) * amp; f *= 2; amp *= 0.5;
  v += smoothNoise2D(x * f, y * f) * amp; f *= 2; amp *= 0.5;
  v += smoothNoise2D(x * f, y * f) * amp;
  return v;
};

const getTerrainType = (worldX, worldY, offset, world) => {
  const cx = Math.floor(worldX / CHUNK_SIZE);
  const cy = Math.floor(worldY / CHUNK_SIZE);
  const centerX = cx * CHUNK_SIZE + 600;
  const centerY = cy * CHUNK_SIZE + 600;

  const warpX1 = (fbmNoise(worldX + offset, worldY + offset, 0.001) - 0.45) * 550;
  const warpY1 = (fbmNoise(worldX + offset + 500, worldY + offset + 500, 0.001) - 0.45) * 550;
  const warpX2 = (fbmNoise(worldX + offset + 1000, worldY + offset + 1000, 0.005) - 0.45) * 150;
  const warpY2 = (fbmNoise(worldX + offset + 1500, worldY + offset + 1500, 0.005) - 0.45) * 150;

  const warpedX = worldX - (warpX1 + warpX2);
  const warpedY = worldY - (warpY1 + warpY2);

  const wdx = warpedX - centerX, wdy = warpedY - centerY;
  const warpedDist = Math.sqrt(wdx * wdx + wdy * wdy);

  const val = fbmNoise(warpedX + offset, warpedY + offset);

  const maxRadius = CHUNK_SIZE * 0.49;
  const falloffStart = CHUNK_SIZE * 0.38;

  let finalVal = val;
  if (warpedDist > falloffStart) {
    finalVal -= ((warpedDist - falloffStart) / (maxRadius - falloffStart)) * 2.0;
  } else if (warpedDist < falloffStart * 0.5) {
    if (finalVal < WATER_LVL + 0.05) finalVal = WATER_LVL + 0.05;
  }

  let type;
  if (finalVal < WATER_LVL) type = "WATER";
  else if (finalVal < CITY_LVL) type = "CITY";
  else if (finalVal < SUBURB_LVL) type = "SUBURB";
  else type = "PARK";

  if ((type === "CITY" || type === "SUBURB") && world && world.generatedChunks) {
    const gx = Math.floor(worldX / 60);
    const gy = Math.floor(worldY / 60);
    const rCells = 3; // 180 radius
    let nearRoad = false;
    for (let i = gx - rCells; i <= gx + rCells; i++) {
        for (let j = gy - rCells; j <= gy + rCells; j++) {
            const cx = Math.floor((i * 60) / CHUNK_SIZE);
            const cy = Math.floor((j * 60) / CHUNK_SIZE);
            const key = `${cx},${cy}`;
            let chunk = world.generatedChunks.get(key);
            if (!chunk && world.activeChunk?.key === key) chunk = world.activeChunk;
            if (chunk && chunk.roadGrid && chunk.roadGrid.has(`${i},${j}`)) {
                nearRoad = true; break;
            }
        }
        if (nearRoad) break;
    }
    if (!nearRoad) type = "PARK";
  }

  return type;
};

function buildMaterials(){
  const std=opt=>new THREE.MeshStandardMaterial({roughness:0.7,metalness:0.1,...opt});
  const glow=hex=>new THREE.MeshBasicMaterial({color:hex});
  const m={
    gndCity:std({color:0x252535,roughness:0.95}),
    gndSuburb:std({color:0x2b3822,roughness:0.95}),
    gndPark:std({color:0x1b3c20,roughness:0.95}),
    gndWater:std({color:0x1e3f5f,roughness:0.1,metalness:0.9}),
    asphalt:std({color:0x22222a,roughness:0.6}),
    highway:std({color:0x2c2620,roughness:0.6,metalness:0.1}),
    causeway:std({color:0x28221c,roughness:0.6,metalness:0.1}),
    ramp:std({color:0x2c2620,roughness:0.6,metalness:0.1}),
    alley:std({color:0x1a1a20,roughness:0.7}),
    parkPath:std({color:0x1c2b18,roughness:0.8}),
    centerLine:glow(0xffaa00),
    whiteDiv:glow(0xdddddd),
    greenLine:glow(0x33cc66),
    sidewalk:std({color:0x3c3c4a,roughness:0.5}),
    curbCity:std({color:0x46465a,roughness:0.6}),
    curbSuburb:std({color:0x323c2a,roughness:0.6}),
    bridgeSide:std({color:0x4c443a,roughness:0.5,metalness:0.2}),
    bridgeDeck:std({color:0x3c342c,roughness:0.6,metalness:0.1}),
    bridgePillar:std({color:0x383028,roughness:0.7}),
    tower:[
      std({color:0x2b3444,roughness:0.4,metalness:0.4}),
      std({color:0x342b40,roughness:0.4,metalness:0.4}),
      std({color:0x282832,roughness:0.5,metalness:0.3}),
      std({color:0x303a28,roughness:0.4,metalness:0.4}),
      std({color:0x3c2b2b,roughness:0.4,metalness:0.4}),
      std({color:0x223440,roughness:0.4,metalness:0.4})
    ],
    towerTop:std({color:0x20202b,roughness:0.6}),
    winWarm:std({color:0xffe3aa,roughness:0.15,metalness:0.8,emissive:0xffa844,emissiveIntensity:0.15}),
    winCyan:std({color:0xb2f0ff,roughness:0.05,metalness:0.9,emissive:0x22b2dd,emissiveIntensity:0.1}),
    winPink:std({color:0xffcce2,roughness:0.05,metalness:0.9,emissive:0xdd2277,emissiveIntensity:0.1}),
    winGreen:std({color:0xd2ffe2,roughness:0.05,metalness:0.9,emissive:0x22dd77,emissiveIntensity:0.1}),
    winAmber:std({color:0xffecc2,roughness:0.1,metalness:0.8,emissive:0xdd8822,emissiveIntensity:0.15}),
    winOff:std({color:0x28303f,roughness:0.05,metalness:0.95}),
    houseWall:std({color:0x3a3025,roughness:0.75}),
    houseRoof:std({color:0x4e2020,roughness:0.75}),
    houseWin:std({color:0xfffacd,roughness:0.1,metalness:0.8,emissive:0xffd700,emissiveIntensity:0.1}),
    parking:std({color:0x2b2b35,roughness:0.75}),
    carMats:[
      std({color:0xe63946,roughness:0.3,metalness:0.6}),
      std({color:0x457b9d,roughness:0.3,metalness:0.6}),
      std({color:0x2a9d8f,roughness:0.3,metalness:0.6}),
      std({color:0xf4a261,roughness:0.3,metalness:0.6}),
      std({color:0x8d99ae,roughness:0.3,metalness:0.6}),
      std({color:0xf8f9fa,roughness:0.2,metalness:0.7})
    ],
    trunk:std({color:0x5d422d,roughness:0.9}),
    leafGreen:std({color:0x2b6b21,roughness:0.8}),
    leafCherry:std({color:0xc74a81,roughness:0.8}),
    leafAutumn:std({color:0xd15b22,roughness:0.8}),
    neonCols:[0xff0066, 0x00f3ff, 0xa300ff, 0xff9900, 0x05ff99, 0xff00bb],
    poleMat:std({color:0x353540,roughness:0.4,metalness:0.7}),
    bulbMat:glow(0xfffdd0),
    headlight:glow(0xffffff),
    taillight:glow(0xff3322),
    fenceMat:std({color:0x3c4040,roughness:0.5,metalness:0.6}),
    clutterMat:std({color:0x4e4e5a,roughness:0.75}),
    warningRed:glow(0xff2211),
  };
  Object.defineProperty(m,'neonMats',{get(){
    if(!this._nm){
      this._nm=this.neonCols.map(c=>glow(c));
      this._ng=this.neonCols.map(c=>new THREE.MeshBasicMaterial({color:c,transparent:true,opacity:0.25,depthWrite:false,side:THREE.DoubleSide}));
    }
    return{solid:this._nm,glow:this._ng};
  }});
  return m;
}

// Instanced Chunk Builder collects all box calls and produces a few InstancedMeshes
class InstancedChunkBuilder {
  constructor(mats) {
    this.mats = mats;
    this.instances = new Map();
    this.dummy = new THREE.Object3D();
    this.extras = []; // Store non-box objects like halos directly
  }

  addBox(w, h, d, mat, px, py, pz, ry = 0, rz = 0, rx = 0) {
    this.dummy.scale.set(w, h, d);
    this.dummy.position.set(px, py, pz);
    this.dummy.rotation.set(rx, ry, rz, "YXZ");
    this.dummy.updateMatrix();
    if (!this.instances.has(mat)) {
      this.instances.set(mat, []);
    }
    this.instances.get(mat).push(this.dummy.matrix.clone());
  }
  
  addExtra(mesh) {
    this.extras.push(mesh);
  }

  addPrism(w, h, d, mat, px, py, pz, ry = 0) {
    this.dummy.scale.set(w, h, d);
    this.dummy.position.set(px, py, pz);
    this.dummy.rotation.set(0, ry, 0, "YXZ");
    this.dummy.updateMatrix();
    if (!this.prismInstances) this.prismInstances = new Map();
    if (!this.prismInstances.has(mat)) {
      this.prismInstances.set(mat, []);
    }
    this.prismInstances.get(mat).push(this.dummy.matrix.clone());
  }

  build(group) {
    const geo = new THREE.BoxGeometry(1, 1, 1);
    for (const [mat, matrices] of this.instances.entries()) {
      const mesh = new THREE.InstancedMesh(geo, mat, matrices.length);
      for (let i = 0; i < matrices.length; i++) {
        mesh.setMatrixAt(i, matrices[i]);
      }
      mesh.instanceMatrix.needsUpdate = true;
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      group.add(mesh);
    }

    if (this.prismInstances) {
      const prismGeo = new THREE.CylinderGeometry(0.5, 0.5, 1, 3);
      for (const [mat, matrices] of this.prismInstances.entries()) {
        const mesh = new THREE.InstancedMesh(prismGeo, mat, matrices.length);
        for (let i = 0; i < matrices.length; i++) {
          mesh.setMatrixAt(i, matrices[i]);
        }
        mesh.instanceMatrix.needsUpdate = true;
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        group.add(mesh);
      }
    }

    for (const extra of this.extras) {
      extra.castShadow = true;
      extra.receiveShadow = true;
      group.add(extra);
    }
  }
}

function buildRoad(builder,e,mats,nodeConnections,junctionPolygons){
  const dx=e.n2.x-e.n1.x,dz=e.n2.y-e.n1.y,len=Math.hypot(dx,dz);
  if(len<1)return;
  const ang=Math.atan2(dz,dx);
  
  const hw=ROAD_HW[e.type]??2;
  const n1Poly = junctionPolygons.get(e.n1.id);
  const n2Poly = junctionPolygons.get(e.n2.id);
  
  const offset1 = n1Poly && n1Poly.roadOffsets.has(e.id) ? n1Poly.roadOffsets.get(e.id) : hw;
  const offset2 = n2Poly && n2Poly.roadOffsets.has(e.id) ? n2Poly.roadOffsets.get(e.id) : hw;
  
  const nx = dx / len;
  const nz = dz / len;
  
  let startX = e.n1.x, startZ = e.n1.y;
  let endX = e.n2.x, endZ = e.n2.y;
  let newLen = len;
  
  if (len > offset1 + offset2 + 1.0) {
    startX = e.n1.x + nx * offset1;
    startZ = e.n1.y + nz * offset1;
    endX = e.n2.x - nx * offset2;
    endZ = e.n2.y - nz * offset2;
    newLen = len - offset1 - offset2;
  }
  
  const cx = (startX + endX) * 0.5;
  const cz = (startZ + endZ) * 0.5;
  
  const isEL=e.isBridge&&(e.type==="highway"||e.type==="causeway");
  const y1 = e.n1.z > 0 ? BRIDGE_ELEV : 0;
  const y2 = e.n2.z > 0 ? BRIDGE_ELEV : 0;
  
  // Ramps slope exactly from the edge of the flat junctions
  const startY = y1;
  const endY = y2;
  const dy = endY - startY;
  const midY = (startY + endY) * 0.5;
  const pitch = Math.atan2(dy, newLen);
  
  const rMat={highway:mats.highway,causeway:mats.causeway,ramp:mats.ramp,alley:mats.alley}[e.type]||mats.asphalt;
  
  builder.addBox(newLen,ROAD_H,hw*2,rMat,cx,midY+ROAD_H/2,cz,-ang, pitch);
  
  if(isEL){
    // Girders running under the deck
    for(let g = -1; g <= 1; g++) {
       const o = g * (hw - 0.5);
       const ox=-Math.sin(ang)*o,oz=Math.cos(ang)*o;
       builder.addBox(newLen, 0.4, 0.4, mats.bridgeDeck || mats.bridgeSide, cx+ox, midY-0.2, cz+oz, -ang, pitch);
    }
    
    // Jersey Barriers with Handrails
    for(const s of[-1,1]){
      const o=s*(hw+0.2),ox=-Math.sin(ang)*o,oz=Math.cos(ang)*o;
      // Concrete base
      builder.addBox(newLen, 0.8, 0.4, mats.bridgeSide, cx+ox, midY+ROAD_H+0.4, cz+oz, -ang, pitch);
      // Metal top rail
      builder.addBox(newLen, 0.15, 0.15, mats.poleMat, cx+ox, midY+ROAD_H+1.0, cz+oz, -ang, pitch);
    }
    
    const isCauseway = e.type === "causeway";
    const spacing = isCauseway ? 80 : 30;
    const pc=Math.max(1,Math.floor(newLen/spacing));
    
    for(let i=0;i<=pc;i++){
      const t=i/pc;
      const py = startY + dy * t;
      const px = startX + nx * newLen * t;
      const pz = startZ + nz * newLen * t;
      
      if (py < 1.0) continue; // Too low for pillars

      if (isCauseway) {
        // Massive Cable-Stayed Suspension Tower
        const twH = 25; 
        for(const s of[-1,1]){
           const o=s*(hw+1.5),ox=-Math.sin(ang)*o,oz=Math.cos(ang)*o;
           const legH = py + twH;
           builder.addBox(1.5, legH, 1.5, mats.bridgePillar, px+ox, legH/2, pz+oz, -ang);
        }
        // Top cross beam
        builder.addBox(1.5, 1.0, hw*2+4.5, mats.bridgeSide, px, py+twH-0.5, pz, -ang);
        
        // Tension Cables
        const numCables = 5;
        for (const dir of [-1, 1]) {
           for (let c=1; c<=numCables; c++) {
              const cableDist = c * 12; 
              const offsetFromStart = t * newLen + cableDist * dir;
              if (offsetFromStart < 0 || offsetFromStart > newLen) continue;

              const cpx = startX + nx * offsetFromStart;
              const cpz = startZ + nz * offsetFromStart;
              const cpy = startY + dy * (offsetFromStart / newLen);
              
              for(const s of[-1,1]){
                 const o=s*(hw+0.5); 
                 const oxC = -Math.sin(ang)*o, ozC = Math.cos(ang)*o;
                 
                 // Anchor 1: Tower Top (Left or Right)
                 const twOx = s*(hw+1.5), twOxC = -Math.sin(ang)*twOx, twOzC = Math.cos(ang)*twOx;
                 const tX = px + twOxC;
                 const tY = py + twH - 1.0;
                 const tZ = pz + twOzC;
                 
                 // Anchor 2: Deck Side Barrier
                 const dX = cpx + oxC;
                 const dY = cpy + ROAD_H + 0.5;
                 const dZ = cpz + ozC;
                 
                 const dxC = dX - tX;
                 const dyC = dY - tY;
                 const dzC = dZ - tZ;
                 const cableLen = Math.hypot(dxC, dzC, dyC);
                 const cablePitch = Math.atan2(dyC, Math.hypot(dxC, dzC));
                 const cableAng = Math.atan2(dzC, dxC);
                 
                 builder.addBox(cableLen, 0.1, 0.1, mats.poleMat, (tX+dX)/2, (tY+dY)/2, (tZ+dZ)/2, -cableAng, cablePitch);
              }
           }
        }
      } else {
        // Highway T-Pillar
        builder.addBox(1.5, py, 1.5, mats.bridgePillar, px, py/2, pz, -ang);
        // Wide Crossbeam supporting the road
        builder.addBox(1.5, 1.0, hw*2+2, mats.bridgeSide, px, py-0.5, pz, -ang);
        // Angled V-struts for realistic structural support
        for (const s of [-1, 1]) {
           const strutOx = -Math.sin(ang) * (s * hw * 0.4);
           const strutOz = Math.cos(ang) * (s * hw * 0.4);
           // Using rx (10th param) to pitch sideways!
           builder.addBox(1.0, hw * 1.0, 1.0, mats.bridgePillar, px + strutOx, py-1.5, pz + strutOz, -ang, 0, s * 0.5);
        }
      }
    }
  }
  
  if(e.type==="street"||e.type==="suburb_road"||e.type==="coast"){
    const cm=e.type==="suburb_road"?mats.curbSuburb:mats.curbCity;
    for(const s of[-1,1]){
      const co=s*(hw+CURB_W/2),ox=-Math.sin(ang)*co,oz=Math.cos(ang)*co;
      builder.addBox(newLen,CURB_H,CURB_W,cm,cx+ox,midY+CURB_H/2,cz+oz,-ang, pitch);
    }
    const dC=Math.max(1,Math.floor(newLen/8));
    for(let i=0;i<dC;i++){
      const t=(i+0.5)/dC;
      const px=startX+nx*newLen*t;
      const pz2=startZ+nz*newLen*t;
      const py = startY + dy * t;
      builder.addBox(3.5,0.06,0.22,mats.centerLine,px,py+ROAD_H+0.03,pz2,-ang, pitch);
    }
    for(const s of[-1,1]){
      const lo=s*(hw-0.5),ox=-Math.sin(ang)*lo,oz=Math.cos(ang)*lo;
      builder.addBox(newLen,0.05,0.18,mats.whiteDiv,cx+ox,midY+ROAD_H+0.025,cz+oz,-ang, pitch);
    }
    
    const nL=Math.max(1,Math.floor(newLen/LAMP_SPACING));
    for(let i=0;i<nL;i++){
      const t=(i+0.5)/nL;
      const px=startX+nx*newLen*t;
      const pz2=startZ+nz*newLen*t;
      const py = startY + dy * t;
      const sides=nL<3?[-1,1]:[i%2===0?-1:1];
      for(const s of sides){
        const lo=s*(hw+CURB_W+0.4),ox=-Math.sin(ang)*lo,oz=Math.cos(ang)*lo;
        // Lamps vertical
        builder.addBox(0.2,LAMP_H,0.2,mats.poleMat,px+ox,py+LAMP_H/2,pz2+oz);
        const aD=s*-1,aLen=1.5,aH=LAMP_H-0.2;
        const aox=-Math.sin(ang)*aD*aLen*0.5+ox+px,aoz=Math.cos(ang)*aD*aLen*0.5+oz+pz2;
        builder.addBox(0.12,0.12,aLen,mats.poleMat,aox,py+aH,aoz, -ang);
        const bx2=-Math.sin(ang)*aD*aLen+ox+px,bz2=Math.cos(ang)*aD*aLen+oz+pz2;
        builder.addBox(0.5,0.25,0.5,mats.bulbMat,bx2,py+aH-0.13,bz2, -ang);
        
        const halo=new THREE.Mesh(new THREE.BoxGeometry(1.5,0.8,1.5),new THREE.MeshBasicMaterial({color:0xffefa0,transparent:true,opacity:0.25,depthWrite:false}));
        halo.position.set(bx2,py+aH-0.13,bz2);
        builder.addExtra(halo);
      }
    }
  }
  
  if(e.type==="highway"||e.type==="ramp"){
    const sC=Math.max(1,Math.floor(newLen/12));
    for(let i=0;i<sC;i++){
      const t=(i+0.5)/sC;
      const px=startX+nx*newLen*t;
      const pz2=startZ+nz*newLen*t;
      const py = startY + dy * t;
      builder.addBox(4.0,0.06,0.22,mats.whiteDiv,px,py+ROAD_H+0.03,pz2,-ang, pitch);
    }
    for(const s of[-1,1]){
      const ro=s*(hw-0.28),ox=-Math.sin(ang)*ro,oz=Math.cos(ang)*ro;
      builder.addBox(newLen,0.6,0.18,mats.bridgeSide,cx+ox,midY+ROAD_H+0.3,cz+oz,-ang, pitch);
    }
  }
  
  if(e.type==="alley"){
    // Narrow curbs on both sides
    for(const s of[-1,1]){
      const co=s*(hw+0.1),ox=-Math.sin(ang)*co,oz=Math.cos(ang)*co;
      builder.addBox(newLen,CURB_H,0.3,mats.curbCity,cx+ox,midY+CURB_H/2,cz+oz,-ang, pitch);
    }
    // Center divider line
    builder.addBox(newLen,0.05,0.12,mats.centerLine,cx,midY+ROAD_H+0.03,cz,-ang, pitch);
  }
}

function buildJunctions(builder, nodes, edges, mats, nodeConnections, junctionPolygons) {
  for (const [nodeId, poly] of junctionPolygons.entries()) {
    const n = nodes.find(nod => nod.id === nodeId);
    if (!n || !poly.shapePts || poly.shapePts.length < 3) continue;

    const isBridgeNode = n.z > 0;
    const baseY = isBridgeNode ? BRIDGE_ELEV : 0;

    const info = nodeConnections.get(nodeId);
    if (!info) continue;

    const isHighwayJunction = Array.from(info.types).some(t => t === "highway" || t === "causeway");
    const mat = isHighwayJunction ? mats.highway : mats.asphalt;

    const shape = new THREE.Shape();
    const pts = poly.shapePts;
    shape.moveTo(pts[0].x - n.x, pts[0].y - n.y);
    for (let i = 1; i < pts.length; i++) {
      if (pts[i].type === 'quad') {
        shape.quadraticCurveTo(pts[i].cpX - n.x, pts[i].cpY - n.y, pts[i].x - n.x, pts[i].y - n.y);
      } else {
        shape.lineTo(pts[i].x - n.x, pts[i].y - n.y);
      }
    }
    shape.closePath();

    const extrudeSettings = { depth: ROAD_H, bevelEnabled: false };
    const geom = new THREE.ExtrudeGeometry(shape, extrudeSettings);
    geom.rotateX(Math.PI / 2);

    const mesh = new THREE.Mesh(geom, mat);
    mesh.position.set(n.x, baseY + ROAD_H, n.y);
    mesh.castShadow = true;
    mesh.receiveShadow = true;
    builder.addExtra(mesh);

    if (isBridgeNode && isHighwayJunction) {
      // Thick structural under-pad
      const padGeom = new THREE.ExtrudeGeometry(shape, { depth: 0.8, bevelEnabled: false });
      padGeom.rotateX(Math.PI / 2);
      const padMesh = new THREE.Mesh(padGeom, mats.bridgeDeck || mats.bridgeSide);
      padMesh.position.set(n.x, baseY - 0.2, n.y);
      padMesh.castShadow = true;
      builder.addExtra(padMesh);
      
      // Massive Central Pillar
      builder.addBox(2.5, baseY, 2.5, mats.bridgePillar, n.x, baseY / 2, n.y);
      
      // Curved Edge Barriers along exposed corners
      let prevPt = pts[0];
      for (let i = 1; i < pts.length; i++) {
        const pt = pts[i];
        if (pt.type === 'quad') {
          const P0 = prevPt;
          const P1 = { x: pt.cpX, y: pt.cpY };
          const P2 = pt;
          
          const dist = Math.hypot(P2.x - P0.x, P2.y - P0.y) + Math.hypot(P1.x - P0.x, P1.y - P0.y);
          const segments = Math.max(3, Math.floor(dist / 2.0));
          const boxLen = (dist / segments) + 0.8;
          
          for (let s = 0; s <= segments; s++) {
            const t = s / segments;
            const mt = 1 - t;
            
            const bx = mt*mt*P0.x + 2*mt*t*P1.x + t*t*P2.x;
            const bz = mt*mt*P0.y + 2*mt*t*P1.y + t*t*P2.y;
            
            const dx = 2*mt*(P1.x - P0.x) + 2*t*(P2.x - P1.x);
            const dz = 2*mt*(P1.y - P0.y) + 2*t*(P2.y - P1.y);
            const ang = Math.atan2(dz, dx);
            
            // Concrete base
            builder.addBox(boxLen, 0.8, 0.4, mats.bridgeSide, bx, baseY+ROAD_H+0.4, bz, -ang);
            // Metal top rail
            builder.addBox(boxLen, 0.15, 0.15, mats.poleMat, bx, baseY+ROAD_H+1.0, bz, -ang);
          }
        }
        prevPt = pt;
      }
    }
  }
}

function buildBuilding(builder, b, mats, seed) {
  const bx = b.x, bz = b.y, bw = b.w, bd = b.h, ang = b.angle;
  const bh = bldgHeight(b, seed);
  const wMat = mats.winWarm;
  const tMat = mats.tower[Math.floor(H2(bx, bz, 2) * mats.tower.length)];
  const tTop = mats.towerTop;
  const h1 = H1(bx + seed, bz);
  
  const lbox = (w, h, d, mat, ly) => {
    builder.addBox(w, h, d, mat, bx, ly, bz, -ang);
  };
  const lboxOffset = (w, h, d, mat, ly, lx, lz) => {
    const cosA = Math.cos(-ang), sinA = Math.sin(-ang);
    const wx = bx + cosA * lx - sinA * lz;
    const wz = bz + sinA * lx + cosA * lz;
    builder.addBox(w, h, d, mat, wx, ly, wz, -ang);
  };

  if(b.type==="TREE"){
    const lm=h1>0.80?mats.leafCherry:h1>0.65?mats.leafAutumn:mats.leafGreen;
    const th=3.0+h1*1.5;
    builder.addBox(0.8,th,0.8,mats.trunk,bx,th/2,bz);
    for(const[s,y]of[[3.2,th+0.8],[2.4,th+1.8],[1.5,th+2.6]]) {
      builder.addBox(s,s*0.6,s,lm,bx,y,bz);
    }
    return;
  }

  if(b.type==="PARKING_LOT"){
    lbox(bw,0.5,bd,mats.parking,0.25);
    const cols=Math.max(1,Math.floor((bw-1)/2.8));
    for(let c=0;c<cols-1;c++){
      const lx=-bw/2+1.0+(c+1)*(bw-1)/cols;
      lboxOffset(0.1,0.06,bd-1.5,mats.whiteDiv,0.53,lx,0);
    }
    if(b.cars){
      const cosA = Math.cos(-ang), sinA = Math.sin(-ang);
      for(const car of b.cars){
        const wx=bx+cosA*car.lx-sinA*car.ly,wz=bz+sinA*car.lx+cosA*car.ly;
        const ci=Math.floor(H2(wx,wz,7)*mats.carMats.length);
        builder.addBox(CAR_W,0.7,CAR_D,mats.carMats[ci],wx,0.85,wz,-ang);
        builder.addBox(CAR_W*0.8,0.5,CAR_D*0.55,mats.carMats[ci],wx,1.35,wz,-ang);

        const fx=CAR_W*0.35, fz=CAR_D*0.5+0.05;
        const hx1=bx+cosA*(car.lx-fx)-sinA*(car.ly+fz), hz1=bz+sinA*(car.lx-fx)+cosA*(car.ly+fz);
        const hx2=bx+cosA*(car.lx+fx)-sinA*(car.ly+fz), hz2=bz+sinA*(car.lx+fx)+cosA*(car.ly+fz);
        builder.addBox(0.3,0.15,0.1,mats.headlight,hx1,0.8,hz1,-ang);
        builder.addBox(0.3,0.15,0.1,mats.headlight,hx2,0.8,hz2,-ang);
        
        const bz_val=-CAR_D*0.5-0.05;
        const tx1=bx+cosA*(car.lx-fx)-sinA*(car.ly+bz_val), tz1=bz+sinA*(car.lx-fx)+cosA*(car.ly+bz_val);
        const tx2=bx+cosA*(car.lx+fx)-sinA*(car.ly+bz_val), tz2=bz+sinA*(car.lx+fx)+cosA*(car.ly+bz_val);
        builder.addBox(0.3,0.12,0.1,mats.taillight,tx1,0.85,tz1,-ang);
        builder.addBox(0.3,0.12,0.1,mats.taillight,tx2,0.85,tz2,-ang);
      }
    }
    return;
  }

  if (b.type === "HOUSE") {
    const hw = bw * 0.8, hd = bd * 0.8;
    const hH = 3.0 + H2(bx, bz, 5) * 2.0;
    
    // Core house box
    lbox(hw, hH, hd, mats.houseWall, hH / 2);
    
    // Roof
    builder.addPrism(hw + 0.4, 2.0, hd + 0.4, mats.houseRoof, bx, hH + 1.0, bz, -ang);
    
    return;
  }

  // Commercial Buildings
  const finalLw = bw * 0.85;
  const finalLd = bd * 0.85;
  const archStyle = Math.floor(H2(bx, bz, 99) * 3);
  
  if (b.isTriangle) {
    // Solid triangular core
    builder.addPrism(finalLw, bh, finalLd, tMat, bx, bh / 2, bz, -ang);
    
    // Ledges to fake windows without buggy rendering
    const floors = Math.floor(bh / 3.0);
    for (let i = 1; i < floors; i++) {
       const fy = i * 3.0;
       builder.addPrism(finalLw + 0.4, 0.4, finalLd + 0.4, wMat, bx, fy, bz, -ang);
    }
    
    // Parapet
    builder.addPrism(finalLw, 0.8, finalLd, tTop, bx, bh + 0.4, bz, -ang);
  } else {
    // Rectangular core (Windows)
    lbox(finalLw, bh, finalLd, wMat, bh / 2);
    
    // Horizontal ledges wrapping around the core tightly
    const floorH = 3.0;
    const floors = Math.floor(bh / floorH);
    for (let i = 0; i <= floors; i++) {
       const fy = i * floorH;
       const ledgeThickness = (i === 0 || i === floors) ? 0.8 : 0.4;
       lbox(finalLw + 0.4, ledgeThickness, finalLd + 0.4, tMat, fy);
    }
    
    // Vertical columns perfectly aligned with the exterior boundary
    const colSpacing = 3.0;
    const colsW = Math.max(2, Math.floor(finalLw / colSpacing));
    const colW = archStyle === 0 ? 0.8 : 0.4;
    
    // Front and Back columns spanning the entire building width
    for (let i = 0; i <= colsW; i++) {
       const t = i / colsW;
       const cx = -finalLw / 2 + finalLw * t;
       lboxOffset(colW, bh, 0.4, tMat, bh / 2, cx, finalLd / 2);
       lboxOffset(colW, bh, 0.4, tMat, bh / 2, cx, -finalLd / 2);
    }
    
    // Left and Right columns
    const colsD = Math.max(2, Math.floor(finalLd / colSpacing));
    for (let i = 0; i <= colsD; i++) {
       const t = i / colsD;
       const cz = -finalLd / 2 + finalLd * t;
       lboxOffset(0.4, bh, colW, tMat, bh / 2, finalLw / 2, cz);
       lboxOffset(0.4, bh, colW, tMat, bh / 2, -finalLw / 2, cz);
    }
    
    // Roof Parapet and details
    lbox(finalLw, 0.8, finalLd, tTop, bh + 0.4);
    
    if (finalLw > 15 && finalLd > 15 && H2(bx, bz, 30) > 0.4) {
      lbox(10.0, 0.2, 10.0, tTop, bh + 0.8);
      lbox(9.0, 0.1, 9.0, mats.centerLine, bh + 1.0);
    }
    
    // Roof clutter
    const units = Math.floor(H2(bx, bz, 31) * 3) + 1;
    for (let i = 0; i < units; i++) {
       const rx = (H2(bx, bz, i + 35) - 0.5) * (finalLw * 0.5);
       const rz = (H2(bx, bz, i + 37) - 0.5) * (finalLd * 0.5);
       lboxOffset(1.8, 1.4, 1.8, mats.clutterMat, bh + 0.8, rx, rz);
    }
  }
}

function buildFence(builder,f,mats){
  const dx=f.x2-f.x1,dz=f.y2-f.y1,len=Math.hypot(dx,dz);
  if(len<0.5)return;
  const cx=(f.x1+f.x2)/2,cz=(f.y1+f.y2)/2,ang=Math.atan2(dz,dx);
  const rail=y=>{
    builder.addBox(len,0.12,0.12,mats.fenceMat,cx,y,cz,-ang);
  };
  rail(FENCE_H-0.06);rail(FENCE_H*0.5);
  const posts=Math.max(2,Math.floor(len/3));
  for(let i=0;i<=posts;i++){
    const t=i/posts;
    builder.addBox(0.12,FENCE_H,0.12,mats.fenceMat,f.x1+dx*t,FENCE_H/2,f.y1+dz*t);
  }
}

function buildChunkGroup(chunk,mats,seed,world){
  const group=new THREE.Group();
  
  let edgeId = 0;
  for (const e of chunk.edges) if (e.id === undefined) e.id = `e_${edgeId++}`;
  
  const nodeConnections = new Map();
  for (const e of chunk.edges) {
    if (!nodeConnections.has(e.n1.id)) nodeConnections.set(e.n1.id, { node: e.n1, types: new Set(), maxHW: 0 });
    if (!nodeConnections.has(e.n2.id)) nodeConnections.set(e.n2.id, { node: e.n2, types: new Set(), maxHW: 0 });
    const hw = ROAD_HW[e.type] ?? 2.0;
    nodeConnections.get(e.n1.id).types.add(e.type);
    nodeConnections.get(e.n1.id).maxHW = Math.max(nodeConnections.get(e.n1.id).maxHW, hw);
    nodeConnections.get(e.n2.id).types.add(e.type);
    nodeConnections.get(e.n2.id).maxHW = Math.max(nodeConnections.get(e.n2.id).maxHW, hw);
  }

  // Precompute exact junction polygon boundaries and clipping offsets
  const junctionPolygons = calculateJunctionPolygons(chunk.edges, chunk.nodes);

  const builder = new InstancedChunkBuilder(mats);
  
  // Render a 30x30 grid of terrain tiles (40m size) matching App.jsx terrain noise
  const TILE_RES = 30;
  const TILE_W = CHUNK_SIZE / TILE_RES;
  for (let tz = 0; tz < TILE_RES; tz++) {
    for (let tx = 0; tx < TILE_RES; tx++) {
      const px = chunk.cx * CHUNK_SIZE + (tx + 0.5) * TILE_W;
      const pz = chunk.cy * CHUNK_SIZE + (tz + 0.5) * TILE_W;
      const tType = getTerrainType(px, pz, seed, world);
      
      if (tType === "WATER") {
        // Recessed water (Y=-0.7, thickness=0.2)
        builder.addBox(TILE_W, 0.2, TILE_W, mats.gndWater, px, -0.7, pz);
      } else {
        const mat = tType === "CITY" ? mats.gndCity 
                  : tType === "SUBURB" ? mats.gndSuburb 
                  : mats.gndPark;
        // Flat land ground
        builder.addBox(TILE_W, 0.2, TILE_W, mat, px, -0.1, pz);
      }
    }
  }
  
  for(const e of chunk.edges)buildRoad(builder,e,mats,nodeConnections,junctionPolygons);
  buildJunctions(builder,chunk.nodes,chunk.edges,mats,nodeConnections,junctionPolygons);
  for(const b of chunk.buildings)buildBuilding(builder,b,mats,seed);
  for(const f of chunk.fences)buildFence(builder,f,mats);
  
  builder.build(group);
  return group;
}

class FreeCam{
  constructor(cam,canvas){
    this.cam=cam;this.canvas=canvas;this.yaw=0;this.pitch=-0.15;this.keys={};this.locked=false;this.speed=50;this.fast=240;
    this._k=e=>{this.keys[e.code]=e.type==="keydown";};
    this._mv=e=>this._mouse(e);this._lk=()=>{this.locked=document.pointerLockElement===canvas;};
    this._cl=()=>{if(!this.locked)canvas.requestPointerLock();};
    this._wh=e=>{e.preventDefault();this.cam.position.y=Math.max(1.5,this.cam.position.y-e.deltaY*0.25);};
    window.addEventListener("keydown",this._k);window.addEventListener("keyup",this._k);
    document.addEventListener("pointerlockchange",this._lk);canvas.addEventListener("click",this._cl);
    canvas.addEventListener("wheel",this._wh,{passive:false});document.addEventListener("mousemove",this._mv);this._apply();
  }
  _mouse(e){if(!this.locked)return;this.yaw-=e.movementX*0.002;this.pitch-=e.movementY*0.002;this.pitch=Math.max(-1.55,Math.min(1.55,this.pitch));this._apply();}
  _apply(){const qY=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),this.yaw);const qX=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1,0,0),this.pitch);this.cam.quaternion.copy(qY).multiply(qX);}
  update(dt){
    const K=this.keys,spd=(K.ShiftLeft||K.ShiftRight)?this.fast:this.speed;
    const dir=new THREE.Vector3();this.cam.getWorldDirection(dir);
    const right=new THREE.Vector3().crossVectors(dir,new THREE.Vector3(0,1,0)).normalize();
    const mv=new THREE.Vector3();
    if(K.KeyW||K.ArrowUp)mv.add(dir);if(K.KeyS||K.ArrowDown)mv.sub(dir);
    if(K.KeyA||K.ArrowLeft)mv.sub(right);if(K.KeyD||K.ArrowRight)mv.add(right);
    if(K.KeyE||K.PageUp)mv.y+=1;if(K.KeyQ||K.PageDown)mv.y-=1;
    if(mv.lengthSq()>0)this.cam.position.addScaledVector(mv.normalize(),spd*dt);
    this.cam.position.y=Math.max(1.5,this.cam.position.y);
  }
  dispose(){
    window.removeEventListener("keydown",this._k);window.removeEventListener("keyup",this._k);
    document.removeEventListener("pointerlockchange",this._lk);this.canvas.removeEventListener("click",this._cl);
    this.canvas.removeEventListener("wheel",this._wh);document.removeEventListener("mousemove",this._mv);
  }
}

export default function VoxelView({worldRef,onClose}){
  const mountRef=useRef(null),hudRef=useRef(null);
  useEffect(()=>{
    const el=mountRef.current;if(!el)return;
    const renderer=new THREE.WebGLRenderer({antialias:true});
    renderer.setPixelRatio(Math.min(window.devicePixelRatio,2));renderer.setSize(el.clientWidth,el.clientHeight);
    renderer.setClearColor(0xc2ddf7);
    renderer.toneMapping=THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure=1.0; 
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFShadowMap;
    el.appendChild(renderer.domElement);
    
    const scene=new THREE.Scene();
    scene.background=new THREE.Color(0xc2ddf7);
    scene.fog=new THREE.FogExp2(0xc2ddf7, 0.00035);
    
    const ambientLight = new THREE.HemisphereLight(0xffefff, 0x444a55, 0.85);
    scene.add(ambientLight);
    
    const sun=new THREE.DirectionalLight(0xfffaf0, 1.4);
    sun.position.set(400, 800, 200);
    sun.castShadow = true;
    sun.shadow.mapSize.width = 2048;
    sun.shadow.mapSize.height = 2048;
    sun.shadow.camera.near = 0.5;
    sun.shadow.camera.far = 2500;
    
    const shadowD = 1000;
    sun.shadow.camera.left = -shadowD;
    sun.shadow.camera.right = shadowD;
    sun.shadow.camera.top = shadowD;
    sun.shadow.camera.bottom = -shadowD;
    sun.shadow.bias = -0.0005;
    scene.add(sun);
    
    const fillLight=new THREE.DirectionalLight(0x9cc2f0, 0.45);
    fillLight.position.set(-300,200,-500);
    scene.add(fillLight);
    
    const camera=new THREE.PerspectiveCamera(68,el.clientWidth/el.clientHeight,0.5,5000);
    camera.position.set(600,120,600);
    
    const mats=buildMaterials();
    const builtChunks=new Map();
    
    const syncChunks=()=>{
      const world=worldRef.current;if(!world)return;
      const seed=world.seedOffset??0;
      const camX = camera.position.x;
      const camZ = camera.position.z;
      const camChunkX = Math.floor(camX / CHUNK_SIZE);
      const camChunkZ = Math.floor(camZ / CHUNK_SIZE);
      const renderDistance = 2; // 2 chunks away max
      
      // Load visible chunks
      for(const[key,chunk]of world.generatedChunks){
        const dcx = Math.abs(chunk.cx - camChunkX);
        const dcy = Math.abs(chunk.cy - camChunkZ);
        if (dcx <= renderDistance && dcy <= renderDistance) {
          if(!builtChunks.has(key)){
            const g=buildChunkGroup(chunk,mats,seed,world);
            scene.add(g);
            builtChunks.set(key,g);
          }
        }
      }
      
      // Unload distant chunks
      for(const[key,g]of builtChunks.entries()){
        const [cxStr, cyStr] = key.split(',');
        const cx = parseInt(cxStr), cy = parseInt(cyStr);
        const dcx = Math.abs(cx - camChunkX);
        const dcy = Math.abs(cy - camChunkZ);
        if (dcx > renderDistance || dcy > renderDistance) {
          scene.remove(g);
          g.traverse((child) => {
            if(child.geometry) child.geometry.dispose();
            // Don't dispose material since they are shared
          });
          builtChunks.delete(key);
        }
      }
    };
    
    const freeCam=new FreeCam(camera,renderer.domElement);
    const onLC=()=>{
      const locked=document.pointerLockElement===renderer.domElement;
      const hint=hudRef.current?.querySelector(".mouse-hint");
      if(hint)hint.style.display=locked?"none":"block";
    };
    document.addEventListener("pointerlockchange",onLC);
    
    let lastT=performance.now(),fc=0,ft=0,fps=0,aId;
    const loop=()=>{
      aId=requestAnimationFrame(loop);
      const now=performance.now();
      const dt=Math.min((now-lastT)/1000,0.1);
      lastT=now;
      fc++;ft+=dt;if(ft>=0.5){fps=Math.round(fc/ft);fc=0;ft=0;}
      freeCam.update(dt);syncChunks();
      const cx=camera.position.x,cy=camera.position.y,cz=camera.position.z;
      renderer.render(scene,camera);
      
      if(hudRef.current){
        const fe=hudRef.current.querySelector(".fps"),pe=hudRef.current.querySelector(".pos"),ce=hudRef.current.querySelector(".chk");
        if(fe){fe.textContent=fps;fe.style.color=fps>50?"#00ff88":fps>30?"#ffaa00":"#ff4444";}
        if(pe)pe.textContent=cx.toFixed(0)+" · "+cy.toFixed(0)+" · "+cz.toFixed(0);
        if(ce)ce.textContent=builtChunks.size;
      }
    };
    loop();
    
    const onResize=()=>{
      camera.aspect=el.clientWidth/el.clientHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(el.clientWidth,el.clientHeight);
    };
    window.addEventListener("resize",onResize);
    
    return()=>{
      cancelAnimationFrame(aId);
      freeCam.dispose();
      document.removeEventListener("pointerlockchange",onLC);
      window.removeEventListener("resize",onResize);
      renderer.dispose();
      if(document.pointerLockElement)document.exitPointerLock();
      if(el.contains(renderer.domElement))el.removeChild(renderer.domElement);
    };
  },[worldRef]);

  const mono={fontFamily:"'Courier New',monospace"};
  const panel={background:"rgba(8,8,22,0.92)",backdropFilter:"blur(6px)",border:"1px solid",padding:"10px 14px",fontSize:11,lineHeight:"2",letterSpacing:1,...mono};
  return(
    <div style={{position:"absolute",inset:0,zIndex:20,background:"#0a0a18",...mono}}>
      <div ref={mountRef} style={{width:"100%",height:"100%"}}/>
      <button onClick={onClose} style={{position:"absolute",top:14,left:"50%",transform:"translateX(-50%)",padding:"7px 26px",background:"rgba(8,8,22,0.92)",border:"1px solid rgba(0,224,255,0.45)",color:"#00f0ff",fontSize:11,letterSpacing:3,cursor:"pointer",backdropFilter:"blur(5px)",boxShadow:"0 0 18px rgba(0,224,255,0.2)",zIndex:30}}>← 2D MAP</button>
      
      <div ref={hudRef} style={{...panel,position:"absolute",top:14,left:14,borderColor:"rgba(0,224,255,0.25)",color:"#7ea1b5"}}>
        <div style={{color:"#00f0ff",fontWeight:700,letterSpacing:3,marginBottom:2}}>◈ 3D WORLD</div>
        <div>FPS &nbsp;<span className="fps">—</span></div>
        <div>CHUNKS &nbsp;<span className="chk" style={{color:"#fff"}}>0</span></div>
        <div style={{fontSize:10}}>POS &nbsp;<span className="pos" style={{color:"#9ebcd0"}}>—</span></div>
        <div className="mouse-hint" style={{color:"#ffa500",marginTop:4,fontSize:10}}>CLICK canvas to fly</div>
      </div>
      
      <div style={{...panel,position:"absolute",top:14,right:14,borderColor:"rgba(163,0,255,0.3)",color:"#8f9fae"}}>
        <div style={{color:"#b53cff",fontWeight:700,letterSpacing:3,marginBottom:2}}>◈ FLY</div>
        <div><span style={{color:"#ddd"}}>WASD</span>&nbsp;&nbsp;Move</div>
        <div><span style={{color:"#ddd"}}>MOUSE</span>&nbsp;Look</div>
        <div><span style={{color:"#ddd"}}>E / Q</span>&nbsp;&nbsp;Up/Dn</div>
        <div><span style={{color:"#ddd"}}>SCROLL</span>&nbsp;Alt</div>
        <div><span style={{color:"#ddd"}}>SHIFT</span>&nbsp;&nbsp;Fast</div>
        <div><span style={{color:"#ddd"}}>ESC</span>&nbsp;&nbsp;&nbsp;Free</div>
      </div>
      
      <div style={{position:"absolute",top:"50%",left:"50%",transform:"translate(-50%,-50%)",width:16,height:16,pointerEvents:"none"}}>
        <div style={{position:"absolute",top:"50%",left:0,right:0,height:1,background:"rgba(255,255,255,0.45)",transform:"translateY(-50%)"}}/>
        <div style={{position:"absolute",left:"50%",top:0,bottom:0,width:1,background:"rgba(255,255,255,0.45)",transform:"translateX(-50%)"}}/>
      </div>
      
      <div style={{position:"absolute",bottom:14,left:"50%",transform:"translateX(-50%)",color:"rgba(0,224,255,0.25)",fontSize:9,letterSpacing:5,pointerEvents:"none"}}>
        3D VIEW · CYBERPUNK TOKYO SCALE · VOXEL ENGINE
      </div>
    </div>
  );
}

