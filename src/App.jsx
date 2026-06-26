import React, { useEffect, useRef, useState, useCallback } from 'react';
import VoxelView from './VoxelView.jsx';

// --- Cache Math functions as locals for hot-path speed ---
const _sin = Math.sin;
const _cos = Math.cos;
const _floor = Math.floor;
const _hypot = Math.hypot;
const _atan2 = Math.atan2;
const _sqrt = Math.sqrt;
const _abs = Math.abs;
const _min = Math.min;
const _max = Math.max;
const _random = Math.random;
const _PI = Math.PI;
const _PI2 = _PI * 2;
const _PI_2 = _PI / 2;
const _PI_4 = _PI / 4;

// --- Procedural Noise for Terrain ---
const random = (x, y) => {
    let n = _sin(x * 12.9898 + y * 78.233) * 43758.5453123;
    return n - _floor(n);
};

const smoothNoise = (x, y) => {
    const ix = _floor(x); const iy = _floor(y);
    const fx = x - ix; const fy = y - iy;
    const v1 = random(ix, iy);
    const v2 = random(ix + 1, iy);
    const v3 = random(ix, iy + 1);
    const v4 = random(ix + 1, iy + 1);
    const i1 = v1 + (v2 - v1) * fx;   // lerp inlined, no extra multiply
    const i2 = v3 + (v4 - v3) * fx;
    return i1 + (i2 - i1) * fy;
};

const fbm = (x, y, scale = 0.003) => {
    let v = 0, amp = 0.5, f = scale;
    v += smoothNoise(x * f, y * f) * amp; f *= 2; amp *= 0.5;
    v += smoothNoise(x * f, y * f) * amp; f *= 2; amp *= 0.5;
    v += smoothNoise(x * f, y * f) * amp; f *= 2; amp *= 0.5;
    v += smoothNoise(x * f, y * f) * amp;
    return v;
};

// --- OBB Helpers (optimized: reuse temp arrays, avoid allocations) ---
// Reusable corner arrays
const _cA = [{ x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }];
const _cB = [{ x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }, { x: 0, y: 0 }];

const fillCorners = (out, b) => {
    const hw = b.w * 0.5, hh = b.h * 0.5;
    const c = _cos(b.angle), s = _sin(b.angle);
    out[0].x = b.x + c * (-hw) - s * (-hh); out[0].y = b.y + s * (-hw) + c * (-hh);
    out[1].x = b.x + c * hw - s * (-hh); out[1].y = b.y + s * hw + c * (-hh);
    out[2].x = b.x + c * hw - s * hh; out[2].y = b.y + s * hw + c * hh;
    out[3].x = b.x + c * (-hw) - s * hh; out[3].y = b.y + s * (-hw) + c * hh;
};

const getBuildingCorners = (b) => {
    // Returns a new array (used for AABB bounding in grid insertion — called infrequently)
    const hw = b.w * 0.5, hh = b.h * 0.5;
    const c = _cos(b.angle), s = _sin(b.angle);
    return [
        { x: b.x + c * (-hw) - s * (-hh), y: b.y + s * (-hw) + c * (-hh) },
        { x: b.x + c * hw - s * (-hh), y: b.y + s * hw + c * (-hh) },
        { x: b.x + c * hw - s * hh, y: b.y + s * hw + c * hh },
        { x: b.x + c * (-hw) - s * hh, y: b.y + s * (-hw) + c * hh },
    ];
};

const projectPoly = (corners, ax, ay) => {
    let min = Infinity, max = -Infinity;
    for (let i = 0; i < 4; i++) {
        const p = corners[i].x * ax + corners[i].y * ay;
        if (p < min) min = p;
        if (p > max) max = p;
    }
    return min; // Return min; caller computes max by negating and calling again? No — pack both into an array.
    // Actually keep returning [min,max] but avoid allocation with a shared buffer:
};

// Shared projection result buffers
const _pA = new Float64Array(2);
const _pB = new Float64Array(2);

const projectPolyInto = (corners, ax, ay, out) => {
    let min = Infinity, max = -Infinity;
    for (let i = 0; i < 4; i++) {
        const p = corners[i].x * ax + corners[i].y * ay;
        if (p < min) min = p;
        if (p > max) max = p;
    }
    out[0] = min; out[1] = max;
};

const obbVsObb = (a, b, padding = 0) => {
    fillCorners(_cA, a);
    fillCorners(_cB, b);
    const cosA = _cos(a.angle), sinA = _sin(a.angle);
    const cosB = _cos(b.angle), sinB = _sin(b.angle);
    // 4 axes
    const axes = [
        [cosA, sinA],
        [-sinA, cosA],
        [cosB, sinB],
        [-sinB, cosB],
    ];
    for (let i = 0; i < 4; i++) {
        const ax = axes[i][0], ay = axes[i][1];
        projectPolyInto(_cA, ax, ay, _pA);
        projectPolyInto(_cB, ax, ay, _pB);
        if (!(_pA[0] - padding < _pB[1] && _pB[0] - padding < _pA[1])) return false;
    }
    return true;
};

// Reusable OBB object for road segment tests
const _roadObb = { x: 0, y: 0, w: 0, h: 0, angle: 0 };

const roadSegmentToObbInPlace = (rx1, ry1, rx2, ry2, halfWidth, out) => {
    const dx = rx2 - rx1, dy = ry2 - ry1;
    const len = _hypot(dx, dy);
    if (len < 0.001) return false;
    out.x = (rx1 + rx2) * 0.5; out.y = (ry1 + ry2) * 0.5;
    out.w = len + halfWidth * 2; out.h = halfWidth * 2;
    out.angle = _atan2(dy, dx);
    return true;
};

const roadSegmentToObb = (rx1, ry1, rx2, ry2, halfWidth) => {
    const dx = rx2 - rx1, dy = ry2 - ry1;
    const len = _hypot(dx, dy);
    if (len < 0.001) return null;
    return { x: (rx1 + rx2) * 0.5, y: (ry1 + ry2) * 0.5, w: len + halfWidth * 2, h: halfWidth * 2, angle: _atan2(dy, dx) };
};

const lineIntersect = (p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y) => {
    if ((_abs(p0_x - p2_x) < 0.1 && _abs(p0_y - p2_y) < 0.1) ||
        (_abs(p0_x - p3_x) < 0.1 && _abs(p0_y - p3_y) < 0.1) ||
        (_abs(p1_x - p2_x) < 0.1 && _abs(p1_y - p2_y) < 0.1) ||
        (_abs(p1_x - p3_x) < 0.1 && _abs(p1_y - p3_y) < 0.1)) return false;
    const s1_x = p1_x - p0_x, s1_y = p1_y - p0_y;
    const s2_x = p3_x - p2_x, s2_y = p3_y - p2_y;
    const denom = -s2_x * s1_y + s1_x * s2_y;
    if (_abs(denom) < 0.0001) return false;
    const s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denom;
    const t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denom;
    return s > 0.01 && s < 0.99 && t > 0.01 && t < 0.99;
};

// --- Constants & Config ---
const CHUNK_SIZE = 1200;
const STEP_SIZE = 15;
const ALLEY_STEP = 10;
const MERGE_RADIUS = 12;
const GROWTH_SPEED = 20;
const CELL_SIZE = 30;
const BLDG_CELL = 32;
const ROAD_CELL = 100;

const WATER_LVL = 0.35;
const CITY_LVL = 0.52;
const SUBURB_LVL = 0.65;

// Pre-compute reciprocals for grid key math
const INV_CELL_SIZE = 1 / CELL_SIZE;
const INV_BLDG_CELL = 1 / BLDG_CELL;
const INV_ROAD_CELL = 1 / ROAD_CELL;

// Terrain string constants reused as references
const T_WATER = 'WATER';
const T_CITY = 'CITY';
const T_SUBURB = 'SUBURB';
const T_PARK = 'PARK';

export default function App() {
    const canvasRef = useRef(null);
    const wrapperRef = useRef(null);
    const requestRef = useRef(null);

    const cameraRef = useRef({ x: 0, y: 0 });
    const zoomRef = useRef(1.0);
    const [zoomState, setZoomState] = useState(1.0);
    const dragRef = useRef({ isDragging: false, lastX: 0, lastY: 0 });

    const worldRef = useRef({
        seedOffset: _random() * 10000,
        chunkQueue: [],
        generatedChunks: new Map(),
        activeChunk: null,
        terrainCache: new Map(),
        globalStats: { nodes: 0, edges: 0, buildings: 0, parking: 0, fences: 0, nodeCounter: 0 },
        borderNodes: []
    });

    const [isRunning, setIsRunning] = useState(true);
    const [view3D, setView3D] = useState(false);
    const [uiStats, setUiStats] = useState({
        phase: 'Initializing', activeIsland: null, queued: 0,
        nodes: 0, edges: 0, buildings: 0, parking: 0, fences: 0
    });

    // --- Inline getCellKey to avoid string template overhead in hot paths ---
    const getCellKey = (x, y) => `${_floor(x * INV_CELL_SIZE)},${_floor(y * INV_CELL_SIZE)}`;

    const addNode = (chunk, x, y, z = 0) => {
        const id = worldRef.current.globalStats.nodeCounter++;
        const node = { id, x, y, z };
        chunk.nodes.push(node);
        const key = getCellKey(x, y);
        let cell = chunk.spatialGrid.get(key);
        if (!cell) { cell = []; chunk.spatialGrid.set(key, cell); }
        cell.push(node);
        return node;
    };

    const findNearestNode = (chunk, x, y, radius, excludeNode, targetZ) => {
        const cx = _floor(x * INV_CELL_SIZE);
        const cy = _floor(y * INV_CELL_SIZE);
        const searchCells = Math.ceil(radius * INV_CELL_SIZE);
        let nearest = null;
        let minDist = radius * radius;
        const grid = chunk.spatialGrid;
        for (let i = -searchCells; i <= searchCells; i++) {
            for (let j = -searchCells; j <= searchCells; j++) {
                const cell = grid.get(`${cx + i},${cy + j}`);
                if (cell) {
                    for (let k = 0; k < cell.length; k++) {
                        const node = cell[k];
                        if (node === excludeNode || node.z !== targetZ) continue;
                        const ddx = node.x - x, ddy = node.y - y;
                        const distSq = ddx * ddx + ddy * ddy;
                        if (distSq < minDist) { minDist = distSq; nearest = node; }
                    }
                }
            }
        }
        return nearest;
    };

    const getTerrain = (worldX, worldY, offset) => {
        const cx = _floor(worldX / CHUNK_SIZE);
        const cy = _floor(worldY / CHUNK_SIZE);
        const centerX = cx * CHUNK_SIZE + 600; // CHUNK_SIZE/2 = 600
        const centerY = cy * CHUNK_SIZE + 600;

        const warpX1 = (fbm(worldX + offset, worldY + offset, 0.001) - 0.45) * 550;
        const warpY1 = (fbm(worldX + offset + 500, worldY + offset + 500, 0.001) - 0.45) * 550;
        const warpX2 = (fbm(worldX + offset + 1000, worldY + offset + 1000, 0.005) - 0.45) * 150;
        const warpY2 = (fbm(worldX + offset + 1500, worldY + offset + 1500, 0.005) - 0.45) * 150;

        const warpedX = worldX - (warpX1 + warpX2);
        const warpedY = worldY - (warpY1 + warpY2);

        const wdx = warpedX - centerX, wdy = warpedY - centerY;
        const warpedDist = _sqrt(wdx * wdx + wdy * wdy); // avoid hypot overhead

        const val = fbm(warpedX + offset, warpedY + offset);

        const maxRadius = CHUNK_SIZE * 0.48;
        const falloffStart = CHUNK_SIZE * 0.22;

        let finalVal = val;
        if (warpedDist > falloffStart) {
            finalVal -= ((warpedDist - falloffStart) / (maxRadius - falloffStart)) * 2.0;
        } else if (warpedDist < falloffStart * 0.5) {
            if (finalVal < WATER_LVL + 0.05) finalVal = WATER_LVL + 0.05;
        }

        if (finalVal < WATER_LVL) return T_WATER;
        if (finalVal < CITY_LVL) return T_CITY;
        if (finalVal < SUBURB_LVL) return T_SUBURB;
        return T_PARK;
    };

    const checkWaterCrossing = (n1, n2, offset) => {
        const dx = n2.x - n1.x, dy = n2.y - n1.y;
        if (getTerrain(n1.x + dx * 0.25, n1.y + dy * 0.25, offset) === T_WATER) return true;
        if (getTerrain(n1.x + dx * 0.5, n1.y + dy * 0.5, offset) === T_WATER) return true;
        if (getTerrain(n1.x + dx * 0.75, n1.y + dy * 0.75, offset) === T_WATER) return true;
        return false;
    };

    const isInvalidRampConnection = (chunk, node) => {
        const edges = chunk.edges;
        const id = node.id;
        for (let i = 0; i < edges.length; i++) {
            const e = edges[i];
            if (e.n1.id === id || e.n2.id === id) {
                if (e.type === 'alley' || e.type === 'park_path') return true;
            }
        }
        return false;
    };

    const isRampNode = (chunk, node) => {
        const edges = chunk.edges;
        const id = node.id;
        for (let i = 0; i < edges.length; i++) {
            const e = edges[i];
            if (e.n1.id === id || e.n2.id === id) {
                if (e.type === 'ramp' || e.type === 'highway') return true;
            }
        }
        return false;
    };

    const addBuildingToGrid = (chunk, b) => {
        const corners = getBuildingCorners(b);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (let i = 0; i < 4; i++) {
            const c = corners[i];
            if (c.x < minX) minX = c.x; if (c.x > maxX) maxX = c.x;
            if (c.y < minY) minY = c.y; if (c.y > maxY) maxY = c.y;
        }
        const x0 = _floor(minX * INV_BLDG_CELL), x1 = _floor(maxX * INV_BLDG_CELL);
        const y0 = _floor(minY * INV_BLDG_CELL), y1 = _floor(maxY * INV_BLDG_CELL);
        const grid = chunk.buildingGrid;
        for (let gx = x0; gx <= x1; gx++) {
            for (let gy = y0; gy <= y1; gy++) {
                const key = `${gx},${gy}`;
                let cell = grid.get(key);
                if (!cell) { cell = []; grid.set(key, cell); }
                cell.push(b);
            }
        }
    };

    const getNearbyBuildings = (chunk, b) => {
        const corners = getBuildingCorners(b);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (let i = 0; i < 4; i++) {
            const c = corners[i];
            if (c.x < minX) minX = c.x; if (c.x > maxX) maxX = c.x;
            if (c.y < minY) minY = c.y; if (c.y > maxY) maxY = c.y;
        }
        const x0 = _floor(minX * INV_BLDG_CELL) - 1, x1 = _floor(maxX * INV_BLDG_CELL) + 1;
        const y0 = _floor(minY * INV_BLDG_CELL) - 1, y1 = _floor(maxY * INV_BLDG_CELL) + 1;
        const seen = new Set(), result = [];
        const grid = chunk.buildingGrid;
        for (let gx = x0; gx <= x1; gx++) {
            for (let gy = y0; gy <= y1; gy++) {
                const cell = grid.get(`${gx},${gy}`);
                if (cell) {
                    for (let i = 0; i < cell.length; i++) {
                        const nb = cell[i];
                        if (!seen.has(nb)) { seen.add(nb); result.push(nb); }
                    }
                }
            }
        }
        return result;
    };

    const buildRoadGrid = (edges) => {
        const grid = new Map();
        for (let ei = 0; ei < edges.length; ei++) {
            const e = edges[ei];
            const ddx = e.n2.x - e.n1.x, ddy = e.n2.y - e.n1.y;
            const steps = Math.ceil(_hypot(ddx, ddy) / ROAD_CELL) + 1;
            const seen = new Set();
            for (let s = 0; s <= steps; s++) {
                const t = s / steps;
                const mx = e.n1.x + ddx * t;
                const my = e.n1.y + ddy * t;
                const key = `${_floor(mx * INV_ROAD_CELL)},${_floor(my * INV_ROAD_CELL)}`;
                if (!seen.has(key)) {
                    seen.add(key);
                    let cell = grid.get(key);
                    if (!cell) { cell = []; grid.set(key, cell); }
                    cell.push(e);
                }
            }
        }
        return grid;
    };

    const getNearbyEdges = (chunk, b) => {
        const corners = getBuildingCorners(b);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (let i = 0; i < 4; i++) {
            const c = corners[i];
            if (c.x < minX) minX = c.x; if (c.x > maxX) maxX = c.x;
            if (c.y < minY) minY = c.y; if (c.y > maxY) maxY = c.y;
        }
        const x0 = _floor(minX * INV_ROAD_CELL) - 1, x1 = _floor(maxX * INV_ROAD_CELL) + 1;
        const y0 = _floor(minY * INV_ROAD_CELL) - 1, y1 = _floor(maxY * INV_ROAD_CELL) + 1;
        const seen = new Set(), result = [];
        const grid = chunk.roadGrid;
        for (let gx = x0; gx <= x1; gx++) {
            for (let gy = y0; gy <= y1; gy++) {
                const cell = grid.get(`${gx},${gy}`);
                if (cell) {
                    for (let i = 0; i < cell.length; i++) {
                        const e = cell[i];
                        if (!seen.has(e)) { seen.add(e); result.push(e); }
                    }
                }
            }
        }
        return result;
    };

    const initChunkData = (cx, cy) => {
        const chunk = {
            key: `${cx},${cy}`, cx, cy,
            phase: 'GROWING',
            agents: [], nodes: [], edges: [],
            spatialGrid: new Map(), buildingGrid: new Map(),
            buildings: [], roadGrid: null, fences: [],
            causewayTargets: [],
            edgeProcessIndex: 0, fenceProcessIndex: 0
        };
        const minX = cx * CHUNK_SIZE, maxX = minX + CHUNK_SIZE;
        const minY = cy * CHUNK_SIZE, maxY = minY + CHUNK_SIZE;

        let startedFromBorder = false;
        const borderNodes = worldRef.current.borderNodes;

        for (let i = borderNodes.length - 1; i >= 0; i--) {
            const bn = borderNodes[i];
            if (bn.x >= minX - 1 && bn.x <= maxX + 1 && bn.y >= minY - 1 && bn.y <= maxY + 1) {
                const newNode = addNode(chunk, bn.x, bn.y, bn.z);
                const agentLife = bn.type === 'highway' ? 1800 : 120;
                chunk.agents.push({
                    node: newNode, angle: bn.angle, type: bn.type, life: agentLife, z: bn.z,
                    wasBridge: bn.wasBridge || false, isCardinal: bn.isCardinal || false,
                    hasSpawnedPerpendiculars: false,
                    isArrivingBridge: bn.isArrivingBridge || false
                });
                borderNodes.splice(i, 1);
                startedFromBorder = true;
            }
        }

        if (!startedFromBorder) {
            if (cx === 0 && cy === 0) {
                const centerX = 600, centerY = 600; // cx=0,cy=0 so CHUNK_SIZE/2
                const centerNode = addNode(chunk, centerX, centerY, 1);
                for (let i = 0; i < 4; i++) {
                    chunk.agents.push({ node: centerNode, angle: i * _PI_2, type: 'highway', life: 1800, z: 1, wasBridge: false, isCardinal: true, hasSpawnedPerpendiculars: true });
                }
                for (let i = 0; i < 4; i++) {
                    chunk.agents.push({ node: centerNode, angle: i * _PI_2 + _PI_4, type: 'highway', life: 120, z: 1, wasBridge: false, isCardinal: false });
                }
                chunk.agents.push({ node: centerNode, angle: 0, type: 'ramp', life: 5, z: 0 });
                chunk.agents.push({ node: centerNode, angle: _PI, type: 'ramp', life: 5, z: 0 });
            } else {
                return null;
            }
        }
        return chunk;
    };

    const generateChunkTerrain = (cx, cy) => {
        const offset = worldRef.current.seedOffset;
        const tCanvas = document.createElement('canvas');
        tCanvas.width = CHUNK_SIZE;
        tCanvas.height = CHUNK_SIZE;
        const tCtx = tCanvas.getContext('2d');
        const imgData = tCtx.createImageData(CHUNK_SIZE, CHUNK_SIZE);
        const data = imgData.data;

        const STEP = 4;
        for (let y = 0; y < CHUNK_SIZE; y += STEP) {
            for (let x = 0; x < CHUNK_SIZE; x += STEP) {
                const worldX = cx * CHUNK_SIZE + x;
                const worldY = cy * CHUNK_SIZE + y;
                const type = getTerrain(worldX, worldY, offset);

                let r, g, b;
                if (type === T_WATER) { r = 186; g = 230; b = 253; }
                else if (type === T_CITY) { r = 226; g = 232; b = 240; }
                else if (type === T_SUBURB) { r = 248; g = 250; b = 252; }
                else { r = 220; g = 252; b = 231; }

                const endY = _min(y + STEP, CHUNK_SIZE);
                const endX = _min(x + STEP, CHUNK_SIZE);
                for (let dy = y; dy < endY; dy++) {
                    const rowBase = dy * CHUNK_SIZE;
                    for (let dx = x; dx < endX; dx++) {
                        const i = (rowBase + dx) * 4;
                        data[i] = r; data[i + 1] = g; data[i + 2] = b; data[i + 3] = 255;
                    }
                }
            }
        }
        tCtx.putImageData(imgData, 0, 0);
        worldRef.current.terrainCache.set(`${cx},${cy}`, tCanvas);
    };

    const initWorld = useCallback(() => {
        const wrapper = wrapperRef.current;
        if (!wrapper) return;
        const width = wrapper.clientWidth;
        const height = wrapper.clientHeight;

        cameraRef.current = { x: -(width / 2) + 600, y: -(height / 2) + 600 };

        worldRef.current = {
            seedOffset: _random() * 10000,
            chunkQueue: ['0,0'],
            generatedChunks: new Map(),
            activeChunk: null,
            terrainCache: new Map(),
            globalStats: { nodes: 0, edges: 0, buildings: 0, parking: 0, fences: 0, nodeCounter: 0 },
            borderNodes: []
        };

        setUiStats(prev => ({ ...prev, phase: 'Waking up...' }));
    }, []);

    const tryPlaceBuildingsAlongEdge = (chunk, edge, seedOffset) => {
        const eType = edge.type;
        if (edge.isBridge || eType === 'coast' || eType === 'alley' || eType === 'highway' || eType === 'ramp') return;

        const dx = edge.n2.x - edge.n1.x, dy = edge.n2.y - edge.n1.y;
        const len = _hypot(dx, dy);
        if (len < 5) return;

        const invLen = 1 / len;
        const nx = -dy * invLen, ny = dx * invLen;
        const angle = _atan2(dy, dx);

        const minX = chunk.cx * CHUNK_SIZE, maxX = minX + CHUNK_SIZE;
        const minY = chunk.cy * CHUNK_SIZE, maxY = minY + CHUNK_SIZE;

        let roadHalfWidth = 1.5, sidewalk = 1.5;
        if (eType === 'ramp') { roadHalfWidth = 2; sidewalk = 2; }
        if (eType === 'park_path') { roadHalfWidth = 0.5; sidewalk = 0.5; }

        for (let dir = -1; dir <= 1; dir += 2) {
            let currentT = 1;
            const endT = len - 1;

            while (currentT < endT - 1) {
                const sampleX = edge.n1.x + (dx * invLen) * currentT + nx * dir * 15;
                const sampleY = edge.n1.y + (dy * invLen) * currentT + ny * dir * 15;
                let terrain = getTerrain(sampleX, sampleY, seedOffset);
                if (eType === 'park_path') terrain = T_PARK;

                let bWidth, bDepth, gap, type;
                if (terrain === T_WATER) { currentT += 5; continue; }
                else if (terrain === T_CITY) {
                    bWidth = 3 + _random() * 6; bDepth = 4 + _random() * 7; gap = 0.2;
                    type = _random() < 0.15 ? 'PARKING_LOT' : 'COMMERCIAL';
                } else if (terrain === T_SUBURB) {
                    bWidth = 3 + _random() * 3; bDepth = 4 + _random() * 4; gap = 0.8;
                    type = 'HOUSE';
                } else {
                    bWidth = 2 + _random() * 3; bDepth = bWidth; gap = 2;
                    type = 'TREE';
                }

                if (currentT + bWidth > endT) bWidth = endT - currentT;
                if (bWidth < 2) { currentT += 1; continue; }

                const distToCenter = roadHalfWidth + sidewalk + (bDepth * 0.5);
                const bx = edge.n1.x + (dx * invLen) * (currentT + bWidth * 0.5) + nx * dir * distToCenter;
                const by = edge.n1.y + (dy * invLen) * (currentT + bWidth * 0.5) + ny * dir * distToCenter;

                if (bx <= minX || bx >= maxX || by <= minY || by >= maxY) { currentT += bWidth; continue; }

                const generatedParts = [];
                const basePart = { x: bx, y: by, w: bWidth, h: bDepth, angle, type };

                if (type === 'PARKING_LOT') {
                    const cars = [];
                    const rows = _floor((bDepth - 2) / 3);
                    const cols = _floor((bWidth - 2) / 2.5);
                    const startLX = -((cols - 1) * 2.5) * 0.5;
                    const startLY = -((rows - 1) * 3) * 0.5;
                    for (let r = 0; r < rows; r++) {
                        for (let c = 0; c < cols; c++) {
                            if (_random() < 0.6) {
                                cars.push({
                                    lx: startLX + c * 2.5, ly: startLY + r * 3,
                                    color: ['#ef4444', '#3b82f6', '#10b981', '#f59e0b', '#94a3b8', '#ffffff'][_floor(_random() * 6)]
                                });
                            }
                        }
                    }
                    basePart.cars = cars;
                    generatedParts.push(basePart);
                } else {
                    generatedParts.push(basePart);
                    if (type === 'COMMERCIAL' && _random() < 0.3) {
                        const wingW = 2 + _random() * 3;
                        const wingH = bDepth * (0.4 + _random() * 0.4);
                        const lx = (bWidth * 0.5 + wingW * 0.5) * (_random() > 0.5 ? 1 : -1);
                        const ly = (bDepth * 0.5 - wingH * 0.5) * (_random() > 0.5 ? 1 : -1);
                        generatedParts.push({
                            x: bx + _cos(angle) * lx - _sin(angle) * ly,
                            y: by + _sin(angle) * lx + _cos(angle) * ly,
                            w: wingW, h: wingH, angle, type
                        });
                    }
                }

                let overlap = false;
                for (let pi = 0; pi < generatedParts.length; pi++) {
                    const part = generatedParts[pi];
                    const nearbyEdges = getNearbyEdges(chunk, part);
                    for (let ei = 0; ei < nearbyEdges.length; ei++) {
                        const e = nearbyEdges[ei];
                        let hw = 1.5 + sidewalk;
                        if (e.type === 'highway') hw = 3 + sidewalk;
                        else if (e.type === 'park_path') hw = 0.5 + sidewalk;
                        else if (e.type === 'alley') hw = 0.5;
                        const rObb = roadSegmentToObb(e.n1.x, e.n1.y, e.n2.x, e.n2.y, hw);
                        if (rObb && obbVsObb(part, rObb, 0)) { overlap = true; break; }
                    }
                    if (overlap) break;

                    const pad = part.type === 'TREE' ? 0.2 : 0.1;
                    const nearbyBuildings = getNearbyBuildings(chunk, part);
                    for (let bi = 0; bi < nearbyBuildings.length; bi++) {
                        if (obbVsObb(part, nearbyBuildings[bi], pad)) { overlap = true; break; }
                    }
                    if (overlap) break;
                }

                if (!overlap) {
                    for (let pi = 0; pi < generatedParts.length; pi++) {
                        const part = generatedParts[pi];
                        chunk.buildings.push(part);
                        addBuildingToGrid(chunk, part);
                    }
                    currentT += bWidth + gap;
                } else {
                    currentT += 0.5;
                }
            }
        }
    };

    const stepSimulation = useCallback(() => {
        const world = worldRef.current;

        if (!world.activeChunk) {
            if (world.chunkQueue.length > 0) {
                const nextKey = world.chunkQueue.shift();
                const comma = nextKey.indexOf(',');
                const cx = +nextKey.slice(0, comma);
                const cy = +nextKey.slice(comma + 1);
                if (!world.generatedChunks.has(nextKey)) {
                    const newChunk = initChunkData(cx, cy);
                    if (newChunk) world.activeChunk = newChunk;
                }
            }
            return;
        }

        const chunk = world.activeChunk;
        const seedOffset = world.seedOffset;

        const minX = chunk.cx * CHUNK_SIZE, maxX = minX + CHUNK_SIZE;
        const minY = chunk.cy * CHUNK_SIZE, maxY = minY + CHUNK_SIZE;

        if (chunk.phase === 'GROWING') {
            const agents = chunk.agents;
            if (agents.length < 300) {
                for (let tries = 0; tries < 6; tries++) {
                    const rx = minX + _random() * CHUNK_SIZE;
                    const ry = minY + _random() * CHUNK_SIZE;
                    const t = getTerrain(rx, ry, seedOffset);
                    if (t === T_CITY || t === T_SUBURB) {
                        const nearest = findNearestNode(chunk, rx, ry, 50, null, 0);
                        if (!nearest) {
                            const newNode = addNode(chunk, rx, ry, 0);
                            for (let i = 0; i < 4; i++) {
                                agents.push({ node: newNode, angle: i * _PI_2 + _random(), type: 'street', life: 120, z: 0 });
                            }
                        }
                    }
                }
            }

            let iterations = 0;
            while (agents.length > 0 && iterations < GROWTH_SPEED) {
                iterations++;
                const idx = _floor(_random() * agents.length);
                const agent = agents[idx];
                agent.life--;

                let forceMerge = agent.life <= 0;

                if (forceMerge && agent.type === 'ramp') {
                    agent.type = 'street';
                    agent.life = 100;
                    forceMerge = false;
                    agents.push({ node: agent.node, angle: agent.angle + _PI_2, type: 'street', life: 80, z: 0 });
                    agents.push({ node: agent.node, angle: agent.angle - _PI_2, type: 'street', life: 80, z: 0 });
                    const cityNode = findNearestNode(chunk, agent.node.x, agent.node.y, 120, agent.node, 0);
                    if (cityNode) chunk.edges.push({ n1: agent.node, n2: cityNode, type: 'street', isBridge: false });
                }

                const stepAmount = agent.type === 'alley' ? ALLEY_STEP : STEP_SIZE;
                let nx = agent.node.x + _cos(agent.angle) * stepAmount;
                let ny = agent.node.y + _sin(agent.angle) * stepAmount;

                const isOutOfBounds = nx < minX || nx >= maxX || ny < minY || ny >= maxY;
                if (!forceMerge && isOutOfBounds) {
                    if (agent.type === 'highway') {
                        const zLevel = agent.z || 0;
                        const boundTerrain = getTerrain(nx, ny, seedOffset);
                        const isOverWater = boundTerrain === T_WATER;

                        const nextNode = addNode(chunk, nx, ny, zLevel);
                        chunk.edges.push({ n1: agent.node, n2: nextNode, type: agent.type, isBridge: agent.wasBridge || isOverWater });

                        let nextCx = chunk.cx, nextCy = chunk.cy;
                        if (nx < minX) nextCx--; else if (nx >= maxX) nextCx++;
                        if (ny < minY) nextCy--; else if (ny >= maxY) nextCy++;
                        const nextKey = `${nextCx},${nextCy}`;

                        if (world.generatedChunks.has(nextKey)) {
                            const targetChunk = world.generatedChunks.get(nextKey);
                            const targetNode = findNearestNode(targetChunk, nx, ny, 1000, null, zLevel);
                            if (targetNode) chunk.edges.push({ n1: nextNode, n2: targetNode, type: 'highway', isBridge: isOverWater });
                        } else {
                            world.borderNodes.push({ x: nx, y: ny, angle: agent.angle, type: agent.type, z: zLevel, wasBridge: isOverWater, isCardinal: agent.isCardinal, hasSpawnedPerpendiculars: false, isArrivingBridge: isOverWater });
                            if (!world.chunkQueue.includes(nextKey)) world.chunkQueue.push(nextKey);
                        }
                    }
                    agents.splice(idx, 1);
                    continue;
                }

                const nextTerrain = getTerrain(nx, ny, seedOffset);
                let isBridge = false;

                if (!forceMerge && agent.type === 'causeway' && nextTerrain !== T_WATER) {
                    agent.type = 'street';
                    agent.life = 60;
                } else if (!forceMerge && agent.type === 'street' && nextTerrain === T_SUBURB) {
                    agent.type = 'suburb_road';
                } else if (!forceMerge && agent.type === 'suburb_road' && nextTerrain === T_CITY) {
                    agent.type = 'street';
                }

                if (!forceMerge && nextTerrain === T_WATER) {
                    if (agent.type === 'ramp') { agents.splice(idx, 1); continue; }
                    else if (agent.type === 'highway' || agent.type === 'causeway') { isBridge = true; agent.wasBridge = true; }
                    else if (agent.type === 'alley') { forceMerge = true; }
                    else if (agent.type === 'street' || agent.type === 'suburb_road' || agent.type === 'coast' || agent.type === 'park_path') {
                        let hitLand = false, hasCity = false;
                        if (agent.type === 'street' || agent.type === 'suburb_road') {
                            const cosA = _cos(agent.angle), sinA = _sin(agent.angle);
                            for (let i = 1; i <= 180; i++) {
                                const rx = agent.node.x + cosA * (STEP_SIZE * i);
                                const ry = agent.node.y + sinA * (STEP_SIZE * i);
                                if (rx < minX || rx >= maxX || ry < minY || ry >= maxY) break;
                                const terrain = getTerrain(rx, ry, seedOffset);
                                if (terrain !== T_WATER) {
                                    hitLand = true;
                                    if (terrain === T_CITY || terrain === T_SUBURB) hasCity = true;
                                    break;
                                }
                            }
                        }
                        if (hitLand && hasCity) {
                            let alreadyExists = false;
                            const ct = chunk.causewayTargets;
                            const anx = agent.node.x, any = agent.node.y;
                            for (let ci = 0; ci < ct.length; ci++) {
                                const cdx = ct[ci].x - anx, cdy = ct[ci].y - any;
                                if (cdx * cdx + cdy * cdy < 14400) { alreadyExists = true; break; } // 120^2
                            }
                            if (!alreadyExists) {
                                ct.push({ x: anx, y: any });
                                agents.push({ node: agent.node, angle: agent.angle, type: 'causeway', life: 100, z: 0, wasBridge: true });
                            }
                            forceMerge = true;
                        } else {
                            agent.type = 'coast';
                            const eps = 2;
                            const sx = agent.node.x + seedOffset, sy = agent.node.y + seedOffset;
                            const vRight = fbm(sx + eps, sy);
                            const vTop = fbm(sx, sy + eps);
                            const vCenter = fbm(sx, sy);
                            let coastAngle = _atan2(-(vRight - vCenter), vTop - vCenter);
                            let diff = coastAngle - agent.angle;
                            while (diff <= -_PI) diff += _PI2;
                            while (diff > _PI) diff -= _PI2;
                            if (_abs(diff) > _PI_2) coastAngle += _PI;
                            agent.angle = coastAngle;
                            nx = agent.node.x + _cos(agent.angle) * (STEP_SIZE * 0.8);
                            ny = agent.node.y + _sin(agent.angle) * (STEP_SIZE * 0.8);
                            if (getTerrain(nx, ny, seedOffset) === T_WATER || nx < minX || nx > maxX || ny < minY || ny > maxY) forceMerge = true;
                            else agent.life = _max(agent.life, 30);
                        }
                    }
                } else if (!forceMerge && nextTerrain === T_PARK) {
                    if (agent.type === 'street' || agent.type === 'suburb_road') agent.type = 'park_path';
                    if (agent.type === 'alley') forceMerge = true;
                } else if (!forceMerge && agent.type === 'alley' && nextTerrain !== T_CITY) {
                    forceMerge = true;
                }

                const zLevel = agent.z || 0;

                if (forceMerge) {
                    const mergeDist = agent.type === 'alley' ? 40 : agent.type === 'ramp' ? 80 : 300;
                    let target = findNearestNode(chunk, agent.node.x, agent.node.y, mergeDist, agent.node, zLevel);

                    if (target && agent.type === 'ramp') {
                        const tt = getTerrain(target.x, target.y, seedOffset);
                        if (tt === T_PARK || tt === T_WATER || isInvalidRampConnection(chunk, target)) target = null;
                    }
                    if (target && (agent.type === 'alley' || agent.type === 'park_path')) {
                        if (isRampNode(chunk, target)) target = null;
                    }
                    if (target && (agent.type === 'highway' || agent.type === 'causeway' || !checkWaterCrossing(agent.node, target, seedOffset))) {
                        chunk.edges.push({ n1: agent.node, n2: target, type: agent.type, isBridge: (agent.type === 'highway' || agent.type === 'causeway') ? isBridge : false });
                    }
                    agents.splice(idx, 1);
                    continue;
                }

                const mergeSens = agent.type === 'alley' ? MERGE_RADIUS * 0.8 : MERGE_RADIUS;
                let nearbyNode = isBridge ? null : findNearestNode(chunk, nx, ny, mergeSens, agent.node, zLevel);

                if (nearbyNode && agent.type === 'ramp') {
                    const tt = getTerrain(nearbyNode.x, nearbyNode.y, seedOffset);
                    if (tt === T_PARK || tt === T_WATER || isInvalidRampConnection(chunk, nearbyNode)) nearbyNode = null;
                }
                if (nearbyNode && (agent.type === 'alley' || agent.type === 'park_path')) {
                    if (isRampNode(chunk, nearbyNode)) nearbyNode = null;
                }
                if (nearbyNode && agent.type !== 'highway' && checkWaterCrossing(agent.node, nearbyNode, seedOffset)) {
                    agents.splice(idx, 1); continue;
                }

                const nextNode = nearbyNode || addNode(chunk, nx, ny, zLevel);
                chunk.edges.push({ n1: agent.node, n2: nextNode, type: agent.type, isBridge: (agent.type === 'highway' || agent.type === 'causeway') ? isBridge : false });
                if (nearbyNode) { agents.splice(idx, 1); continue; }

                agent.node = nextNode;

                if (agent.type === 'highway') {
                    if (agent.isCardinal && !agent.hasSpawnedPerpendiculars) {
                        const centerX = chunk.cx * CHUNK_SIZE + 600;
                        const centerY = chunk.cy * CHUNK_SIZE + 600;
                        const ddx = agent.node.x - centerX, ddy = agent.node.y - centerY;
                        const distToCenter = _sqrt(ddx * ddx + ddy * ddy);

                        if (distToCenter < STEP_SIZE * 2) {
                            agent.hasSpawnedPerpendiculars = true;
                            agents.push({ node: agent.node, angle: agent.angle + _PI_2, type: 'highway', life: 1800, z: agent.z, wasBridge: false, isCardinal: true, hasSpawnedPerpendiculars: true });
                            agents.push({ node: agent.node, angle: agent.angle - _PI_2, type: 'highway', life: 1800, z: agent.z, wasBridge: false, isCardinal: true, hasSpawnedPerpendiculars: true });
                            agents.push({ node: agent.node, angle: agent.angle + _PI_4, type: 'highway', life: 120, z: agent.z, wasBridge: false, isCardinal: false });
                            agents.push({ node: agent.node, angle: agent.angle - _PI_4, type: 'highway', life: 120, z: agent.z, wasBridge: false, isCardinal: false });
                            agents.push({ node: agent.node, angle: agent.angle + _PI_2, type: 'ramp', life: 5, z: 0 });
                            agents.push({ node: agent.node, angle: agent.angle - _PI_2, type: 'ramp', life: 5, z: 0 });
                        }
                    }

                    if (agent.isCardinal) {
                        if (!isBridge && _random() < 0.05) {
                            agents.push({ node: nextNode, angle: agent.angle + (_random() > 0.5 ? _PI_2 : -_PI_2), type: 'ramp', life: 4, z: 0 });
                        }
                    } else {
                        agent.angle += (_random() - 0.5) * (isBridge ? 0.0 : 0.3);
                    }

                    if (agent.isArrivingBridge && isBridge) {
                        const centerX = chunk.cx * CHUNK_SIZE + 600;
                        const centerY = chunk.cy * CHUNK_SIZE + 600;
                        const targetAngle = _atan2(centerY - ny, centerX - nx);
                        let diff = targetAngle - agent.angle;
                        while (diff > _PI) diff -= _PI2;
                        while (diff < -_PI) diff += _PI2;
                        agent.angle += diff * 0.15;
                    }
                    if (!isBridge) agent.isArrivingBridge = false;

                    if (agent.wasBridge && !isBridge) {
                        let cityNode = findNearestNode(chunk, nx, ny, 150, nextNode, zLevel);
                        if (cityNode && isInvalidRampConnection(chunk, cityNode)) cityNode = null;
                        if (cityNode) chunk.edges.push({ n1: nextNode, n2: cityNode, type: 'ramp', isBridge: false });
                        agents.push({ node: nextNode, angle: agent.angle + _PI / 3, type: 'ramp', life: 6, z: 0 });
                        agents.push({ node: nextNode, angle: agent.angle - _PI / 3, type: 'ramp', life: 6, z: 0 });
                    }
                    agent.wasBridge = isBridge;

                    if (!isBridge && !agent.isCardinal && _random() < 0.25) {
                        agents.push({ node: nextNode, angle: agent.angle + (_random() > 0.5 ? _PI_2 : -_PI_2), type: 'ramp', life: 4, z: 0 });
                    }
                } else if (agent.type === 'ramp') {
                    agent.angle += (_random() - 0.5) * 0.1;
                } else if (agent.type === 'street' || agent.type === 'suburb_road') {
                    if (_random() < 0.1) agent.angle += (_random() > 0.5 ? _PI_4 : -_PI_4);
                    if (_random() < 0.45) {
                        agents.push({ node: nextNode, angle: agent.angle + (_random() > 0.5 ? _PI_2 : -_PI_2), type: agent.type, life: 80, z: 0 });
                    }
                    if (_random() < 0.15 && nextTerrain === T_CITY) {
                        agents.push({ node: nextNode, angle: agent.angle + (_random() > 0.5 ? _PI_2 : -_PI_2), type: 'alley', life: 25, z: 0 });
                    }
                } else if (agent.type === 'alley' || agent.type === 'coast') {
                    agent.angle += (_random() - 0.5) * 0.1;
                } else if (agent.type === 'park_path') {
                    agent.angle += (_random() - 0.5) * 1.2;
                    if (_random() < 0.15) {
                        agents.push({ node: nextNode, angle: agent.angle + (_random() > 0.5 ? _PI_2 : -_PI_2), type: 'park_path', life: 30, z: 0 });
                    }
                    if (getTerrain(nx, ny, seedOffset) !== T_PARK) forceMerge = true;
                }
            }

            if (agents.length === 0) {
                const uniqueEdges = [];
                const edgeSet = new Set();
                for (let i = 0; i < chunk.edges.length; i++) {
                    const e = chunk.edges[i];
                    const key = e.n1.id < e.n2.id ? `${e.n1.id}-${e.n2.id}` : `${e.n2.id}-${e.n1.id}`;
                    if (!edgeSet.has(key) && e.n1.id !== e.n2.id) { edgeSet.add(key); uniqueEdges.push(e); }
                }
                let cEdges = uniqueEdges;
                let changed = true;
                while (changed) {
                    changed = false;
                    const degrees = new Map();
                    for (let i = 0; i < cEdges.length; i++) {
                        const e = cEdges[i];
                        degrees.set(e.n1.id, (degrees.get(e.n1.id) || 0) + 1);
                        degrees.set(e.n2.id, (degrees.get(e.n2.id) || 0) + 1);
                    }
                    cEdges = cEdges.filter(e => {
                        if (e.type === 'highway') return true;
                        const d1 = degrees.get(e.n1.id), d2 = degrees.get(e.n2.id);
                        const isB1 = e.n1.x <= minX + 5 || e.n1.x >= maxX - 5 || e.n1.y <= minY + 5 || e.n1.y >= maxY - 5;
                        const isB2 = e.n2.x <= minX + 5 || e.n2.x >= maxX - 5 || e.n2.y <= minY + 5 || e.n2.y >= maxY - 5;
                        if ((d1 === 1 && !isB1) || (d2 === 1 && !isB2)) { changed = true; return false; }
                        return true;
                    });
                }
                chunk.edges = cEdges;
                chunk.roadGrid = buildRoadGrid(chunk.edges);
                chunk.phase = 'BUILDINGS';
            }
        }

        if (chunk.phase === 'BUILDINGS') {
            const EDGES_PER_FRAME = 60;
            let processed = 0;
            while (processed < EDGES_PER_FRAME && chunk.edgeProcessIndex < chunk.edges.length) {
                tryPlaceBuildingsAlongEdge(chunk, chunk.edges[chunk.edgeProcessIndex], seedOffset);
                chunk.edgeProcessIndex++;
                processed++;
            }
            if (chunk.edgeProcessIndex >= chunk.edges.length) chunk.phase = 'FENCES';
        }

        if (chunk.phase === 'FENCES') {
            const BLDGS_PER_FRAME = 120;
            let processed = 0;
            const buildings = chunk.buildings;
            while (processed < BLDGS_PER_FRAME && chunk.fenceProcessIndex < buildings.length) {
                const b1 = buildings[chunk.fenceProcessIndex];
                if (b1.type === 'COMMERCIAL' || b1.type === 'HOUSE') {
                    b1.fenceCount = b1.fenceCount || 0;
                    b1.connectedTo = b1.connectedTo || new Set();
                    if (b1.fenceCount < 2) {
                        const neighbors = getNearbyBuildings(chunk, b1);
                        const validNeighbors = [];
                        for (let ni = 0; ni < neighbors.length; ni++) {
                            const b2 = neighbors[ni];
                            if (b1 === b2 || (b2.type !== 'COMMERCIAL' && b2.type !== 'HOUSE') || b1.connectedTo.has(b2)) continue;
                            const ddx = b2.x - b1.x, ddy = b2.y - b1.y;
                            const dist = _sqrt(ddx * ddx + ddy * ddy);
                            if (dist < 40) validNeighbors.push({ b2, dist, dx: ddx, dy: ddy });
                        }
                        validNeighbors.sort((a, b) => a.dist - b.dist);

                        for (let vi = 0; vi < validNeighbors.length; vi++) {
                            if (b1.fenceCount >= 2) break;
                            const { b2, dist, dx, dy } = validNeighbors[vi];
                            b2.fenceCount = b2.fenceCount || 0;
                            b2.connectedTo = b2.connectedTo || new Set();
                            if (b2.fenceCount >= 2) continue;
                            const fenceObb = { x: (b1.x + b2.x) * 0.5, y: (b1.y + b2.y) * 0.5, w: dist, h: 1, angle: _atan2(dy, dx) };

                            let crossesRoad = false;
                            const nearbyEdges = getNearbyEdges(chunk, fenceObb);
                            for (let ei = 0; ei < nearbyEdges.length; ei++) {
                                const e = nearbyEdges[ei];
                                let hw = 1.5;
                                if (e.type === 'highway') hw = 4;
                                else if (e.type === 'park_path' || e.type === 'alley') hw = 0.5;
                                const rObb = roadSegmentToObb(e.n1.x, e.n1.y, e.n2.x, e.n2.y, hw + 1);
                                if (rObb && obbVsObb(fenceObb, rObb, 0)) { crossesRoad = true; break; }
                            }
                            if (!crossesRoad) {
                                let crossesFence = false;
                                const fences = chunk.fences;
                                for (let fi = 0; fi < fences.length; fi++) {
                                    const f = fences[fi];
                                    if (lineIntersect(b1.x, b1.y, b2.x, b2.y, f.x1, f.y1, f.x2, f.y2)) { crossesFence = true; break; }
                                }
                                if (!crossesFence) {
                                    chunk.fences.push({ x1: b1.x, y1: b1.y, x2: b2.x, y2: b2.y });
                                    b1.fenceCount++; b2.fenceCount++;
                                    b1.connectedTo.add(b2); b2.connectedTo.add(b1);
                                }
                            }
                        }
                    }
                }
                chunk.fenceProcessIndex++;
                processed++;
            }
            if (chunk.fenceProcessIndex >= buildings.length) chunk.phase = 'DONE';
        }

        if (chunk.phase === 'DONE') {
            world.generatedChunks.set(chunk.key, chunk);
            world.globalStats.nodes += chunk.nodes.length;
            world.globalStats.edges += chunk.edges.length;
            world.globalStats.fences += chunk.fences.length;
            const pk = chunk.buildings.filter(b => b.type === 'PARKING_LOT').length;
            world.globalStats.parking += pk;
            world.globalStats.buildings += (chunk.buildings.length - pk);
            world.activeChunk = null;
        }
    }, []);

    const drawCity = useCallback(() => {
        const canvas = canvasRef.current;
        if (!canvas) return;
        const ctx = canvas.getContext('2d');
        const wrapper = wrapperRef.current;
        if (!ctx || !wrapper) return;

        const cw = wrapper.clientWidth, ch = wrapper.clientHeight;
        if (canvas.width !== cw || canvas.height !== ch) { canvas.width = cw; canvas.height = ch; }

        const { x: camX, y: camY } = cameraRef.current;
        const zoom = zoomRef.current;

        ctx.fillStyle = '#bae6fd';
        ctx.fillRect(0, 0, cw, ch);

        const invZoom = 1 / zoom;
        const startX = _floor(camX / CHUNK_SIZE);
        const endX = _floor((camX + cw * invZoom) / CHUNK_SIZE);
        const startY = _floor(camY / CHUNK_SIZE);
        const endY = _floor((camY + ch * invZoom) / CHUNK_SIZE);

        const world = worldRef.current;

        ctx.save();
        ctx.scale(zoom, zoom);
        ctx.translate(-camX, -camY);

        const drawChunkData = (chunk) => {
            const edges = chunk.edges;
            const edgeLen = edges.length;

            ctx.lineCap = 'butt';
            ctx.lineJoin = 'miter';

            // Batch draw helper using index loop (avoids filter allocations per frame)
            const batchDraw = (color, lw, testFn) => {
                ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = lw;
                for (let i = 0; i < edgeLen; i++) {
                    const e = edges[i];
                    if (testFn(e)) { ctx.moveTo(e.n1.x, e.n1.y); ctx.lineTo(e.n2.x, e.n2.y); }
                }
                ctx.stroke();
            };

            // Ground casings
            batchDraw('#1e293b', 3.8, e => !e.isBridge && (e.type === 'street' || e.type === 'coast'));
            batchDraw('#334155', 3.0, e => !e.isBridge && e.type === 'suburb_road');
            batchDraw('#1e293b', 3.8, e => !e.isBridge && e.type === 'ramp');

            // Ground fills
            batchDraw('#ffffff', 2.2, e => !e.isBridge && (e.type === 'street' || e.type === 'coast'));
            batchDraw('#cbd5e1', 1.6, e => !e.isBridge && e.type === 'suburb_road');
            batchDraw('#facc15', 2.2, e => !e.isBridge && e.type === 'ramp');

            // Alleys dashed
            ctx.save(); ctx.setLineDash([2, 3]);
            batchDraw('#475569', 1.2, e => !e.isBridge && e.type === 'alley');
            ctx.restore();

            // Park trails dashed
            ctx.save(); ctx.setLineDash([3, 2]);
            batchDraw('#059669', 1.4, e => !e.isBridge && e.type === 'park_path');
            ctx.restore();

            // Parking lots
            const buildings = chunk.buildings;
            const bLen = buildings.length;
            for (let i = 0; i < bLen; i++) {
                const b = buildings[i];
                if (b.type !== 'PARKING_LOT') continue;
                ctx.save();
                ctx.translate(b.x, b.y); ctx.rotate(b.angle);
                ctx.fillStyle = '#e2e8f0';
                ctx.fillRect(-b.w * 0.5, -b.h * 0.5, b.w, b.h);
                ctx.strokeStyle = '#cbd5e1'; ctx.lineWidth = 0.6;
                ctx.strokeRect(-b.w * 0.5, -b.h * 0.5, b.w, b.h);

                const cars = b.cars;
                ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 0.5; ctx.beginPath();
                for (let ci = 0; ci < cars.length; ci++) {
                    ctx.moveTo(cars[ci].lx - 1.25, cars[ci].ly - 1.5);
                    ctx.lineTo(cars[ci].lx - 1.25, cars[ci].ly + 1.5);
                }
                ctx.stroke();
                for (let ci = 0; ci < cars.length; ci++) {
                    ctx.fillStyle = cars[ci].color;
                    ctx.fillRect(cars[ci].lx - 0.8, cars[ci].ly - 1.2, 1.6, 2.4);
                }
                ctx.restore();
            }

            // Fences
            if (chunk.fences.length > 0) {
                ctx.beginPath(); ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 1.0;
                const fences = chunk.fences;
                for (let i = 0; i < fences.length; i++) {
                    ctx.moveTo(fences[i].x1, fences[i].y1);
                    ctx.lineTo(fences[i].x2, fences[i].y2);
                }
                ctx.stroke();
            }

            // Buildings & trees
            for (let i = 0; i < bLen; i++) {
                const b = buildings[i];
                if (b.type === 'PARKING_LOT') continue;
                ctx.save();
                ctx.translate(b.x, b.y); ctx.rotate(b.angle);
                if (b.type === 'COMMERCIAL') {
                    ctx.fillStyle = '#e0f2fe'; ctx.strokeStyle = '#0284c7'; ctx.lineWidth = 0.8;
                    ctx.fillRect(-b.w * 0.5, -b.h * 0.5, b.w, b.h);
                    ctx.strokeRect(-b.w * 0.5, -b.h * 0.5, b.w, b.h);
                    ctx.fillStyle = '#ffffff';
                    ctx.fillRect(-b.w * 0.5 + 1, -b.h * 0.5 + 1, b.w / 3.5, b.h - 2);
                } else if (b.type === 'HOUSE') {
                    ctx.fillStyle = '#fed7aa'; ctx.strokeStyle = '#c2410c'; ctx.lineWidth = 0.8;
                    ctx.fillRect(-b.w * 0.5, -b.h * 0.5, b.w, b.h);
                    ctx.strokeRect(-b.w * 0.5, -b.h * 0.5, b.w, b.h);
                    ctx.beginPath(); ctx.strokeStyle = '#9a3412'; ctx.lineWidth = 0.8;
                    ctx.moveTo(-b.w * 0.5, 0); ctx.lineTo(b.w * 0.5, 0); ctx.stroke();
                } else if (b.type === 'TREE') {
                    ctx.fillStyle = '#86efac'; ctx.strokeStyle = '#15803d'; ctx.lineWidth = 0.8;
                    ctx.beginPath(); ctx.arc(0, 0, _max(1.2, b.w * 0.5), 0, _PI2); ctx.fill(); ctx.stroke();
                }
                ctx.restore();
            }

            // Highways & bridges on top
            ctx.lineCap = 'round'; ctx.lineJoin = 'round';

            // Shadows with offset
            ctx.save(); ctx.translate(2.5, 4.0);
            batchDraw('rgba(15,23,42,0.25)', 5.5, e => e.type === 'highway');
            ctx.restore();
            ctx.save(); ctx.translate(2.0, 3.5);
            batchDraw('rgba(78,53,15,0.3)', 4.0, e => e.type === 'causeway');
            ctx.restore();

            // Casings
            batchDraw('#0f172a', 6.0, e => e.type === 'highway');
            batchDraw('#78350f', 4.5, e => e.type === 'causeway');

            // Fills
            batchDraw('#f59e0b', 3.8, e => e.type === 'highway');
            batchDraw('#d97706', 2.5, e => e.type === 'causeway');
        };

        // Terrain tiles
        for (let cx = startX; cx <= endX; cx++) {
            for (let cy = startY; cy <= endY; cy++) {
                const key = `${cx},${cy}`;
                const isReached = world.generatedChunks.has(key) || world.activeChunk?.key === key || world.chunkQueue.includes(key);
                if (isReached) {
                    if (!world.terrainCache.has(key)) generateChunkTerrain(cx, cy);
                    const tCanvas = world.terrainCache.get(key);
                    if (tCanvas) ctx.drawImage(tCanvas, cx * CHUNK_SIZE, cy * CHUNK_SIZE);
                }
            }
        }

        // Vector data
        for (let cx = startX; cx <= endX; cx++) {
            for (let cy = startY; cy <= endY; cy++) {
                const key = `${cx},${cy}`;
                if (world.generatedChunks.has(key)) drawChunkData(world.generatedChunks.get(key));
                if (world.activeChunk?.key === key) drawChunkData(world.activeChunk);
            }
        }
        ctx.restore();

        const ac = world.activeChunk;
        let activeLabel = 'Idle (Pan to discover)';
        if (ac) {
            activeLabel = ac.phase === 'GROWING' ? 'Growing Roads'
                : ac.phase === 'BUILDINGS' ? `Districts (${_floor((ac.edgeProcessIndex / ac.edges.length) * 100)}%)`
                    : `Fences (${_floor((ac.fenceProcessIndex / ac.buildings.length) * 100)}%)`;
        }

        setUiStats({
            phase: activeLabel,
            activeIsland: ac ? ac.key : null,
            queued: world.chunkQueue.length,
            nodes: world.globalStats.nodes + (ac ? ac.nodes.length : 0),
            edges: world.globalStats.edges + (ac ? ac.edges.length : 0),
            buildings: world.globalStats.buildings + (ac ? ac.buildings.filter(b => b.type !== 'PARKING_LOT').length : 0),
            parking: world.globalStats.parking + (ac ? ac.buildings.filter(b => b.type === 'PARKING_LOT').length : 0),
            fences: world.globalStats.fences + (ac ? ac.fences.length : 0)
        });
    }, []);

    const animate = useCallback(() => {
        if (isRunning) stepSimulation();
        drawCity();
        requestRef.current = requestAnimationFrame(animate);
    }, [isRunning, stepSimulation, drawCity]);

    const changeZoom = useCallback((newZoom) => {
        newZoom = _max(0.15, _min(4.0, newZoom));
        zoomRef.current = newZoom;
        setZoomState(newZoom);
        drawCity();
    }, [drawCity]);

    useEffect(() => {
        initWorld();
        const handleResize = () => drawCity();
        window.addEventListener('resize', handleResize);
        return () => window.removeEventListener('resize', handleResize);
    }, [initWorld, drawCity]);

    useEffect(() => {
        requestRef.current = requestAnimationFrame(animate);
        return () => { if (requestRef.current) cancelAnimationFrame(requestRef.current); };
    }, [animate]);

    useEffect(() => {
        const wrapper = wrapperRef.current;
        if (!wrapper) return;
        const handleWheel = (e) => {
            e.preventDefault();
            const rect = wrapper.getBoundingClientRect();
            const zoom = zoomRef.current;
            const mouseXWorld = cameraRef.current.x + (e.clientX - rect.left) / zoom;
            const mouseYWorld = cameraRef.current.y + (e.clientY - rect.top) / zoom;
            const newZoom = _max(0.15, _min(4.0, e.deltaY < 0 ? zoom * 1.1 : zoom / 1.1));
            cameraRef.current.x = mouseXWorld - (e.clientX - rect.left) / newZoom;
            cameraRef.current.y = mouseYWorld - (e.clientY - rect.top) / newZoom;
            zoomRef.current = newZoom;
            setZoomState(newZoom);
            drawCity();
        };
        wrapper.addEventListener('wheel', handleWheel, { passive: false });
        return () => wrapper.removeEventListener('wheel', handleWheel);
    }, [drawCity]);

    const handleMouseDown = (e) => { dragRef.current = { isDragging: true, lastX: e.clientX, lastY: e.clientY }; };
    const handleMouseMove = (e) => {
        if (!dragRef.current.isDragging) return;
        const zoom = zoomRef.current;
        cameraRef.current.x -= (e.clientX - dragRef.current.lastX) / zoom;
        cameraRef.current.y -= (e.clientY - dragRef.current.lastY) / zoom;
        dragRef.current.lastX = e.clientX;
        dragRef.current.lastY = e.clientY;
    };
    const handleMouseUp = () => { dragRef.current.isDragging = false; };

    return (
        <>
        <div className="flex flex-col h-screen w-full bg-slate-50 font-sans text-slate-800 select-none">
            <div className="flex flex-wrap items-center justify-between p-4 bg-white border-b border-slate-200 z-10 relative shrink-0 shadow-sm gap-4">
                <div>
                    <h1 className="text-lg font-semibold text-slate-800 tracking-tight">Procedural City Plan</h1>
                    <div className="flex flex-col gap-1.5 mt-1.5 text-xs text-slate-500">
                        <div className="flex flex-wrap gap-x-4 gap-y-1 items-center">
                            <span className="font-semibold text-slate-700 mr-1">Roads:</span>
                            <span className="flex items-center gap-1.5"><span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#0f172a] bg-[#f59e0b]"></span>Highway / Bridge</span>
                            <span className="flex items-center gap-1.5"><span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#78350f] bg-[#d97706]"></span>Causeway</span>
                            <span className="flex items-center gap-1.5"><span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#1e293b] bg-[#ffffff]"></span>Main Street</span>
                            <span className="flex items-center gap-1.5"><span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#334155] bg-[#cbd5e1]"></span>Suburb Road</span>
                            <span className="flex items-center gap-1.5"><span className="w-5 h-0.5 border-t border-dashed border-[#475569]"></span>Alley</span>
                            <span className="flex items-center gap-1.5"><span className="w-5 h-0.5 border-t border-dashed border-[#059669]"></span>Park Trail</span>
                        </div>
                        <div className="flex flex-wrap gap-x-4 gap-y-1 items-center">
                            <span className="font-semibold text-slate-700 mr-1">Zones:</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#bae6fd] border border-[#cbd5e1]"></span>Water</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#e2e8f0] border border-[#cbd5e1]"></span>City Center</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#f8fafc] border border-[#cbd5e1]"></span>Suburbs</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#dcfce7] border border-[#cbd5e1]"></span>Park / Forest</span>
                            <span className="font-semibold text-slate-700 ml-2 mr-1">Structures:</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#e0f2fe] border border-[#0284c7]"></span>Commercial</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#fed7aa] border border-[#c2410c]"></span>House</span>
                            <span className="flex items-center gap-1.5"><span className="w-2.5 h-2.5 rounded-full bg-[#86efac] border border-[#15803d]"></span>Tree</span>
                            <span className="flex items-center gap-1.5"><span className="w-3.5 h-2.5 rounded bg-[#e2e8f0] border border-[#cbd5e1] flex items-center justify-center"><span className="text-[7.5px] text-[#64748b] font-bold font-sans leading-none">P</span></span>Parking</span>
                        </div>
                    </div>
                </div>

                <div className="flex flex-wrap items-center gap-4">
                    <div className="flex gap-4 text-xs font-mono text-slate-600 bg-slate-50 px-3 py-1.5 rounded border border-slate-200">
                        <div>Island: <span className="text-slate-800 font-medium">{uiStats.activeIsland ? `[${uiStats.activeIsland}]` : 'None'}</span></div>
                        <div>Status: <span className="text-slate-800 font-medium">{uiStats.phase}</span></div>
                        <div>Queued: <span className="text-slate-800 font-medium">{uiStats.queued}</span></div>
                    </div>
                    <div className="flex items-center gap-4">
                        <div className="flex gap-3 text-xs font-mono text-slate-500">
                            <div>Roads: <span className="text-slate-700 font-medium">{uiStats.edges}</span></div>
                            <div>Structures: <span className="text-slate-700 font-medium">{uiStats.buildings}</span></div>
                            <div>Fences: <span className="text-slate-700 font-medium">{uiStats.fences}</span></div>
                        </div>
                        <button onClick={() => setIsRunning(!isRunning)} className="px-3 py-1.5 rounded bg-white hover:bg-slate-50 border border-slate-200 text-xs font-medium text-slate-700 transition-colors shadow-sm cursor-pointer">
                            {isRunning ? 'Pause Engine' : 'Resume Engine'}
                        </button>
                        <button onClick={() => { initWorld(); setIsRunning(true); }} className="px-3 py-1.5 rounded bg-slate-900 hover:bg-slate-800 text-white text-xs font-medium transition-colors shadow-sm cursor-pointer">
                            Wipe Earth
                        </button>
                        <button
                            onClick={() => setView3D(true)}
                            style={{ background: 'linear-gradient(135deg,#00c8ff,#8800ff)', border: 'none', color: '#fff', padding: '6px 16px', borderRadius: 6, fontSize: 12, fontWeight: 700, letterSpacing: 2, cursor: 'pointer', boxShadow: '0 0 16px rgba(0,200,255,0.35)' }}
                        >
                            ▶ 3D VIEW
                        </button>
                    </div>
                </div>
            </div>

            <div
                className="flex-1 relative overflow-hidden cursor-grab active:cursor-grabbing bg-[#e0f2fe]"
                ref={wrapperRef}
                onMouseDown={handleMouseDown}
                onMouseMove={handleMouseMove}
                onMouseUp={handleMouseUp}
                onMouseLeave={handleMouseUp}
            >
                <canvas ref={canvasRef} className="absolute inset-0 block" />
                <div className="absolute bottom-4 left-4 text-[10px] font-mono text-slate-400 pointer-events-none bg-white/80 backdrop-blur-sm px-2 py-1 rounded border border-slate-200 shadow-sm">
                    Drag to Pan • Scroll to Zoom
                </div>
                <div className="absolute bottom-4 right-4 flex items-center gap-1 bg-white/90 backdrop-blur-sm border border-slate-200 p-1.5 rounded-lg shadow-sm pointer-events-auto">
                    <button
                        onClick={() => {
                            if (!wrapperRef.current) return;
                            const zoom = zoomRef.current;
                            const cx = cameraRef.current.x + wrapperRef.current.clientWidth / (2 * zoom);
                            const cy = cameraRef.current.y + wrapperRef.current.clientHeight / (2 * zoom);
                            const newZoom = _min(4.0, zoom * 1.25);
                            cameraRef.current.x = cx - wrapperRef.current.clientWidth / (2 * newZoom);
                            cameraRef.current.y = cy - wrapperRef.current.clientHeight / (2 * newZoom);
                            changeZoom(newZoom);
                        }}
                        className="w-7 h-7 flex items-center justify-center rounded bg-slate-50 hover:bg-slate-100 border border-slate-200 text-slate-700 font-bold text-sm cursor-pointer transition-colors"
                        title="Zoom In"
                    >+</button>
                    <button
                        onClick={() => {
                            if (!wrapperRef.current) return;
                            const zoom = zoomRef.current;
                            const cx = cameraRef.current.x + wrapperRef.current.clientWidth / (2 * zoom);
                            const cy = cameraRef.current.y + wrapperRef.current.clientHeight / (2 * zoom);
                            const newZoom = _max(0.15, zoom / 1.25);
                            cameraRef.current.x = cx - wrapperRef.current.clientWidth / (2 * newZoom);
                            cameraRef.current.y = cy - wrapperRef.current.clientHeight / (2 * newZoom);
                            changeZoom(newZoom);
                        }}
                        className="w-7 h-7 flex items-center justify-center rounded bg-slate-50 hover:bg-slate-100 border border-slate-200 text-slate-700 font-bold text-sm cursor-pointer transition-colors"
                        title="Zoom Out"
                    >−</button>
                    <button
                        onClick={() => {
                            if (!wrapperRef.current) return;
                            const zoom = zoomRef.current;
                            const cx = cameraRef.current.x + wrapperRef.current.clientWidth / (2 * zoom);
                            const cy = cameraRef.current.y + wrapperRef.current.clientHeight / (2 * zoom);
                            cameraRef.current.x = cx - wrapperRef.current.clientWidth / 2;
                            cameraRef.current.y = cy - wrapperRef.current.clientHeight / 2;
                            changeZoom(1.0);
                        }}
                        className="px-2 h-7 flex items-center justify-center rounded bg-slate-50 hover:bg-slate-100 border border-slate-200 text-slate-700 text-xs font-semibold cursor-pointer transition-colors"
                        title="Reset Zoom"
                    >{Math.round(zoomState * 100)}%</button>
                </div>
            </div>
        </div>
        {view3D && (
            <VoxelView worldRef={worldRef} onClose={() => setView3D(false)} />
        )}
        </>
    );
}