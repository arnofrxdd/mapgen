import React, { useEffect, useRef, useState, useCallback } from 'react';

// --- Procedural Noise for Terrain ---
const random = (x, y) => {
    let n = Math.sin(x * 12.9898 + y * 78.233) * 43758.5453123;
    return n - Math.floor(n);
};

const smoothNoise = (x, y) => {
    const ix = Math.floor(x); const iy = Math.floor(y);
    const fx = x - ix; const fy = y - iy;
    const v1 = random(ix, iy);
    const v2 = random(ix + 1, iy);
    const v3 = random(ix, iy + 1);
    const v4 = random(ix + 1, iy + 1);
    const i1 = v1 * (1 - fx) + v2 * fx;
    const i2 = v3 * (1 - fx) + v4 * fx;
    return i1 * (1 - fy) + i2 * fy;
};

const fbm = (x, y, scale = 0.003) => {
    let v = 0; let amp = 0.5; let f = scale;
    for (let i = 0; i < 4; i++) {
        v += smoothNoise(x * f, y * f) * amp;
        f *= 2; amp *= 0.5;
    }
    return v;
};

// --- OBB (Oriented Bounding Box) Helpers ---
const getBuildingCorners = (b) => {
    const hw = b.w / 2;
    const hh = b.h / 2;
    const cos = Math.cos(b.angle);
    const sin = Math.sin(b.angle);
    return [
        { x: b.x + cos * (-hw) - sin * (-hh), y: b.y + sin * (-hw) + cos * (-hh) },
        { x: b.x + cos * (hw) - sin * (-hh), y: b.y + sin * (hw) + cos * (-hh) },
        { x: b.x + cos * (hw) - sin * (hh), y: b.y + sin * (hw) + cos * (hh) },
        { x: b.x + cos * (-hw) - sin * (hh), y: b.y + sin * (-hw) + cos * (hh) },
    ];
};

const dot = (ax, ay, bx, by) => ax * bx + ay * by;

const projectPoly = (corners, ax, ay) => {
    let min = Infinity, max = -Infinity;
    for (const c of corners) {
        const p = dot(c.x, c.y, ax, ay);
        if (p < min) min = p;
        if (p > max) max = p;
    }
    return [min, max];
};

const overlapIntervals = (a, b, clearance) => {
    return a[0] - clearance < b[1] && b[0] - clearance < a[1];
};

const obbVsObb = (a, b, padding = 0) => {
    const cornersA = getBuildingCorners(a);
    const cornersB = getBuildingCorners(b);
    const axes = [
        [Math.cos(a.angle), Math.sin(a.angle)],
        [-Math.sin(a.angle), Math.cos(a.angle)],
        [Math.cos(b.angle), Math.sin(b.angle)],
        [-Math.sin(b.angle), Math.cos(b.angle)],
    ];
    for (const [ax, ay] of axes) {
        const pA = projectPoly(cornersA, ax, ay);
        const pB = projectPoly(cornersB, ax, ay);
        if (!overlapIntervals(pA, pB, padding)) return false;
    }
    return true;
};

const roadSegmentToObb = (rx1, ry1, rx2, ry2, halfWidth) => {
    const dx = rx2 - rx1;
    const dy = ry2 - ry1;
    const len = Math.hypot(dx, dy);
    if (len < 0.001) return null;
    return {
        x: (rx1 + rx2) / 2, y: (ry1 + ry2) / 2,
        w: len + halfWidth * 2, h: halfWidth * 2,
        angle: Math.atan2(dy, dx),
    };
};

const lineIntersect = (p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y) => {
    if ((Math.abs(p0_x - p2_x) < 0.1 && Math.abs(p0_y - p2_y) < 0.1) ||
        (Math.abs(p0_x - p3_x) < 0.1 && Math.abs(p0_y - p3_y) < 0.1) ||
        (Math.abs(p1_x - p2_x) < 0.1 && Math.abs(p1_y - p2_y) < 0.1) ||
        (Math.abs(p1_x - p3_x) < 0.1 && Math.abs(p1_y - p3_y) < 0.1)) {
        return false;
    }
    const s1_x = p1_x - p0_x, s1_y = p1_y - p0_y;
    const s2_x = p3_x - p2_x, s2_y = p3_y - p2_y;
    const denom = -s2_x * s1_y + s1_x * s2_y;
    if (Math.abs(denom) < 0.0001) return false;
    const s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / denom;
    const t = (s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / denom;
    return s > 0.01 && s < 0.99 && t > 0.01 && t < 0.99;
};

// --- Constants & Config ---
const CHUNK_SIZE = 1200; // Represents an isolated "Island" Region
const STEP_SIZE = 15;
const ALLEY_STEP = 10;
const MERGE_RADIUS = 12;
const GROWTH_SPEED = 20;
const CELL_SIZE = 30;
const BLDG_CELL = 32;
const ROAD_CELL = 100; // Larger cell size for roads

// Terrain Thresholds for Districts
const WATER_LVL = 0.35;
const CITY_LVL = 0.52;
const SUBURB_LVL = 0.65;

export default function App() {
    const canvasRef = useRef(null);
    const wrapperRef = useRef(null);
    const requestRef = useRef(null);

    // Camera, Zoom and Panning state
    const cameraRef = useRef({ x: 0, y: 0 });
    const zoomRef = useRef(1.0);
    const [zoomState, setZoomState] = useState(1.0);
    const dragRef = useRef({ isDragging: false, lastX: 0, lastY: 0 });

    // Endless World State Machine
    const worldRef = useRef({
        seedOffset: Math.random() * 10000,
        chunkQueue: [], // Array of string keys 'cx,cy'
        generatedChunks: new Map(), // Keyed by 'cx,cy'
        activeChunk: null, // The island currently being built
        terrainCache: new Map(), // Offscreen canvases for performance
        globalStats: { nodes: 0, edges: 0, buildings: 0, parking: 0, fences: 0, nodeCounter: 0 },
        borderNodes: [] // Global registry for bridges handing off between chunks
    });

    const [isRunning, setIsRunning] = useState(true);
    const [uiStats, setUiStats] = useState({
        phase: 'Initializing', activeIsland: null, queued: 0,
        nodes: 0, edges: 0, buildings: 0, parking: 0, fences: 0
    });

    // --- Core Spatials & Data Localized to a Chunk ---
    const getCellKey = (x, y) => `${Math.floor(x / CELL_SIZE)},${Math.floor(y / CELL_SIZE)}`;

    const addNode = (chunk, x, y, z = 0) => {
        const id = worldRef.current.globalStats.nodeCounter++;
        const node = { id, x, y, z };
        chunk.nodes.push(node);
        const key = getCellKey(x, y);
        if (!chunk.spatialGrid.has(key)) chunk.spatialGrid.set(key, []);
        chunk.spatialGrid.get(key).push(node);
        return node;
    };

    const findNearestNode = (chunk, x, y, radius, excludeNode = null, targetZ = 0) => {
        const cx = Math.floor(x / CELL_SIZE);
        const cy = Math.floor(y / CELL_SIZE);
        const searchCells = Math.ceil(radius / CELL_SIZE);
        let nearest = null; let minDist = radius * radius;
        for (let i = -searchCells; i <= searchCells; i++) {
            for (let j = -searchCells; j <= searchCells; j++) {
                const cell = chunk.spatialGrid.get(`${cx + i},${cy + j}`);
                if (cell) {
                    for (const node of cell) {
                        if (node === excludeNode || node.z !== targetZ) continue;
                        const distSq = (node.x - x) ** 2 + (node.y - y) ** 2;
                        if (distSq < minDist) { minDist = distSq; nearest = node; }
                    }
                }
            }
        }
        return nearest;
    };

    const getTerrain = (worldX, worldY, offset) => {
        const cx = Math.floor(worldX / CHUNK_SIZE);
        const cy = Math.floor(worldY / CHUNK_SIZE);
        const centerX = cx * CHUNK_SIZE + CHUNK_SIZE / 2;
        const centerY = cy * CHUNK_SIZE + CHUNK_SIZE / 2;

        // Domain warping: warp the input coordinates using multi-scale noise
        const warpX1 = (fbm(worldX + offset, worldY + offset, 0.001) - 0.45) * 550;
        const warpY1 = (fbm(worldX + offset + 500, worldY + offset + 500, 0.001) - 0.45) * 550;
        const warpX2 = (fbm(worldX + offset + 1000, worldY + offset + 1000, 0.005) - 0.45) * 150;
        const warpY2 = (fbm(worldX + offset + 1500, worldY + offset + 1500, 0.005) - 0.45) * 150;

        const warpedX = worldX - (warpX1 + warpX2);
        const warpedY = worldY - (warpY1 + warpY2);

        // Calculate distance from the warped position to the center of the chunk
        const warpedDist = Math.hypot(warpedX - centerX, warpedY - centerY);

        // Use warped coordinates for the base terrain value so land features align with coastlines
        const val = fbm(warpedX + offset, warpedY + offset);

        const maxRadius = CHUNK_SIZE * 0.48; // Water border radius
        const falloffStart = CHUNK_SIZE * 0.22; // Start dipping into water here

        let finalVal = val;
        if (warpedDist > falloffStart) {
            const drop = (warpedDist - falloffStart) / (maxRadius - falloffStart);
            finalVal -= drop * 2.0; // Stronger dropoff to ensure clear channels of water between warped islands
        } else if (warpedDist < falloffStart * 0.5) {
            // Keep only the deep interior solidly above water
            if (finalVal < WATER_LVL + 0.05) {
                finalVal = WATER_LVL + 0.05;
            }
        }

        if (finalVal < WATER_LVL) return 'WATER';
        if (finalVal < CITY_LVL) return 'CITY';
        if (finalVal < SUBURB_LVL) return 'SUBURB';
        return 'PARK';
    };

    // STRICT WATER CHECK: Raycasts the proposed road line. Returns true if there's water beneath it.
    const checkWaterCrossing = (n1, n2, offset) => {
        const dx = n2.x - n1.x; const dy = n2.y - n1.y;
        for (let i = 1; i <= 3; i++) {
            if (getTerrain(n1.x + dx * (i / 4), n1.y + dy * (i / 4), offset) === 'WATER') return true;
        }
    };

    const isInvalidRampConnection = (chunk, node) => {
        for (const e of chunk.edges) {
            if (e.n1.id === node.id || e.n2.id === node.id) {
                if (e.type === 'alley' || e.type === 'park_path') return true;
            }
        }
        return false;
    };

    const isRampNode = (chunk, node) => {
        for (const e of chunk.edges) {
            if (e.n1.id === node.id || e.n2.id === node.id) {
                if (e.type === 'ramp' || e.type === 'highway') return true;
            }
        }
        return false;
    };

    const addBuildingToGrid = (chunk, b) => {
        const corners = getBuildingCorners(b);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const c of corners) {
            minX = Math.min(minX, c.x); maxX = Math.max(maxX, c.x);
            minY = Math.min(minY, c.y); maxY = Math.max(maxY, c.y);
        }
        const x0 = Math.floor(minX / BLDG_CELL), x1 = Math.floor(maxX / BLDG_CELL);
        const y0 = Math.floor(minY / BLDG_CELL), y1 = Math.floor(maxY / BLDG_CELL);
        for (let gx = x0; gx <= x1; gx++) {
            for (let gy = y0; gy <= y1; gy++) {
                const key = `${gx},${gy}`;
                if (!chunk.buildingGrid.has(key)) chunk.buildingGrid.set(key, []);
                chunk.buildingGrid.get(key).push(b);
            }
        }
    };

    const getNearbyBuildings = (chunk, b) => {
        const corners = getBuildingCorners(b);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const c of corners) {
            minX = Math.min(minX, c.x); maxX = Math.max(maxX, c.x);
            minY = Math.min(minY, c.y); maxY = Math.max(maxY, c.y);
        }
        const x0 = Math.floor(minX / BLDG_CELL) - 1, x1 = Math.floor(maxX / BLDG_CELL) + 1;
        const y0 = Math.floor(minY / BLDG_CELL) - 1, y1 = Math.floor(maxY / BLDG_CELL) + 1;
        const seen = new Set(), result = [];
        for (let gx = x0; gx <= x1; gx++) {
            for (let gy = y0; gy <= y1; gy++) {
                const cell = chunk.buildingGrid.get(`${gx},${gy}`);
                if (cell) {
                    for (const nb of cell) {
                        if (!seen.has(nb)) { seen.add(nb); result.push(nb); }
                    }
                }
            }
        }
        return result;
    };

    const buildRoadGrid = (edges) => {
        const grid = new Map();
        for (const e of edges) {
            const steps = Math.ceil(Math.hypot(e.n2.x - e.n1.x, e.n2.y - e.n1.y) / ROAD_CELL) + 1;
            const seen = new Set();
            for (let s = 0; s <= steps; s++) {
                const t = s / steps;
                const mx = e.n1.x + (e.n2.x - e.n1.x) * t;
                const my = e.n1.y + (e.n2.y - e.n1.y) * t;
                const key = `${Math.floor(mx / ROAD_CELL)},${Math.floor(my / ROAD_CELL)}`;
                if (!seen.has(key)) {
                    seen.add(key);
                    if (!grid.has(key)) grid.set(key, []);
                    grid.get(key).push(e);
                }
            }
        }
        return grid;
    };

    const getNearbyEdges = (chunk, b) => {
        const corners = getBuildingCorners(b);
        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        for (const c of corners) {
            minX = Math.min(minX, c.x); maxX = Math.max(maxX, c.x);
            minY = Math.min(minY, c.y); maxY = Math.max(maxY, c.y);
        }
        const x0 = Math.floor(minX / ROAD_CELL) - 1, x1 = Math.floor(maxX / ROAD_CELL) + 1;
        const y0 = Math.floor(minY / ROAD_CELL) - 1, y1 = Math.floor(maxY / ROAD_CELL) + 1;
        const seen = new Set(), result = [];
        for (let gx = x0; gx <= x1; gx++) {
            for (let gy = y0; gy <= y1; gy++) {
                const cell = chunk.roadGrid.get(`${gx},${gy}`);
                if (cell) {
                    for (const e of cell) {
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
        const minX = cx * CHUNK_SIZE; const maxX = minX + CHUNK_SIZE;
        const minY = cy * CHUNK_SIZE; const maxY = minY + CHUNK_SIZE;

        let startedFromBorder = false;

        for (let i = worldRef.current.borderNodes.length - 1; i >= 0; i--) {
            const bn = worldRef.current.borderNodes[i];
            if (bn.x >= minX - 1 && bn.x <= maxX + 1 && bn.y >= minY - 1 && bn.y <= maxY + 1) {
                const newNode = addNode(chunk, bn.x, bn.y, bn.z);

                const agentLife = bn.type === 'highway' ? 1800 : 120;
                chunk.agents.push({
                    node: newNode, angle: bn.angle, type: bn.type, life: agentLife, z: bn.z,
                    wasBridge: bn.wasBridge || false, isCardinal: bn.isCardinal || false,
                    hasSpawnedPerpendiculars: false, // Resets for the new chunk so it can trigger the center branching
                    isArrivingBridge: bn.isArrivingBridge || false
                });

                worldRef.current.borderNodes.splice(i, 1);
                startedFromBorder = true;
            }
        }

        if (!startedFromBorder) {
            // ONLY Chunk 0,0 is allowed to manifest without a bridge
            if (cx === 0 && cy === 0) {
                const centerX = cx * CHUNK_SIZE + CHUNK_SIZE / 2;
                const centerY = cy * CHUNK_SIZE + CHUNK_SIZE / 2;
                const centerNode = addNode(chunk, centerX, centerY, 1);

                for (let i = 0; i < 4; i++) {
                    chunk.agents.push({
                        node: centerNode, angle: (i * Math.PI) / 2, type: 'highway', life: 1800, z: 1, wasBridge: false, isCardinal: true, hasSpawnedPerpendiculars: true
                    });
                }

                for (let i = 0; i < 4; i++) {
                    chunk.agents.push({
                        node: centerNode, angle: (i * Math.PI) / 2 + Math.PI / 4, type: 'highway', life: 120, z: 1, wasBridge: false, isCardinal: false
                    });
                }
                chunk.agents.push({ node: centerNode, angle: 0, type: 'ramp', life: 5, z: 0 });
                chunk.agents.push({ node: centerNode, angle: Math.PI, type: 'ramp', life: 5, z: 0 });
            } else {
                return null; // Violently reject disconnected chunk generation
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

        const STEP = 4;
        for (let y = 0; y < CHUNK_SIZE; y += STEP) {
            for (let x = 0; x < CHUNK_SIZE; x += STEP) {
                const worldX = cx * CHUNK_SIZE + x;
                const worldY = cy * CHUNK_SIZE + y;
                const type = getTerrain(worldX, worldY, offset);

                let r, g, b;
                if (type === 'WATER') { r = 186; g = 230; b = 253; } // #bae6fd (sky-200)
                else if (type === 'CITY') { r = 226; g = 232; b = 240; } // #e2e8f0 (slate-200)
                else if (type === 'SUBURB') { r = 248; g = 250; b = 252; } // #f8fafc (slate-50)
                else { r = 220; g = 252; b = 231; } // #dcfce7 (emerald-100)

                for (let dy = 0; dy < STEP; dy++) {
                    if (y + dy >= CHUNK_SIZE) continue;
                    for (let dx = 0; dx < STEP; dx++) {
                        if (x + dx >= CHUNK_SIZE) continue;
                        const i = ((y + dy) * CHUNK_SIZE + (x + dx)) * 4;
                        imgData.data[i] = r; imgData.data[i + 1] = g; imgData.data[i + 2] = b; imgData.data[i + 3] = 255;
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

        cameraRef.current = {
            x: -(width / 2) + (CHUNK_SIZE / 2),
            y: -(height / 2) + (CHUNK_SIZE / 2)
        };

        worldRef.current = {
            seedOffset: Math.random() * 10000,
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
        if (edge.isBridge || edge.type === 'coast' || edge.type === 'alley' || edge.type === 'highway' || edge.type === 'ramp') return;

        const dx = edge.n2.x - edge.n1.x;
        const dy = edge.n2.y - edge.n1.y;
        const len = Math.hypot(dx, dy);
        if (len < 5) return;

        const nx = -dy / len;
        const ny = dx / len;
        const angle = Math.atan2(dy, dx);

        const minX = chunk.cx * CHUNK_SIZE; const maxX = minX + CHUNK_SIZE;
        const minY = chunk.cy * CHUNK_SIZE; const maxY = minY + CHUNK_SIZE;

        let roadHalfWidth = 1.5; let sidewalk = 1.5;
        if (edge.type === 'ramp') { roadHalfWidth = 2; sidewalk = 2; }
        if (edge.type === 'park_path') { roadHalfWidth = 0.5; sidewalk = 0.5; }

        for (const dir of [1, -1]) {
            let currentT = 1;
            const endT = len - 1;

            while (currentT < endT - 1) {
                const sampleX = edge.n1.x + (dx / len) * currentT + nx * dir * 15;
                const sampleY = edge.n1.y + (dy / len) * currentT + ny * dir * 15;
                let terrain = getTerrain(sampleX, sampleY, seedOffset);
                if (edge.type === 'park_path') {
                    terrain = 'PARK';
                }

                let bWidth, bDepth, gap, type;
                if (terrain === 'WATER') { currentT += 5; continue; }
                else if (terrain === 'CITY') {
                    bWidth = 3 + Math.random() * 6; bDepth = 4 + Math.random() * 7; gap = 0.2;
                    type = Math.random() < 0.15 ? 'PARKING_LOT' : 'COMMERCIAL';
                }
                else if (terrain === 'SUBURB') {
                    bWidth = 3 + Math.random() * 3; bDepth = 4 + Math.random() * 4; gap = 0.8;
                    type = 'HOUSE';
                }
                else if (terrain === 'PARK') {
                    bWidth = 2 + Math.random() * 3; bDepth = bWidth; gap = 2;
                    type = 'TREE';
                }

                if (currentT + bWidth > endT) bWidth = endT - currentT;
                if (bWidth < 2) { currentT += 1; continue; }

                const distToCenter = roadHalfWidth + sidewalk + (bDepth / 2);
                const bx = edge.n1.x + (dx / len) * (currentT + bWidth / 2) + nx * dir * distToCenter;
                const by = edge.n1.y + (dy / len) * (currentT + bWidth / 2) + ny * dir * distToCenter;

                if (bx <= minX || bx >= maxX || by <= minY || by >= maxY) { currentT += bWidth; continue; }

                const generatedParts = [];
                const basePart = { x: bx, y: by, w: bWidth, h: bDepth, angle, type };

                if (type === 'PARKING_LOT') {
                    const cars = [];
                    const rows = Math.floor((bDepth - 2) / 3);
                    const cols = Math.floor((bWidth - 2) / 2.5);
                    const startX = -((cols - 1) * 2.5) / 2;
                    const startY = -((rows - 1) * 3) / 2;
                    for (let r = 0; r < rows; r++) {
                        for (let c = 0; c < cols; c++) {
                            if (Math.random() < 0.6) {
                                cars.push({
                                    lx: startX + c * 2.5, ly: startY + r * 3,
                                    color: ['#ef4444', '#3b82f6', '#10b981', '#f59e0b', '#94a3b8', '#ffffff'][Math.floor(Math.random() * 6)]
                                });
                            }
                        }
                    }
                    basePart.cars = cars;
                    generatedParts.push(basePart);
                } else {
                    generatedParts.push(basePart);
                    if (type === 'COMMERCIAL' && Math.random() < 0.3) {
                        const wingW = 2 + Math.random() * 3;
                        const wingH = bDepth * (0.4 + Math.random() * 0.4);
                        const lx = (bWidth / 2 + wingW / 2) * (Math.random() > 0.5 ? 1 : -1);
                        const ly = (bDepth / 2 - wingH / 2) * (Math.random() > 0.5 ? 1 : -1);
                        generatedParts.push({
                            x: bx + Math.cos(angle) * lx - Math.sin(angle) * ly,
                            y: by + Math.sin(angle) * lx + Math.cos(angle) * ly,
                            w: wingW, h: wingH, angle, type
                        });
                    }
                }

                let overlap = false;
                for (let part of generatedParts) {
                    const nearbyEdges = getNearbyEdges(chunk, part);
                    for (const e of nearbyEdges) {
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
                    for (const eb of nearbyBuildings) {
                        if (obbVsObb(part, eb, pad)) { overlap = true; break; }
                    }
                    if (overlap) break;
                }

                if (!overlap) {
                    for (let part of generatedParts) {
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
                const [cx, cy] = nextKey.split(',').map(Number);
                if (!world.generatedChunks.has(nextKey)) {
                    const newChunk = initChunkData(cx, cy);
                    if (newChunk) {
                        world.activeChunk = newChunk;
                    }
                }
            }
            return;
        }

        const chunk = world.activeChunk;
        const seedOffset = world.seedOffset;

        const minX = chunk.cx * CHUNK_SIZE; const maxX = minX + CHUNK_SIZE;
        const minY = chunk.cy * CHUNK_SIZE; const maxY = minY + CHUNK_SIZE;

        if (chunk.phase === 'GROWING') {
            if (chunk.agents.length < 300) {
                for (let tries = 0; tries < 6; tries++) {
                    const rx = minX + Math.random() * CHUNK_SIZE;
                    const ry = minY + Math.random() * CHUNK_SIZE;
                    const t = getTerrain(rx, ry, seedOffset);
                    if (t === 'CITY' || t === 'SUBURB') {
                        const nearest = findNearestNode(chunk, rx, ry, 50, null, 0);
                        if (!nearest) {
                            const newNode = addNode(chunk, rx, ry, 0);
                            for (let i = 0; i < 4; i++) {
                                chunk.agents.push({ node: newNode, angle: (i * Math.PI) / 2 + Math.random(), type: 'street', life: 120, z: 0 });
                            }
                        }
                    }
                }
            }

            let iterations = 0;
            while (chunk.agents.length > 0 && iterations < GROWTH_SPEED) {
                iterations++;
                const idx = Math.floor(Math.random() * chunk.agents.length);
                const agent = chunk.agents[idx];
                agent.life--;

                let forceMerge = agent.life <= 0;

                if (forceMerge && agent.type === 'ramp') {
                    agent.type = 'street';
                    agent.life = 100;
                    forceMerge = false;
                    chunk.agents.push({ node: agent.node, angle: agent.angle + Math.PI / 2, type: 'street', life: 80, z: 0 });
                    chunk.agents.push({ node: agent.node, angle: agent.angle - Math.PI / 2, type: 'street', life: 80, z: 0 });

                    const cityNode = findNearestNode(chunk, agent.node.x, agent.node.y, 120, agent.node, 0);
                    if (cityNode) {
                        chunk.edges.push({ n1: agent.node, n2: cityNode, type: 'street', isBridge: false });
                    }
                }

                const stepAmount = agent.type === 'alley' ? ALLEY_STEP : STEP_SIZE;
                let nx = agent.node.x + Math.cos(agent.angle) * stepAmount;
                let ny = agent.node.y + Math.sin(agent.angle) * stepAmount;

                const isOutOfBounds = (nx < minX || nx >= maxX || ny < minY || ny >= maxY);
                if (!forceMerge && isOutOfBounds) {
                    if (agent.type === 'highway') {
                        const zLevel = agent.z || 0;
                        const boundTerrain = getTerrain(nx, ny, seedOffset);
                        const isOverWater = boundTerrain === 'WATER';

                        const nextNode = addNode(chunk, nx, ny, zLevel);
                        chunk.edges.push({ n1: agent.node, n2: nextNode, type: agent.type, isBridge: agent.wasBridge || isOverWater });

                        let nextCx = chunk.cx; let nextCy = chunk.cy;
                        if (nx < minX) nextCx--; else if (nx >= maxX) nextCx++;
                        if (ny < minY) nextCy--; else if (ny >= maxY) nextCy++;
                        const nextKey = `${nextCx},${nextCy}`;

                        if (worldRef.current.generatedChunks.has(nextKey)) {
                            const targetChunk = worldRef.current.generatedChunks.get(nextKey);
                            // Increase search radius significantly so bridges successfully dock into the existing city network
                            const targetNode = findNearestNode(targetChunk, nx, ny, 1000, null, zLevel);
                            if (targetNode) {
                                chunk.edges.push({ n1: nextNode, n2: targetNode, type: 'highway', isBridge: isOverWater });
                            }
                        } else {
                            // Always spawn the chunk, but mark the bridge so it steers toward the island
                            worldRef.current.borderNodes.push({ x: nx, y: ny, angle: agent.angle, type: agent.type, z: zLevel, wasBridge: isOverWater, isCardinal: agent.isCardinal, hasSpawnedPerpendiculars: false, isArrivingBridge: isOverWater });
                            if (!worldRef.current.chunkQueue.includes(nextKey)) {
                                worldRef.current.chunkQueue.push(nextKey);
                            }
                        }
                    }
                    chunk.agents.splice(idx, 1);
                    continue;
                }

                const nextTerrain = getTerrain(nx, ny, seedOffset);
                let isBridge = false;

                if (!forceMerge && agent.type === 'causeway' && nextTerrain !== 'WATER') {
                    agent.type = 'street';
                    agent.life = 60; // Refresh life when hitting a new island fragment
                } else if (!forceMerge && agent.type === 'street' && nextTerrain === 'SUBURB') agent.type = 'suburb_road';
                else if (!forceMerge && agent.type === 'suburb_road' && nextTerrain === 'CITY') agent.type = 'street';

                if (!forceMerge && nextTerrain === 'WATER') {
                    if (agent.type === 'ramp') {
                        chunk.agents.splice(idx, 1);
                        continue;
                    }
                    else if (agent.type === 'highway' || agent.type === 'causeway') { isBridge = true; agent.wasBridge = true; }
                    else if (agent.type === 'alley') { forceMerge = true; }
                    else if (agent.type === 'street' || agent.type === 'suburb_road' || agent.type === 'coast' || agent.type === 'park_path') {
                        // Raycast ahead to ensure there is actually land to connect to within range
                        let hitLand = false;
                        let hasCity = false;
                        if (agent.type === 'street' || agent.type === 'suburb_road') {
                            for (let i = 1; i <= 180; i++) {
                                const rx = agent.node.x + Math.cos(agent.angle) * (STEP_SIZE * i);
                                const ry = agent.node.y + Math.sin(agent.angle) * (STEP_SIZE * i);
                                // Causeways are local! Don't target land in adjacent chunks, or they'll die at the border.
                                if (rx < minX || rx >= maxX || ry < minY || ry >= maxY) break;
                                const terrain = getTerrain(rx, ry, seedOffset);

                                if (terrain !== 'WATER') {

                                    hitLand = true;

                                    if (
                                        terrain === 'CITY' ||
                                        terrain === 'SUBURB'
                                    ) {
                                        hasCity = true;
                                    }

                                    break;
                                }
                            }
                        }

                        // Spawn causeway ONLY if it's guaranteed to reach land (no dead ends in the water)
                        if (hitLand && hasCity) {
                            let alreadyExists = false;

                            for (const c of chunk.causewayTargets) {

                                const dx = c.x - agent.node.x;
                                const dy = c.y - agent.node.y;

                                if (Math.hypot(dx, dy) < 120) {
                                    alreadyExists = true;
                                    break;
                                }
                            }

                            if (!alreadyExists) {

                                chunk.causewayTargets.push({
                                    x: agent.node.x,
                                    y: agent.node.y
                                });

                                chunk.agents.push({
                                    node: agent.node,
                                    angle: agent.angle,
                                    type: 'causeway',
                                    life: 100,
                                    z: 0,
                                    wasBridge: true
                                });
                            }

                            forceMerge = true;
                        } else {
                            agent.type = 'coast';
                            const eps = 2;
                            const vRight = fbm(agent.node.x + eps + seedOffset, agent.node.y + seedOffset);
                            const vTop = fbm(agent.node.x + seedOffset, agent.node.y + eps + seedOffset);
                            const vCenter = fbm(agent.node.x + seedOffset, agent.node.y + seedOffset);
                            let coastAngle = Math.atan2(-(vRight - vCenter), vTop - vCenter);
                            let diff = coastAngle - agent.angle;
                            while (diff <= -Math.PI) diff += Math.PI * 2;
                            while (diff > Math.PI) diff -= Math.PI * 2;
                            if (Math.abs(diff) > Math.PI / 2) coastAngle += Math.PI;
                            agent.angle = coastAngle;
                            nx = agent.node.x + Math.cos(agent.angle) * (STEP_SIZE * 0.8);
                            ny = agent.node.y + Math.sin(agent.angle) * (STEP_SIZE * 0.8);
                            if (getTerrain(nx, ny, seedOffset) === 'WATER' || nx < minX || nx > maxX || ny < minY || ny > maxY) forceMerge = true;
                            else agent.life = Math.max(agent.life, 30);
                        }
                    }
                } else if (!forceMerge && nextTerrain === 'PARK') {
                    if (agent.type === 'street' || agent.type === 'suburb_road') agent.type = 'park_path';
                    if (agent.type === 'alley') forceMerge = true;
                } else if (!forceMerge && agent.type === 'alley' && nextTerrain !== 'CITY') {
                    forceMerge = true;
                }

                const zLevel = agent.z || 0;

                if (forceMerge) {
                    const mergeDist = agent.type === 'alley' ? 40 : agent.type === 'ramp' ? 80 : 300;
                    let target = findNearestNode(chunk, agent.node.x, agent.node.y, mergeDist, agent.node, zLevel);

                    if (target && agent.type === 'ramp') {
                        const targetTerrain = getTerrain(target.x, target.y, seedOffset);
                        if (targetTerrain === 'PARK' || targetTerrain === 'WATER' || isInvalidRampConnection(chunk, target)) {
                            target = null;
                        }
                    }

                    if (target && (agent.type === 'alley' || agent.type === 'park_path')) {
                        if (isRampNode(chunk, target)) {
                            target = null;
                        }
                    }

                    if (target && (agent.type === 'highway' || agent.type === 'causeway' || !checkWaterCrossing(agent.node, target, seedOffset))) {
                        chunk.edges.push({ n1: agent.node, n2: target, type: agent.type, isBridge: (agent.type === 'highway' || agent.type === 'causeway') ? isBridge : false });
                    }
                    chunk.agents.splice(idx, 1);
                    continue;
                }

                const mergeSens = agent.type === 'alley' ? MERGE_RADIUS * 0.8 : MERGE_RADIUS;
                // Bridges over water should not merge into each other, forming T-junctions. Let them cross to land!
                let nearbyNode = isBridge ? null : findNearestNode(chunk, nx, ny, mergeSens, agent.node, zLevel);

                if (nearbyNode && agent.type === 'ramp') {
                    const targetTerrain = getTerrain(nearbyNode.x, nearbyNode.y, seedOffset);
                    if (targetTerrain === 'PARK' || targetTerrain === 'WATER' || isInvalidRampConnection(chunk, nearbyNode)) {
                        nearbyNode = null;
                    }
                }

                if (nearbyNode && (agent.type === 'alley' || agent.type === 'park_path')) {
                    if (isRampNode(chunk, nearbyNode)) {
                        nearbyNode = null;
                    }
                }

                if (nearbyNode && agent.type !== 'highway' && checkWaterCrossing(agent.node, nearbyNode, seedOffset)) {
                    chunk.agents.splice(idx, 1);
                    continue;
                }

                let nextNode = nearbyNode || addNode(chunk, nx, ny, zLevel);
                chunk.edges.push({ n1: agent.node, n2: nextNode, type: agent.type, isBridge: (agent.type === 'highway' || agent.type === 'causeway') ? isBridge : false });
                if (nearbyNode) { chunk.agents.splice(idx, 1); continue; }

                agent.node = nextNode;

                if (agent.type === 'highway') {
                    // THE CENTRAL GRID BRANCHING LOGIC
                    if (agent.isCardinal && !agent.hasSpawnedPerpendiculars) {
                        const centerX = chunk.cx * CHUNK_SIZE + CHUNK_SIZE / 2;
                        const centerY = chunk.cy * CHUNK_SIZE + CHUNK_SIZE / 2;
                        const distToCenter = Math.hypot(agent.node.x - centerX, agent.node.y - centerY);

                        if (distToCenter < STEP_SIZE * 2) {
                            agent.hasSpawnedPerpendiculars = true;

                            chunk.agents.push({ node: agent.node, angle: agent.angle + Math.PI / 2, type: 'highway', life: 1800, z: agent.z, wasBridge: false, isCardinal: true, hasSpawnedPerpendiculars: true });
                            chunk.agents.push({ node: agent.node, angle: agent.angle - Math.PI / 2, type: 'highway', life: 1800, z: agent.z, wasBridge: false, isCardinal: true, hasSpawnedPerpendiculars: true });

                            chunk.agents.push({ node: agent.node, angle: agent.angle + Math.PI / 4, type: 'highway', life: 120, z: agent.z, wasBridge: false, isCardinal: false });
                            chunk.agents.push({ node: agent.node, angle: agent.angle - Math.PI / 4, type: 'highway', life: 120, z: agent.z, wasBridge: false, isCardinal: false });

                            chunk.agents.push({ node: agent.node, angle: agent.angle + Math.PI / 2, type: 'ramp', life: 5, z: 0 });
                            chunk.agents.push({ node: agent.node, angle: agent.angle - Math.PI / 2, type: 'ramp', life: 5, z: 0 });
                        }
                    }

                    if (agent.isCardinal) {
                        if (!isBridge && Math.random() < 0.05) {
                            const turnAngle = (Math.random() > 0.5 ? Math.PI / 2 : -Math.PI / 2);
                            chunk.agents.push({ node: nextNode, angle: agent.angle + turnAngle, type: 'ramp', life: 4, z: 0 });
                        }
                    } else {
                        agent.angle += (Math.random() - 0.5) * (isBridge ? 0.0 : 0.3);
                    }

                    // Steer arriving bridges toward the island to guarantee a perfect connection
                    if (agent.isArrivingBridge && isBridge) {
                        const centerX = chunk.cx * CHUNK_SIZE + CHUNK_SIZE / 2;
                        const centerY = chunk.cy * CHUNK_SIZE + CHUNK_SIZE / 2;
                        const targetAngle = Math.atan2(centerY - ny, centerX - nx);
                        let diff = targetAngle - agent.angle;
                        while (diff > Math.PI) diff -= Math.PI * 2;
                        while (diff < -Math.PI) diff += Math.PI * 2;
                        agent.angle += diff * 0.15; // Aggressively curve toward the center to avoid entering orbit
                    }
                    if (!isBridge) {
                        agent.isArrivingBridge = false; // Disable steering once it hits land
                    }

                    if (agent.wasBridge && !isBridge) {
                        let cityNode = findNearestNode(chunk, nx, ny, 150, nextNode, zLevel);
                        if (cityNode && isInvalidRampConnection(chunk, cityNode)) {
                            cityNode = null;
                        }
                        if (cityNode) chunk.edges.push({ n1: nextNode, n2: cityNode, type: 'ramp', isBridge: false });
                        chunk.agents.push({ node: nextNode, angle: agent.angle + Math.PI / 3, type: 'ramp', life: 6, z: 0 });
                        chunk.agents.push({ node: nextNode, angle: agent.angle - Math.PI / 3, type: 'ramp', life: 6, z: 0 });
                    }
                    agent.wasBridge = isBridge;

                    if (!isBridge && !agent.isCardinal && Math.random() < 0.25) {
                        const turnAngle = (Math.random() > 0.5 ? Math.PI / 2 : -Math.PI / 2);
                        chunk.agents.push({ node: nextNode, angle: agent.angle + turnAngle, type: 'ramp', life: 4, z: 0 });
                    }
                } else if (agent.type === 'ramp') {
                    agent.angle += (Math.random() - 0.5) * 0.1;
                } else if (agent.type === 'street' || agent.type === 'suburb_road') {
                    if (Math.random() < 0.1) agent.angle += (Math.random() > 0.5 ? Math.PI / 4 : -Math.PI / 4);
                    if (Math.random() < 0.45) {
                        chunk.agents.push({ node: nextNode, angle: agent.angle + (Math.random() > 0.5 ? Math.PI / 2 : -Math.PI / 2), type: agent.type, life: 80, z: 0 });
                    }
                    if (Math.random() < 0.15 && nextTerrain === 'CITY') {
                        chunk.agents.push({ node: nextNode, angle: agent.angle + (Math.random() > 0.5 ? Math.PI / 2 : -Math.PI / 2), type: 'alley', life: 25, z: 0 });
                    }
                } else if (agent.type === 'alley' || agent.type === 'coast') {
                    agent.angle += (Math.random() - 0.5) * 0.1;
                } else if (agent.type === 'park_path') {
                    agent.angle += (Math.random() - 0.5) * 1.2;
                    if (Math.random() < 0.15) {
                        chunk.agents.push({ node: nextNode, angle: agent.angle + (Math.random() > 0.5 ? Math.PI / 2 : -Math.PI / 2), type: 'park_path', life: 30, z: 0 });
                    }
                    if (getTerrain(nx, ny, seedOffset) !== 'PARK') forceMerge = true;
                }
            }

            if (chunk.agents.length === 0) {
                const uniqueEdges = []; const edgeSet = new Set();
                for (const e of chunk.edges) {
                    const key = e.n1.id < e.n2.id ? `${e.n1.id}-${e.n2.id}` : `${e.n2.id}-${e.n1.id}`;
                    if (!edgeSet.has(key) && e.n1.id !== e.n2.id) { edgeSet.add(key); uniqueEdges.push(e); }
                }
                let cEdges = uniqueEdges;
                let changed = true;

                // BULLETPROOF PRUNER
                while (changed) {
                    changed = false;
                    const degrees = new Map();
                    cEdges.forEach(e => {
                        degrees.set(e.n1.id, (degrees.get(e.n1.id) || 0) + 1);
                        degrees.set(e.n2.id, (degrees.get(e.n2.id) || 0) + 1);
                    });
                    cEdges = cEdges.filter(e => {
                        if (e.type === 'highway') return true;

                        const d1 = degrees.get(e.n1.id); const d2 = degrees.get(e.n2.id);
                        const isB1 = (e.n1.x <= minX + 5 || e.n1.x >= maxX - 5 || e.n1.y <= minY + 5 || e.n1.y >= maxY - 5);
                        const isB2 = (e.n2.x <= minX + 5 || e.n2.x >= maxX - 5 || e.n2.y <= minY + 5 || e.n2.y >= maxY - 5);

                        if ((d1 === 1 && !isB1) || (d2 === 1 && !isB2)) {
                            changed = true; return false;
                        }
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
                const edge = chunk.edges[chunk.edgeProcessIndex];
                tryPlaceBuildingsAlongEdge(chunk, edge, seedOffset);
                chunk.edgeProcessIndex++;
                processed++;
            }
            if (chunk.edgeProcessIndex >= chunk.edges.length) {
                chunk.phase = 'FENCES';
            }
        }

        if (chunk.phase === 'FENCES') {
            const BLDGS_PER_FRAME = 120;
            let processed = 0;
            while (processed < BLDGS_PER_FRAME && chunk.fenceProcessIndex < chunk.buildings.length) {
                const b1 = chunk.buildings[chunk.fenceProcessIndex];
                if (b1.type === 'COMMERCIAL' || b1.type === 'HOUSE') {
                    b1.fenceCount = b1.fenceCount || 0;
                    b1.connectedTo = b1.connectedTo || new Set();
                    if (b1.fenceCount < 2) {
                        const neighbors = getNearbyBuildings(chunk, b1);
                        const validNeighbors = [];
                        for (const b2 of neighbors) {
                            if (b1 === b2 || (b2.type !== 'COMMERCIAL' && b2.type !== 'HOUSE') || b1.connectedTo.has(b2)) continue;
                            const dx = b2.x - b1.x; const dy = b2.y - b1.y;
                            const dist = Math.hypot(dx, dy);
                            if (dist < 40) validNeighbors.push({ b2, dist, dx, dy });
                        }
                        validNeighbors.sort((a, b) => a.dist - b.dist);

                        for (const { b2, dist, dx, dy } of validNeighbors) {
                            if (b1.fenceCount >= 2) break;
                            b2.fenceCount = b2.fenceCount || 0; b2.connectedTo = b2.connectedTo || new Set();
                            if (b2.fenceCount >= 2) continue;
                            const fenceObb = { x: (b1.x + b2.x) / 2, y: (b1.y + b2.y) / 2, w: dist, h: 1, angle: Math.atan2(dy, dx) };

                            let crossesRoad = false;
                            for (const e of getNearbyEdges(chunk, fenceObb)) {
                                let hw = 1.5; if (e.type === 'highway') hw = 4; else if (e.type === 'park_path' || e.type === 'alley') hw = 0.5;
                                const rObb = roadSegmentToObb(e.n1.x, e.n1.y, e.n2.x, e.n2.y, hw + 1);
                                if (rObb && obbVsObb(fenceObb, rObb, 0)) { crossesRoad = true; break; }
                            }
                            if (!crossesRoad) {
                                let crossesFence = false;
                                for (const f of chunk.fences) {
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
            if (chunk.fenceProcessIndex >= chunk.buildings.length) {
                chunk.phase = 'DONE';
            }
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
        const ctx = canvas?.getContext('2d');
        const wrapper = wrapperRef.current;
        if (!ctx || !wrapper) return;

        if (canvas.width !== wrapper.clientWidth || canvas.height !== wrapper.clientHeight) {
            canvas.width = wrapper.clientWidth; canvas.height = wrapper.clientHeight;
        }

        const { x: camX, y: camY } = cameraRef.current;
        const width = canvas.width; const height = canvas.height;
        const zoom = zoomRef.current;

        // Water base color
        // Rich sky-blue water backdrop
        ctx.fillStyle = '#bae6fd';
        ctx.fillRect(0, 0, width, height);

        const startX = Math.floor(camX / CHUNK_SIZE);
        const endX = Math.floor((camX + width / zoom) / CHUNK_SIZE);
        const startY = Math.floor(camY / CHUNK_SIZE);
        const endY = Math.floor((camY + height / zoom) / CHUNK_SIZE);

        const world = worldRef.current;

        ctx.save();
        ctx.scale(zoom, zoom);
        ctx.translate(-camX, -camY);

        const drawChunkData = (chunk) => {
            ctx.lineCap = 'butt'; ctx.lineJoin = 'miter';

            const drawGroup = (filterFn, color, width) => {
                ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width;
                for (const e of chunk.edges) {
                    if (filterFn(e)) { ctx.moveTo(e.n1.x, e.n1.y); ctx.lineTo(e.n2.x, e.n2.y); }
                }
                ctx.stroke();
            };

            const drawGroupOffset = (filterFn, color, width, dx, dy) => {
                ctx.beginPath(); ctx.strokeStyle = color; ctx.lineWidth = width;
                for (const e of chunk.edges) {
                    if (filterFn(e)) { ctx.moveTo(e.n1.x + dx, e.n1.y + dy); ctx.lineTo(e.n2.x + dx, e.n2.y + dy); }
                }
                ctx.stroke();
            };

            // Ground Road Casings (Dark borders)
            drawGroup(e => !e.isBridge && (e.type === 'street' || e.type === 'coast'), '#1e293b', 3.8);
            drawGroup(e => !e.isBridge && e.type === 'suburb_road', '#334155', 3.0);
            drawGroup(e => !e.isBridge && e.type === 'ramp', '#1e293b', 3.8);

            // Ground Road Fills (Colored interior)
            drawGroup(e => !e.isBridge && (e.type === 'street' || e.type === 'coast'), '#ffffff', 2.2); // White main streets
            drawGroup(e => !e.isBridge && e.type === 'suburb_road', '#cbd5e1', 1.6); // Light slate suburb roads
            drawGroup(e => !e.isBridge && e.type === 'ramp', '#facc15', 2.2); // Yellow/amber ramps

            // Alleys: Dashed dark slate lines
            ctx.save();
            ctx.setLineDash([2, 3]);
            drawGroup(e => !e.isBridge && e.type === 'alley', '#475569', 1.2);
            ctx.restore();

            // Park trails: Dashed green paths
            ctx.save();
            ctx.setLineDash([3, 2]);
            drawGroup(e => !e.isBridge && e.type === 'park_path', '#059669', 1.4);
            ctx.restore();

            // Parking bases / backdrops
            for (const b of chunk.buildings) {
                if (b.type === 'PARKING_LOT') {
                    ctx.save();
                    ctx.translate(b.x, b.y); ctx.rotate(b.angle);
                    ctx.fillStyle = '#e2e8f0'; // clean light grey asphalt
                    ctx.fillRect(-b.w / 2, -b.h / 2, b.w, b.h);
                    ctx.strokeStyle = '#cbd5e1'; ctx.lineWidth = 0.6;
                    ctx.strokeRect(-b.w / 2, -b.h / 2, b.w, b.h);

                    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 0.5; ctx.beginPath();
                    for (let car of b.cars) {
                        ctx.moveTo(car.lx - 1.25, car.ly - 1.5); ctx.lineTo(car.lx - 1.25, car.ly + 1.5);
                    }
                    ctx.stroke();
                    for (let car of b.cars) {
                        ctx.fillStyle = car.color; ctx.fillRect(car.lx - 0.8, car.ly - 1.2, 1.6, 2.4);
                    }
                    ctx.restore();
                }
            }

            // Fences (crisp outline)
            if (chunk.fences.length > 0) {
                ctx.beginPath(); ctx.strokeStyle = '#94a3b8'; ctx.lineWidth = 1.0;
                for (const f of chunk.fences) { ctx.moveTo(f.x1, f.y1); ctx.lineTo(f.x2, f.y2); }
                ctx.stroke();
            }

            // Buildings / Trees with premium architectural colors
            for (const b of chunk.buildings) {
                if (b.type === 'PARKING_LOT') continue;
                ctx.save();
                ctx.translate(b.x, b.y); ctx.rotate(b.angle);
                if (b.type === 'COMMERCIAL') {
                    // Modern sky-blue/glass commercial buildings
                    ctx.fillStyle = '#e0f2fe'; // sky-100
                    ctx.strokeStyle = '#0284c7'; // sky-600
                    ctx.lineWidth = 0.8;
                    ctx.fillRect(-b.w / 2, -b.h / 2, b.w, b.h);
                    ctx.strokeRect(-b.w / 2, -b.h / 2, b.w, b.h);
                    // Glass reflection highlight
                    ctx.fillStyle = '#ffffff';
                    ctx.fillRect(-b.w / 2 + 1, -b.h / 2 + 1, b.w / 3.5, b.h - 2);
                } else if (b.type === 'HOUSE') {
                    // Terracotta roof tiles style
                    ctx.fillStyle = '#fed7aa'; // orange-200 (warm terracotta)
                    ctx.strokeStyle = '#c2410c'; // orange-700
                    ctx.lineWidth = 0.8;
                    ctx.fillRect(-b.w / 2, -b.h / 2, b.w, b.h);
                    ctx.strokeRect(-b.w / 2, -b.h / 2, b.w, b.h);
                    // Roof ridge line
                    ctx.beginPath();
                    ctx.strokeStyle = '#9a3412'; // orange-800
                    ctx.lineWidth = 0.8;
                    ctx.moveTo(-b.w / 2, 0); ctx.lineTo(b.w / 2, 0);
                    ctx.stroke();
                } else if (b.type === 'TREE') {
                    ctx.fillStyle = '#86efac';
                    ctx.strokeStyle = '#15803d';
                    ctx.lineWidth = 0.8;
                    ctx.beginPath(); ctx.arc(0, 0, Math.max(1.2, b.w / 2), 0, Math.PI * 2); ctx.fill(); ctx.stroke();
                }
                ctx.restore();
            }

            // Highways and Bridges (drawn on top with round lineCap for premium feel)
            ctx.lineCap = 'round'; ctx.lineJoin = 'round';

            // 1. Draw highway and causeway shadows (shifted offsets)
            drawGroupOffset(e => e.type === 'highway', 'rgba(15, 23, 42, 0.25)', 5.5, 2.5, 4.0);
            drawGroupOffset(e => e.type === 'causeway', 'rgba(78, 53, 15, 0.3)', 4.0, 2.0, 3.5);

            // 2. Draw highway and causeway casings (thick dark outline)
            drawGroup(e => e.type === 'highway', '#0f172a', 6.0);
            drawGroup(e => e.type === 'causeway', '#78350f', 4.5);

            // 3. Draw highway and causeway fills (vibrant colors)
            drawGroup(e => e.type === 'highway', '#f59e0b', 3.8); // Vibrant orange/gold highways
            drawGroup(e => e.type === 'causeway', '#d97706', 2.5); // Warm terracotta causeways
        };

        // Draw cached terrain tiles
        for (let cx = startX; cx <= endX; cx++) {
            for (let cy = startY; cy <= endY; cy++) {
                const key = `${cx},${cy}`;
                const isReached = world.generatedChunks.has(key) || world.activeChunk?.key === key || world.chunkQueue.includes(key);

                if (isReached) {
                    if (!world.terrainCache.has(key)) {
                        generateChunkTerrain(cx, cy);
                    }
                    const tCanvas = world.terrainCache.get(key);
                    if (tCanvas) {
                        ctx.drawImage(tCanvas, cx * CHUNK_SIZE, cy * CHUNK_SIZE);
                    }
                }
            }
        }

        // Draw vector roads, buildings, etc.
        for (let cx = startX; cx <= endX; cx++) {
            for (let cy = startY; cy <= endY; cy++) {
                const key = `${cx},${cy}`;
                if (world.generatedChunks.has(key)) {
                    drawChunkData(world.generatedChunks.get(key));
                }
                if (world.activeChunk?.key === key) {
                    drawChunkData(world.activeChunk);
                }
            }
        }
        ctx.restore();

        let activeLabel = 'Idle (Pan to discover)';
        if (world.activeChunk) {
            const ac = world.activeChunk;
            activeLabel = ac.phase === 'GROWING' ? `Growing Roads`
                : ac.phase === 'BUILDINGS' ? `Districts (${Math.floor((ac.edgeProcessIndex / ac.edges.length) * 100)}%)`
                    : `Fences (${Math.floor((ac.fenceProcessIndex / ac.buildings.length) * 100)}%)`;
        }

        setUiStats({
            phase: activeLabel,
            activeIsland: world.activeChunk ? world.activeChunk.key : null,
            queued: world.chunkQueue.length,
            nodes: world.globalStats.nodes + (world.activeChunk ? world.activeChunk.nodes.length : 0),
            edges: world.globalStats.edges + (world.activeChunk ? world.activeChunk.edges.length : 0),
            buildings: world.globalStats.buildings + (world.activeChunk ? world.activeChunk.buildings.filter(b => b.type !== 'PARKING_LOT').length : 0),
            parking: world.globalStats.parking + (world.activeChunk ? world.activeChunk.buildings.filter(b => b.type === 'PARKING_LOT').length : 0),
            fences: world.globalStats.fences + (world.activeChunk ? world.activeChunk.fences.length : 0)
        });

    }, []);

    const animate = useCallback(() => {
        if (isRunning) stepSimulation();
        drawCity();
        requestRef.current = requestAnimationFrame(animate);
    }, [isRunning, stepSimulation, drawCity]);

    const changeZoom = useCallback((newZoom) => {
        newZoom = Math.max(0.15, Math.min(4.0, newZoom));
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

    // Handle wheel zoom centered on cursor
    useEffect(() => {
        const wrapper = wrapperRef.current;
        if (!wrapper) return;

        const handleWheel = (e) => {
            e.preventDefault();
            const rect = wrapper.getBoundingClientRect();
            const mouseXScreen = e.clientX - rect.left;
            const mouseYScreen = e.clientY - rect.top;

            const zoom = zoomRef.current;
            const mouseXWorld = cameraRef.current.x + mouseXScreen / zoom;
            const mouseYWorld = cameraRef.current.y + mouseYScreen / zoom;

            const zoomFactor = 1.1;
            let newZoom = e.deltaY < 0 ? zoom * zoomFactor : zoom / zoomFactor;
            newZoom = Math.max(0.15, Math.min(4.0, newZoom));

            cameraRef.current.x = mouseXWorld - mouseXScreen / newZoom;
            cameraRef.current.y = mouseYWorld - mouseYScreen / newZoom;
            zoomRef.current = newZoom;
            setZoomState(newZoom);
            drawCity();
        };

        wrapper.addEventListener('wheel', handleWheel, { passive: false });
        return () => {
            wrapper.removeEventListener('wheel', handleWheel);
        };
    }, [drawCity]);

    const handleMouseDown = (e) => {
        dragRef.current = { isDragging: true, lastX: e.clientX, lastY: e.clientY };
    };
    const handleMouseMove = (e) => {
        if (!dragRef.current.isDragging) return;
        const zoom = zoomRef.current;
        const dx = (e.clientX - dragRef.current.lastX) / zoom;
        const dy = (e.clientY - dragRef.current.lastY) / zoom;
        cameraRef.current.x -= dx;
        cameraRef.current.y -= dy;
        dragRef.current.lastX = e.clientX;
        dragRef.current.lastY = e.clientY;
    };
    const handleMouseUp = () => { dragRef.current.isDragging = false; };

    return (
        <div className="flex flex-col h-screen w-full bg-slate-50 font-sans text-slate-800 select-none">
            <div className="flex flex-wrap items-center justify-between p-4 bg-white border-b border-slate-200 z-10 relative shrink-0 shadow-sm gap-4">
                <div>
                    <h1 className="text-lg font-semibold text-slate-800 tracking-tight">
                        Procedural City Plan
                    </h1>
                    <div className="flex flex-col gap-1.5 mt-1.5 text-xs text-slate-500">
                        {/* Roads Row */}
                        <div className="flex flex-wrap gap-x-4 gap-y-1 items-center">
                            <span className="font-semibold text-slate-700 mr-1">Roads:</span>
                            <span className="flex items-center gap-1.5">
                                <span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#0f172a] bg-[#f59e0b]"></span>
                                Highway / Bridge
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#78350f] bg-[#d97706]"></span>
                                Causeway
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#1e293b] bg-[#ffffff]"></span>
                                Main Street
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="inline-flex items-center justify-center w-5 h-2.5 rounded border border-[#334155] bg-[#cbd5e1]"></span>
                                Suburb Road
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-5 h-0.5 border-t border-dashed border-[#475569]"></span>
                                Alley
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-5 h-0.5 border-t border-dashed border-[#059669]"></span>
                                Park Trail
                            </span>
                        </div>
                        {/* Zones & Structures Row */}
                        <div className="flex flex-wrap gap-x-4 gap-y-1 items-center">
                            <span className="font-semibold text-slate-700 mr-1">Zones:</span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#bae6fd] border border-[#cbd5e1]"></span>
                                Water
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#e2e8f0] border border-[#cbd5e1]"></span>
                                City Center
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#f8fafc] border border-[#cbd5e1]"></span>
                                Suburbs
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#dcfce7] border border-[#cbd5e1]"></span>
                                Park / Forest
                            </span>

                            <span className="font-semibold text-slate-700 ml-2 mr-1">Structures:</span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#e0f2fe] border border-[#0284c7]"></span>
                                Commercial
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#fed7aa] border border-[#c2410c]"></span>
                                House
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-2.5 h-2.5 rounded-full bg-[#86efac] border border-[#15803d]"></span>
                                Tree
                            </span>
                            <span className="flex items-center gap-1.5">
                                <span className="w-3.5 h-2.5 rounded bg-[#e2e8f0] border border-[#cbd5e1] flex items-center justify-center"><span className="text-[7.5px] text-[#64748b] font-bold font-sans leading-none">P</span></span>
                                Parking
                            </span>
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
                        <button
                            onClick={() => setIsRunning(!isRunning)}
                            className="px-3 py-1.5 rounded bg-white hover:bg-slate-50 border border-slate-200 text-xs font-medium text-slate-700 transition-colors shadow-sm cursor-pointer"
                        >
                            {isRunning ? 'Pause Engine' : 'Resume Engine'}
                        </button>
                        <button
                            onClick={() => { initWorld(); setIsRunning(true); }}
                            className="px-3 py-1.5 rounded bg-slate-900 hover:bg-slate-800 text-white text-xs font-medium transition-colors shadow-sm cursor-pointer"
                        >
                            Wipe Earth
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

                {/* Floating Navigation Instructions */}
                <div className="absolute bottom-4 left-4 text-[10px] font-mono text-slate-400 pointer-events-none bg-white/80 backdrop-blur-sm px-2 py-1 rounded border border-slate-200 shadow-sm">
                    Drag to Pan • Scroll to Zoom
                </div>

                {/* Floating Zoom Controls */}
                <div className="absolute bottom-4 right-4 flex items-center gap-1 bg-white/90 backdrop-blur-sm border border-slate-200 p-1.5 rounded-lg shadow-sm pointer-events-auto">
                    <button
                        onClick={() => {
                            if (!wrapperRef.current) return;
                            const center = {
                                x: cameraRef.current.x + wrapperRef.current.clientWidth / (2 * zoomRef.current),
                                y: cameraRef.current.y + wrapperRef.current.clientHeight / (2 * zoomRef.current)
                            };
                            const newZoom = Math.min(4.0, zoomRef.current * 1.25);
                            cameraRef.current.x = center.x - wrapperRef.current.clientWidth / (2 * newZoom);
                            cameraRef.current.y = center.y - wrapperRef.current.clientHeight / (2 * newZoom);
                            changeZoom(newZoom);
                        }}
                        className="w-7 h-7 flex items-center justify-center rounded bg-slate-50 hover:bg-slate-100 border border-slate-200 text-slate-700 font-bold text-sm cursor-pointer transition-colors"
                        title="Zoom In"
                    >
                        +
                    </button>
                    <button
                        onClick={() => {
                            if (!wrapperRef.current) return;
                            const center = {
                                x: cameraRef.current.x + wrapperRef.current.clientWidth / (2 * zoomRef.current),
                                y: cameraRef.current.y + wrapperRef.current.clientHeight / (2 * zoomRef.current)
                            };
                            const newZoom = Math.max(0.15, zoomRef.current / 1.25);
                            cameraRef.current.x = center.x - wrapperRef.current.clientWidth / (2 * newZoom);
                            cameraRef.current.y = center.y - wrapperRef.current.clientHeight / (2 * newZoom);
                            changeZoom(newZoom);
                        }}
                        className="w-7 h-7 flex items-center justify-center rounded bg-slate-50 hover:bg-slate-100 border border-slate-200 text-slate-700 font-bold text-sm cursor-pointer transition-colors"
                        title="Zoom Out"
                    >
                        −
                    </button>
                    <button
                        onClick={() => {
                            if (!wrapperRef.current) return;
                            const center = {
                                x: cameraRef.current.x + wrapperRef.current.clientWidth / (2 * zoomRef.current),
                                y: cameraRef.current.y + wrapperRef.current.clientHeight / (2 * zoomRef.current)
                            };
                            cameraRef.current.x = center.x - wrapperRef.current.clientWidth / 2;
                            cameraRef.current.y = center.y - wrapperRef.current.clientHeight / 2;
                            changeZoom(1.0);
                        }}
                        className="px-2 h-7 flex items-center justify-center rounded bg-slate-50 hover:bg-slate-100 border border-slate-200 text-slate-700 text-xs font-semibold cursor-pointer transition-colors"
                        title="Reset Zoom"
                    >
                        {Math.round(zoomState * 100)}%
                    </button>
                </div>
            </div>
        </div>
    );
}