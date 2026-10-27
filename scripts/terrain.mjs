// Everything the post knows about the view from the hill, computed from the Copernicus GLO-30
// elevation model (30 m, scripts/lib/dem.mjs) and the named summits in OpenStreetMap
// (scripts/data/peaks-osm.json).
//
// It casts 7,200 sight lines from the hilltop — one every 0.05° — and walks each one out to
// 160 km in 30 m steps, which is the resolution of the model itself. Every sample is turned
// into an elevation angle as seen from eye height on the summit, including earth curvature
// and standard atmospheric refraction. From that it writes:
//
//   src/data/panorama.json           the skyline, the skyline of each depth band, the ridge
//                                    lines that stand in front of one another, every named
//                                    summit with whether you can see it, and the hill itself
//   public/assets/cerro/terrain.png  the terrain around the hill on a polar grid, for the 3D
//                                    view (elevation packed in the red and green channels)
//   public/assets/cerro/viewshed.png everything that can be seen from the summit, as a map
//                                    overlay in Web Mercator
//
// Run with: npm run terrain   (the first run downloads ~550 MB of elevation tiles)

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { deflateSync } from "node:zlib";
import { loadDem } from "./lib/dem.mjs";

// ------------------------------------------------------------------------------ settings

/** The top of the hill, as found in the elevation model (re-snapped below). */
const SUMMIT_GUESS = { lat: -32.85886, lon: -70.6594 };
/** Centre of the planted area, for the slope and aspect of the south face. */
const PLANTING = { lat: -32.85806, lon: -70.65717 };
const EYE = 1.7;

const EARTH = 6371008.8;
/** Standard refraction coefficient: light bends with the earth by about 1/7 of its curvature. */
const REFRACTION = 0.13;
const EFFECTIVE = EARTH / (1 - REFRACTION);

const RADIUS = 160_000;
const RAY_STEP = 0.05;
const RAYS = Math.round(360 / RAY_STEP);
const STEP = 30;
const START = 60;
const STEPS = Math.floor((RADIUS - START) / STEP) + 1;

/** Depth bands for the drawing: the skyline of everything within each distance, in km. */
const BANDS = [1.5, 4, 8, 15, 25, 40, 70, 160];

/** Polar grid for the 3D view. */
const MESH_AZ_STEP = 0.15;
const MESH_COLS = Math.round(360 / MESH_AZ_STEP);
const MESH_R0 = 12;
const MESH_RINGS = 440;
const MESH_RATIO = (RADIUS / MESH_R0) ** (1 / (MESH_RINGS - 1));
/** Beyond this distance a mesh cell keeps the highest ground inside it. */
const MESH_POOL_FROM = 8000;

/** Within this angle of the skyline a summit is too close to call: DEM error, refraction. */
const BORDERLINE = 0.06;

/** Peaks the post talks about by name, which are always kept in the data. */
const NAMED_IN_POST = [
  "Mocoen", "Cerro Colunquén", "Cerro La Giganta", "Alto del Cobre", "Cerro Parva del Inca", "Cerro Gloria",
  "Cerro Glorieta", "Cerro Bastión", "Cerro Becker", "Cerro Espinazo del Diablo", "Cerro Algarrobo",
  "Cerro Aconcagua", "Cerro La Monja", "Cerro El Plomo", "Nevado Juncal", "Volcán Tupungato",
  "Cerro Mercedario", "Cerro La Campana", "Cerro El Roble", "Alto de los Leones", "Cerro Tolosa",
  "Cerro La Ramada", "Cerro Chivato", "Cerro Cobre del Chacabuco", "Cerro Mercachas",
];

/** How far from the top to look for a visible flank. A spine is a long mountain. */
const FLANK_REACH = { "Cerro Espinazo del Diablo": 6000 };

const OUT_DATA = new URL("../src/data/panorama.json", import.meta.url);
const OUT_ASSETS = new URL("../public/assets/cerro/", import.meta.url);

// ------------------------------------------------------------------------------ geometry

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;

function bearingDistance(from, to) {
  const φ1 = from.lat * RAD;
  const φ2 = to.lat * RAD;
  const Δλ = (to.lon - from.lon) * RAD;
  const a = Math.sin((φ2 - φ1) / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const distance = 2 * EARTH * Math.asin(Math.sqrt(a));
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  return { azimuth: (Math.atan2(y, x) * DEG + 360) % 360, distance };
}

function destination(from, azimuth, distance) {
  const δ = distance / EARTH;
  const θ = azimuth * RAD;
  const φ1 = from.lat * RAD;
  const φ2 = Math.asin(Math.sin(φ1) * Math.cos(δ) + Math.cos(φ1) * Math.sin(δ) * Math.cos(θ));
  const λ2 = from.lon * RAD + Math.atan2(Math.sin(θ) * Math.sin(δ) * Math.cos(φ1), Math.cos(δ) - Math.sin(φ1) * Math.sin(φ2));
  return { lat: φ2 * DEG, lon: λ2 * DEG };
}

/** Apparent elevation angle, in degrees, of ground `height` m high and `distance` m away. */
const angleOf = (height, distance, eye) => Math.atan((height - eye - (distance * distance) / (2 * EFFECTIVE)) / distance) * DEG;

// ------------------------------------------------------------------------------ the model

const box = {
  south: SUMMIT_GUESS.lat - RADIUS / 111_000 - 0.05,
  north: SUMMIT_GUESS.lat + RADIUS / 111_000 + 0.05,
  west: SUMMIT_GUESS.lon - RADIUS / (111_000 * Math.cos(SUMMIT_GUESS.lat * RAD)) - 0.05,
  east: SUMMIT_GUESS.lon + RADIUS / (111_000 * Math.cos(SUMMIT_GUESS.lat * RAD)) + 0.05,
};
console.time("dem");
const dem = await loadDem(box);
console.timeEnd("dem");

/** Highest model node within `radius` m of a point. */
function snapToTop(point, radius) {
  const N = 3600;
  let best = { ...point, elevation: -Infinity };
  const rows = Math.ceil(radius / 30.8);
  const cols = Math.ceil(radius / (30.9 * Math.cos(point.lat * RAD)));
  const gy = Math.round(-point.lat * N);
  const gx = Math.round((point.lon + 180) * N);
  for (let dy = -rows; dy <= rows; dy++) {
    for (let dx = -cols; dx <= cols; dx++) {
      const lat = -(gy + dy) / N;
      const lon = (gx + dx) / N - 180;
      if (bearingDistance(point, { lat, lon }).distance > radius) continue;
      const elevation = dem.node(lat, lon);
      if (elevation > best.elevation) best = { lat, lon, elevation };
    }
  }
  return best;
}

const summit = snapToTop(SUMMIT_GUESS, 80);
const EYE_HEIGHT = summit.elevation + EYE;
console.log(`summit ${summit.lat.toFixed(5)}, ${summit.lon.toFixed(5)} at ${summit.elevation} m`);

// ------------------------------------------------------------------------------ the hill

function hillFacts() {
  const N = 3600;
  const cellArea = (EARTH * RAD / N) ** 2 * Math.cos(summit.lat * RAD); // m² per node
  const gy0 = Math.round(-summit.lat * N);
  const gx0 = Math.round((summit.lon + 180) * N);
  const at = (gy, gx) => dem.node(-gy / N, gx / N - 180);

  // The fields around: the median of a ring 600–1,000 m out.
  const ring = [];
  for (let dy = -40; dy <= 40; dy++) {
    for (let dx = -40; dx <= 40; dx++) {
      const d = Math.hypot(dy * 30.9, dx * 30.9 * Math.cos(summit.lat * RAD));
      if (d >= 600 && d <= 1000) ring.push(at(gy0 + dy, gx0 + dx));
    }
  }
  ring.sort((a, b) => a - b);
  const fields = ring[Math.floor(ring.length / 2)];

  // Area of the hill above a contour: flood-fill from the summit.
  const areaAbove = (contour) => {
    const seen = new Set();
    const stack = [[0, 0]];
    let count = 0;
    while (stack.length) {
      const [dy, dx] = stack.pop();
      const key = `${dy},${dx}`;
      if (seen.has(key) || Math.abs(dy) > 60 || Math.abs(dx) > 60) continue;
      seen.add(key);
      if (at(gy0 + dy, gx0 + dx) < contour) continue;
      count++;
      stack.push([dy + 1, dx], [dy - 1, dx], [dy, dx + 1], [dy, dx - 1]);
    }
    return (count * cellArea) / 10_000;
  };

  // Slope and aspect at the planting centre: a plane fitted to the nodes within 45 m.
  let sxx = 0, syy = 0, sxy = 0, sxz = 0, syz = 0, sx = 0, sy = 0, sz = 0, n = 0;
  for (let dy = -3; dy <= 3; dy++) {
    for (let dx = -3; dx <= 3; dx++) {
      const lat = PLANTING.lat - dy / N;
      const lon = PLANTING.lon + dx / N;
      const x = dx * 30.9 * Math.cos(PLANTING.lat * RAD); // east
      const y = -dy * 30.9; // north
      if (Math.hypot(x, y) > 46) continue;
      const z = dem.elevation(lat, lon);
      sxx += x * x; syy += y * y; sxy += x * y; sxz += x * z; syz += y * z; sx += x; sy += y; sz += z; n++;
    }
  }
  // Least squares for z = a·x + b·y + c (centred, so the normal equations decouple to 2×2).
  const mx = sx / n, my = sy / n, mz = sz / n;
  const cxx = sxx / n - mx * mx, cyy = syy / n - my * my, cxy = sxy / n - mx * my;
  const cxz = sxz / n - mx * mz, cyz = syz / n - my * mz;
  const det = cxx * cyy - cxy * cxy;
  const a = (cxz * cyy - cyz * cxy) / det;
  const b = (cyz * cxx - cxz * cxy) / det;
  const slope = Math.atan(Math.hypot(a, b)) * DEG;
  // The slope faces down-gradient: the direction of (-a, -b).
  const aspect = (Math.atan2(-a, -b) * DEG + 360) % 360;

  // The shape of the hill: the nodes above the body contour, and their principal axes.
  const body = [];
  {
    const contour = fields + 8;
    const seen = new Set();
    const stack = [[0, 0]];
    while (stack.length) {
      const [dy, dx] = stack.pop();
      const key = `${dy},${dx}`;
      if (seen.has(key) || Math.abs(dy) > 60 || Math.abs(dx) > 60) continue;
      seen.add(key);
      if (at(gy0 + dy, gx0 + dx) < contour) continue;
      body.push([dx * 30.9 * Math.cos(summit.lat * RAD), -dy * 30.9]);
      stack.push([dy + 1, dx], [dy - 1, dx], [dy, dx + 1], [dy, dx - 1]);
    }
  }
  const cx = body.reduce((t, [x]) => t + x, 0) / body.length;
  const cy = body.reduce((t, [, y]) => t + y, 0) / body.length;
  let vxx = 0, vyy = 0, vxy = 0;
  for (const [x, y] of body) { vxx += (x - cx) ** 2; vyy += (y - cy) ** 2; vxy += (x - cx) * (y - cy); }
  const axis = 0.5 * Math.atan2(2 * vxy, vxx - vyy); // radians from east, counter-clockwise
  const along = body.map(([x, y]) => (x - cx) * Math.cos(axis) + (y - cy) * Math.sin(axis));
  const across = body.map(([x, y]) => -(x - cx) * Math.sin(axis) + (y - cy) * Math.cos(axis));
  const extent = (values) => Math.max(...values) - Math.min(...values) + 30;
  const trend = ((90 - axis * DEG) % 180 + 180) % 180; // compass bearing of the long axis

  // A small grid around the hill, for the contour plan in the post: 1 arc-second nodes.
  const GRID = 28;
  const grid = [];
  for (let dy = -GRID; dy <= GRID; dy++) for (let dx = -GRID; dx <= GRID; dx++) grid.push(at(gy0 + dy, gx0 + dx));

  return {
    length: Math.round(extent(along) / 10) * 10,
    width: Math.round(extent(across) / 10) * 10,
    trend: Math.round(trend),
    grid: { size: GRID * 2 + 1, north: -(gy0 - GRID) / N, west: (gx0 - GRID) / N - 180, step: 1 / N, values: grid },
    summit: { lat: +summit.lat.toFixed(5), lon: +summit.lon.toFixed(5), elevation: Math.round(summit.elevation) },
    planting: { ...PLANTING, elevation: Math.round(dem.elevation(PLANTING.lat, PLANTING.lon)) },
    fields: Math.round(fields),
    prominence: Math.round(summit.elevation - fields),
    topArea: +areaAbove(summit.elevation - 5).toFixed(1),
    bodyArea: +areaAbove(fields + 8).toFixed(1),
    bodyContour: Math.round(fields + 8),
    aspect: Math.round(aspect),
    slope: Math.round(slope),
  };
}

const hill = hillFacts();
console.log("hill", hill);

// ------------------------------------------------------------------------------ the rays

const distances = new Float64Array(STEPS);
for (let s = 0; s < STEPS; s++) distances[s] = START + s * STEP;

const skyline = new Float32Array(RAYS);
const skylineDistance = new Float32Array(RAYS);
const bandSkylines = BANDS.map(() => new Float32Array(RAYS).fill(-90));
const bandLimit = BANDS.map((km) => Math.min(STEPS - 1, Math.floor((km * 1000 - START) / STEP)));
/** Per ray, the silhouette edges: where visible ground ends and the next visible ground is far behind. */
const crests = new Array(RAYS);
const visible = new Uint8Array(RAYS * STEPS);

const mesh = new Float32Array(MESH_COLS * MESH_RINGS).fill(-1);
const ringOf = new Int32Array(STEPS);
for (let s = 0; s < STEPS; s++) {
  ringOf[s] = Math.min(MESH_RINGS - 1, Math.max(0, Math.round(Math.log(distances[s] / MESH_R0) / Math.log(MESH_RATIO))));
}

const stats = { visibleArea: 0, above3000: 0, farthest: { distance: 0, azimuth: 0 } };

console.time("rays");
const alpha = new Float64Array(STEPS);
const height = new Float32Array(STEPS);
const φ1 = summit.lat * RAD;
const λ1 = summit.lon * RAD;
const sinφ1 = Math.sin(φ1);
const cosφ1 = Math.cos(φ1);
const sinδ = distances.map((d) => Math.sin(d / EARTH));
const cosδ = distances.map((d) => Math.cos(d / EARTH));

for (let k = 0; k < RAYS; k++) {
  const θ = k * RAY_STEP * RAD;
  const sinθ = Math.sin(θ);
  const cosθ = Math.cos(θ);
  const meshCol = Math.round((k * RAY_STEP) / MESH_AZ_STEP) % MESH_COLS;

  let max = -Infinity;
  let maxAt = 0;
  let runEnd = -1;
  const edges = [];
  for (let s = 0; s < STEPS; s++) {
    const sinφ2 = sinφ1 * cosδ[s] + cosφ1 * sinδ[s] * cosθ;
    const lat = Math.asin(sinφ2) * DEG;
    const lon = (λ1 + Math.atan2(sinθ * sinδ[s] * cosφ1, cosδ[s] - sinφ1 * sinφ2)) * DEG;
    const h = dem.elevation(lat, lon);
    height[s] = h;
    const d = distances[s];
    const a = angleOf(h, d, EYE_HEIGHT);
    alpha[s] = a;

    // Far away, each mesh cell keeps the highest ground inside it: that is what makes the
    // skyline. Near the hill the mesh is finer than the model, so it samples directly (below).
    if (d > MESH_POOL_FROM) {
      const cell = meshCol * MESH_RINGS + ringOf[s];
      if (h > mesh[cell]) mesh[cell] = h;
    }

    if (a > max) {
      // Visible. If the previous visible ground ended well in front of this, that end was an edge.
      if (runEnd >= 0 && s - runEnd > 1 && d - distances[runEnd] > Math.max(250, 0.08 * distances[runEnd])) {
        edges.push(runEnd);
      }
      max = a;
      maxAt = s;
      runEnd = s;
      visible[k * STEPS + s] = 1;
      const area = RAY_STEP * RAD * d * STEP;
      stats.visibleArea += area;
      if (h >= 3000) stats.above3000 += area;
      if (d > stats.farthest.distance && h > 0) stats.farthest = { distance: d, azimuth: k * RAY_STEP, lat, lon, elevation: Math.round(h) };
    }
    for (let b = 0; b < BANDS.length; b++) if (s <= bandLimit[b] && a > bandSkylines[b][k]) bandSkylines[b][k] = a;
  }
  skyline[k] = max;
  skylineDistance[k] = distances[maxAt];
  crests[k] = edges.map((s) => ({ s, a: alpha[s], d: distances[s] }));
}
console.timeEnd("rays");

// Near cells: the model at the vertex, softened over ±25 m. This is a surface model, so the
// valley floor carries every row of poplars and every roof; seen at a grazing angle from
// 1.7 m up, unsoftened, they read as ripples across the fields.
for (let c = 0; c < MESH_COLS; c++) {
  for (let r = 0; r < MESH_RINGS; r++) {
    const cell = c * MESH_RINGS + r;
    const distance = MESH_R0 * MESH_RATIO ** r;
    const azimuth = c * MESH_AZ_STEP;
    const point = destination(summit, azimuth, distance);
    let h = distance <= MESH_POOL_FROM * 1.5 ? dem.smoothElevation(point.lat, point.lon) : dem.elevation(point.lat, point.lon);
    if (distance > 150 && distance <= MESH_POOL_FROM * 1.5) {
      let sum = h * 2;
      for (const [dn, de] of [[25, 0], [-25, 0], [0, 25], [0, -25]]) {
        sum += dem.smoothElevation(point.lat + dn / 111_195, point.lon + de / (111_195 * Math.cos(point.lat * RAD)));
      }
      const soft = sum / 6;
      // Blend into the pooled far field across the hand-over.
      const t = Math.min(1, Math.max(0, (distance - MESH_POOL_FROM) / (MESH_POOL_FROM * 0.5)));
      h = soft * (1 - t) + h * t;
    }
    mesh[cell] = distance <= MESH_POOL_FROM ? h : Math.max(h, mesh[cell]);
  }
}

// ------------------------------------------------------------------------------ ridge lines

/**
 * Links the silhouette edges of neighbouring rays into lines: an edge on one ray continues
 * as the edge on the next ray at about the same distance and angle. What comes out is the
 * drawing a panorama artist would make — one line per ridge that stands in front of another.
 */
function linkRidges() {
  const done = [];
  let open = [];
  for (let k = 0; k < RAYS; k++) {
    const next = [];
    const used = new Set();
    // Cheapest continuations first.
    const pairs = [];
    open.forEach((line, i) => {
      const last = line.points[line.points.length - 1];
      crests[k].forEach((edge, j) => {
        const da = Math.abs(edge.a - last.a);
        const dd = Math.abs(Math.log(edge.d / last.d));
        if (da < 0.12 && dd < 0.1) pairs.push({ i, j, cost: da / 0.12 + dd / 0.1 });
      });
    });
    pairs.sort((x, y) => x.cost - y.cost);
    const continued = new Set();
    for (const { i, j } of pairs) {
      if (continued.has(i) || used.has(j)) continue;
      continued.add(i);
      used.add(j);
      const edge = crests[k][j];
      open[i].points.push({ k, a: edge.a, d: edge.d });
      next.push(open[i]);
    }
    open.forEach((line, i) => continued.has(i) || done.push(line));
    crests[k].forEach((edge, j) => used.has(j) || next.push({ points: [{ k, a: edge.a, d: edge.d }] }));
    open = next;
  }
  done.push(...open);
  return done;
}

/** Ramer–Douglas–Peucker on [x, y] pairs. */
function simplify(points, tolerance) {
  if (points.length < 3) return points;
  const keep = new Uint8Array(points.length);
  keep[0] = keep[points.length - 1] = 1;
  const stack = [[0, points.length - 1]];
  while (stack.length) {
    const [first, last] = stack.pop();
    const [x1, y1] = points[first];
    const [x2, y2] = points[last];
    const length = Math.hypot(x2 - x1, y2 - y1) || 1e-9;
    let worst = -1, index = -1;
    for (let i = first + 1; i < last; i++) {
      const [x, y] = points[i];
      const distance = Math.abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1) / length;
      if (distance > worst) { worst = distance; index = i; }
    }
    if (worst > tolerance) {
      keep[index] = 1;
      stack.push([first, index], [index, last]);
    }
  }
  return points.filter((_, i) => keep[i]);
}

const round = (value, digits) => Number(value.toFixed(digits));

console.time("ridges");
const ridges = linkRidges()
  // A ridge has to run far enough to be worth a line. Near the hill that bar is higher: the
  // model is a surface model, so rows of trees and roofs on the valley floor make little
  // "ridges" of their own.
  .filter((line) => {
    const middle = line.points[Math.floor(line.points.length / 2)];
    const span = line.points.length * RAY_STEP;
    if (middle.d < 600) return false;
    // Low on the valley floor, only long features count.
    if (middle.a < 0.6 && middle.d < 30_000 && span < 2) return false;
    return span >= (middle.d < 10_000 ? 0.8 : 0.35);
  })
  .map((line) => {
    const ds = line.points.map((p) => p.d).sort((a, b) => a - b);
    return {
      distance: Math.round(ds[Math.floor(ds.length / 2)]),
      points: simplify(line.points.map((p) => [p.k * RAY_STEP, p.a]), 0.006).map(([x, y]) => [round(x, 3), round(y, 3)]),
    };
  });
console.timeEnd("ridges");

const polyline = (values) =>
  simplify(Array.from(values, (a, k) => [k * RAY_STEP, a]).concat([[360, values[0]]]), 0.004).map(([x, y]) => [round(x, 3), round(y, 3)]);

// ------------------------------------------------------------------------------ the summits

const osm = JSON.parse(readFileSync(new URL("./data/peaks-osm.json", import.meta.url), "utf8")).peaks;

/** Max elevation angle along a bearing between two distances, and where it happens. */
function profileMax(azimuth, from, to) {
  let best = { angle: -90, distance: 0, lat: 0, lon: 0 };
  for (let d = from; d <= to; d += STEP) {
    const p = destination(summit, azimuth, d);
    const a = angleOf(dem.elevation(p.lat, p.lon), d, EYE_HEIGHT);
    if (a > best.angle) best = { angle: a, distance: d, lat: p.lat, lon: p.lon };
  }
  return best;
}

const parseEle = (value) => {
  const n = parseFloat(String(value ?? "").replace(",", "."));
  return Number.isFinite(n) && n > 0 && n < 7000 ? Math.round(n) : null;
};

console.time("peaks");
let peaks = osm
  .map((peak) => {
    const { distance } = bearingDistance(summit, peak);
    if (distance < 800 || distance > RADIUS - 2000) return null;
    const top = snapToTop(peak, 150);
    const { azimuth, distance: d } = bearingDistance(summit, top);
    const angle = angleOf(top.elevation, d, EYE_HEIGHT);
    const guard = Math.max(150, 0.004 * d);
    const front = profileMax(azimuth, START, d - guard);
    const behind = profileMax(azimuth, d + guard, Math.min(RADIUS, d + 60_000));
    const margin = angle - front.angle;
    return {
      osm: peak.id,
      name: peak.name,
      ele: parseEle(peak.ele),
      model: Math.round(top.elevation),
      lat: round(top.lat, 5),
      lon: round(top.lon, 5),
      distance: round(d / 1000, 2),
      azimuth: round(azimuth, 2),
      angle: round(angle, 3),
      horizon: round(front.angle, 3),
      margin: round(margin, 3),
      /** Metres of the mountain that stick out above whatever is in front of it. */
      showing: margin > 0 ? Math.round(Math.tan(margin * RAD) * d) : 0,
      blocker: margin < 0 ? { lat: round(front.lat, 5), lon: round(front.lon, 5), distance: round(front.distance / 1000, 2) } : null,
      /** Seen against the sky, rather than against a bigger mountain behind it. */
      skyline: angle > behind.angle,
      status: margin > BORDERLINE ? "visible" : margin < -BORDERLINE ? "hidden" : "borderline",
      kind: peak.kind ?? "peak",
      wikidata: peak.wikidata ?? null,
    };
  })
  .filter(Boolean);

// A summit within 1.2 km of a higher named one is its shoulder or fore-summit.
for (const peak of peaks) {
  const parent = peaks
    .filter((other) => other !== peak && other.model > peak.model && bearingDistance(peak, other).distance < 1200)
    .sort((a, b) => b.model - a.model)[0];
  peak.parent = parent ? parent.name : null;
}
// Name what hides a hidden summit, when the thing in the way is itself a named summit's flank.
for (const peak of peaks) {
  if (!peak.blocker) continue;
  const near = peaks
    .map((other) => ({ other, distance: bearingDistance(peak.blocker, other).distance }))
    .filter(({ other, distance }) => other !== peak && distance < 2500 && other.distance < peak.distance)
    .sort((a, b) => b.other.model - a.other.model)[0];
  peak.hiddenBy = near ? near.other.name : null;
}

// Names repeat (there are seven Chivatos within 160 km): the post means the nearest one.
const nearestOfName = new Map();
for (const peak of peaks) {
  const current = nearestOfName.get(peak.name);
  if (!current || peak.distance < current.distance) nearestOfName.set(peak.name, peak);
}
const namedInPost = new Set(NAMED_IN_POST.map((name) => nearestOfName.get(name)).filter(Boolean));
for (const peak of peaks) peak.named = namedInPost.has(peak);

const keep = (peak) =>
  peak.named ||
  (peak.status !== "hidden" && !peak.parent) ||
  (peak.status === "hidden" && peak.model >= 5000 && !peak.parent);
peaks = peaks.filter(keep).sort((a, b) => a.azimuth - b.azimuth);
console.timeEnd("peaks");
console.log(
  `${peaks.length} summits kept: ${peaks.filter((p) => p.status === "visible").length} visible, ` +
    `${peaks.filter((p) => p.status === "borderline").length} borderline, ${peaks.filter((p) => p.status === "hidden").length} hidden`,
);
for (const name of NAMED_IN_POST) {
  const p = peaks.find((peak) => peak.name === name && peak.named);
  if (!p) console.log(`  ${name}: not found`);
  else console.log(`  ${p.name.padEnd(28)} ${String(p.ele ?? p.model).padStart(5)} m ${p.distance.toFixed(1).padStart(6)} km az ${p.azimuth.toFixed(1).padStart(6)}  angle ${p.angle.toFixed(2)}  front ${p.horizon.toFixed(2)}  ${p.status}${p.showing ? ` (${p.showing} m showing)` : ""}${p.hiddenBy ? ` behind ${p.hiddenBy}` : ""}`);
}

// ------------------------------------------------------------------------------ flanks

/** How far above (positive) or below the ground in front a point is, in degrees. */
function sight(point, height) {
  const { azimuth, distance } = bearingDistance(summit, point);
  const angle = angleOf(height, distance, EYE_HEIGHT);
  const front = profileMax(azimuth, START, distance - Math.max(60, 0.002 * distance));
  return { azimuth, distance, angle, margin: angle - front.angle };
}

// A hidden summit is not always a hidden mountain: its flank or its ridge can still show.
// For each summit the post names, find the highest visible ground within 2.5 km of the top.
console.time("flanks");
for (const peak of peaks.filter((p) => p.named && p.status !== "visible")) {
  const N = 3600;
  const reach = FLANK_REACH[peak.name] ?? 2500;
  const gy = Math.round(-peak.lat * N);
  const gx = Math.round((peak.lon + 180) * N);
  const rows = Math.ceil(reach / 30.8);
  const cols = Math.ceil(reach / (30.9 * Math.cos(peak.lat * RAD)));
  let best = null;
  for (let dy = -rows; dy <= rows; dy += 3) {
    for (let dx = -cols; dx <= cols; dx += 3) {
      const lat = -(gy + dy) / N;
      const lon = (gx + dx) / N - 180;
      const away = bearingDistance(peak, { lat, lon }).distance;
      if (away > reach) continue;
      const h = dem.node(lat, lon);
      // Higher ground than the summit itself belongs to some other mountain.
      if (h < peak.model - 900 || h > peak.model + 30 || (best && h <= best.elevation)) continue;
      const seen = sight({ lat, lon }, h);
      if (seen.margin > BORDERLINE) best = { lat: round(lat, 5), lon: round(lon, 5), elevation: Math.round(h), away: Math.round(away), azimuth: round(seen.azimuth, 2), angle: round(seen.angle, 3) };
    }
  }
  peak.flank = best;
}
console.timeEnd("flanks");

// ------------------------------------------------------------------------------ my routes

/** Summits with an ascent post but no GPS track. */
const ASCENT_SUMMITS = { mocoen: "Mocoen", mercachas: "Cerro Mercachas" };

const frontmatter = (text) => {
  const head = text.match(/^---\n([\s\S]*?)\n---/);
  const out = {};
  for (const line of head ? head[1].split("\n") : []) {
    const m = line.match(/^(\w+):\s*"?(.*?)"?\s*$/);
    if (m) out[m[1]] = m[2];
  }
  return out;
};

console.time("routes");
const ascentsDir = new URL("../src/content/ascents/", import.meta.url);
const routes = [];
for (const file of readdirSync(ascentsDir).filter((f) => f.endsWith(".md")).sort()) {
  const slug = file.replace(/\.md$/, "");
  const meta = frontmatter(readFileSync(new URL(file, ascentsDir), "utf8"));
  if (ASCENT_SUMMITS[slug]) {
    const peak = peaks.find((p) => p.name === ASCENT_SUMMITS[slug] && p.named);
    if (peak) peak.ascent = { slug, title: meta.title };
  }
  if (!meta.route) continue;
  const gpx = readFileSync(new URL(`../public${meta.route}`, import.meta.url), "utf8");
  const track = [...gpx.matchAll(/<trkpt\b[^>]*>/g)].map(([tag]) => ({
    lat: Number(tag.match(/lat="([-\d.]+)"/)[1]),
    lon: Number(tag.match(/lon="([-\d.]+)"/)[1]),
  }));
  // One point every ~40 m of trail is plenty at these distances.
  const points = [];
  for (const point of track) {
    if (!points.length || bearingDistance(points[points.length - 1], point).distance >= 40) points.push(point);
  }
  const near = Math.min(...points.map((p) => bearingDistance(summit, p).distance));
  if (near > RADIUS - 2000) continue;

  let length = 0, seenLength = 0;
  const projected = points.map((point, i) => {
    const h = dem.elevation(point.lat, point.lon) + EYE;
    const seen = sight(point, h);
    const step = i ? bearingDistance(points[i - 1], point).distance : 0;
    length += step;
    if (seen.margin > 0) seenLength += step;
    return [seen.azimuth, seen.angle, seen.margin > 0 ? 1 : 0, seen.distance];
  });
  // Simplify each run of visible or hidden trail separately, so the switch points survive.
  const runs = [];
  for (const point of projected) {
    const run = runs[runs.length - 1];
    if (run && run.visible === point[2]) run.points.push(point);
    else runs.push({ visible: point[2], points: run ? [run.points[run.points.length - 1], point] : [point] });
  }
  const summits = peaks
    .filter((peak) => points.some((point) => bearingDistance(point, peak).distance < 150))
    .map((peak) => peak.name);
  for (const peak of peaks) if (summits.includes(peak.name)) peak.ascent ??= { slug, title: meta.title };
  routes.push({
    slug,
    title: meta.title,
    date: meta.pubDate,
    distance: [round(near / 1000, 1), round(Math.max(...projected.map((p) => p[3])) / 1000, 1)],
    length: round(length / 1000, 1),
    visibleShare: round(seenLength / length, 3),
    summits,
    runs: runs.map((run) => ({
      visible: !!run.visible,
      points: simplify(run.points.map(([az, a]) => [az, a]), 0.004).map(([x, y]) => [round(x, 3), round(y, 3)]),
    })),
  });
}
console.timeEnd("routes");
for (const route of routes) {
  console.log(`  route ${route.slug.padEnd(22)} ${route.distance.join("–")} km, ${(route.visibleShare * 100).toFixed(0)}% visible, summits: ${route.summits.join(", ") || "—"}`);
}
for (const peak of peaks.filter((p) => p.flank)) {
  console.log(`  ${peak.name}: highest visible flank ${peak.flank.elevation} m, ${peak.flank.away} m from the top`);
}

// ------------------------------------------------------------------------------ PNG writer

const CRC = new Uint32Array(256).map((_, n) => {
  let c = n;
  for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
  return c >>> 0;
});
const crc32 = (bytes) => {
  let c = 0xffffffff;
  for (const byte of bytes) c = CRC[(c ^ byte) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
};

/** A minimal PNG encoder: 8-bit RGB or RGBA, each row filtered with whichever of Sub/Up packs best. */
function png(width, height, channels, pixels) {
  const stride = width * channels;
  const filtered = Buffer.alloc((stride + 1) * height);
  for (let y = 0; y < height; y++) {
    const row = pixels.subarray(y * stride, (y + 1) * stride);
    const above = y ? pixels.subarray((y - 1) * stride, y * stride) : null;
    const sub = Buffer.alloc(stride);
    const up = Buffer.alloc(stride);
    let costSub = 0, costUp = 0;
    for (let i = 0; i < stride; i++) {
      sub[i] = (row[i] - (i >= channels ? row[i - channels] : 0)) & 0xff;
      up[i] = (row[i] - (above ? above[i] : 0)) & 0xff;
      costSub += sub[i] < 128 ? sub[i] : 256 - sub[i];
      costUp += up[i] < 128 ? up[i] : 256 - up[i];
    }
    filtered[y * (stride + 1)] = costSub <= costUp ? 1 : 2;
    (costSub <= costUp ? sub : up).copy(filtered, y * (stride + 1) + 1);
  }
  const chunk = (type, data) => {
    const body = Buffer.concat([Buffer.from(type, "latin1"), data]);
    const out = Buffer.alloc(body.length + 8);
    out.writeUInt32BE(data.length, 0);
    body.copy(out, 4);
    out.writeUInt32BE(crc32(body), body.length + 4);
    return out;
  };
  const header = Buffer.alloc(13);
  header.writeUInt32BE(width, 0);
  header.writeUInt32BE(height, 4);
  header[8] = 8;
  header[9] = channels === 4 ? 6 : 2;
  return Buffer.concat([
    Buffer.from([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a]),
    chunk("IHDR", header),
    chunk("IDAT", deflateSync(filtered, { level: 9 })),
    chunk("IEND", Buffer.alloc(0)),
  ]);
}

// ------------------------------------------------------------------------------ outputs

mkdirSync(OUT_ASSETS, { recursive: true });

// The 3D terrain: one column per bearing, one row per ring, elevation in centimetres packed
// big-endian into R, G and B. Whole metres would do for the mountains but not for the ground
// at your feet, where the rings are only a few metres apart: there even a 10 cm step tilts
// the surface by degrees and draws terraces into the shading.
{
  const pixels = new Uint8Array(MESH_COLS * MESH_RINGS * 3);
  for (let r = 0; r < MESH_RINGS; r++) {
    for (let c = 0; c < MESH_COLS; c++) {
      const v = Math.max(0, Math.min(0xffffff, Math.round(mesh[c * MESH_RINGS + r] * 100)));
      const i = (r * MESH_COLS + c) * 3;
      pixels[i] = v >> 16;
      pixels[i + 1] = (v >> 8) & 0xff;
      pixels[i + 2] = v & 0xff;
    }
  }
  const file = png(MESH_COLS, MESH_RINGS, 3, pixels);
  writeFileSync(new URL("terrain.png", OUT_ASSETS), file);
  console.log(`terrain.png ${MESH_COLS}×${MESH_RINGS}, ${(file.length / 1024).toFixed(0)} KB`);
}

// The viewshed, rasterised in Web Mercator so a Leaflet image overlay lands exactly.
const viewshed = (() => {
  const reach = RADIUS / 111_320;
  const south = summit.lat - reach;
  const north = summit.lat + reach;
  const west = summit.lon - reach / Math.cos(summit.lat * RAD);
  const east = summit.lon + reach / Math.cos(summit.lat * RAD);
  const mercator = (lat) => Math.log(Math.tan(Math.PI / 4 + (lat * RAD) / 2));
  const yNorth = mercator(north);
  const ySouth = mercator(south);
  const width = 1600;
  const height = Math.round((width * (yNorth - ySouth)) / ((east - west) * RAD));
  const pixels = new Uint8Array(width * height * 4);
  for (let y = 0; y < height; y++) {
    const my = yNorth - ((y + 0.5) / height) * (yNorth - ySouth);
    const lat = (2 * Math.atan(Math.exp(my)) - Math.PI / 2) * DEG;
    for (let x = 0; x < width; x++) {
      const lon = west + ((x + 0.5) / width) * (east - west);
      const { azimuth, distance } = bearingDistance(summit, { lat, lon });
      if (distance > RADIUS || distance < START) continue;
      const k = Math.round(azimuth / RAY_STEP) % RAYS;
      const s = Math.round((distance - START) / STEP);
      if (!visible[k * STEPS + s] && !visible[k * STEPS + Math.max(0, s - 1)] && !visible[k * STEPS + Math.min(STEPS - 1, s + 1)]) continue;
      const i = (y * width + x) * 4;
      pixels[i] = 242;
      pixels[i + 1] = 166;
      pixels[i + 2] = 28;
      pixels[i + 3] = 205;
    }
  }
  const file = png(width, height, 4, pixels);
  writeFileSync(new URL("viewshed.png", OUT_ASSETS), file);
  console.log(`viewshed.png ${width}×${height}, ${(file.length / 1024).toFixed(0)} KB`);
  return { south: round(south, 5), north: round(north, 5), west: round(west, 5), east: round(east, 5) };
})();

// ------------------------------------------------------------------------------ the sun on the hill

/** Sun position (NOAA): azimuth clockwise from north and elevation, degrees. */
function sunPosition(date, lat, lon) {
  const jd = date.getTime() / 86_400_000 + 2_440_587.5;
  const T = (jd - 2_451_545) / 36_525;
  const L0 = (280.46646 + T * (36000.76983 + T * 0.0003032)) % 360;
  const M = 357.52911 + T * (35999.05029 - 0.0001537 * T);
  const e = 0.016708634 - T * (0.000042037 + 0.0000001267 * T);
  const C = Math.sin(M * RAD) * (1.914602 - T * (0.004817 + 0.000014 * T)) + Math.sin(2 * M * RAD) * (0.019993 - 0.000101 * T) + Math.sin(3 * M * RAD) * 0.000289;
  const omega = 125.04 - 1934.136 * T;
  const lambda = L0 + C - 0.00569 - 0.00478 * Math.sin(omega * RAD);
  const eps = 23 + (26 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60) / 60 + 0.00256 * Math.cos(omega * RAD);
  const decl = Math.asin(Math.sin(eps * RAD) * Math.sin(lambda * RAD));
  const y = Math.tan((eps * RAD) / 2) ** 2;
  const eqTime = (4 / RAD) * (y * Math.sin(2 * L0 * RAD) - 2 * e * Math.sin(M * RAD) + 4 * e * y * Math.sin(M * RAD) * Math.cos(2 * L0 * RAD) - 0.5 * y * y * Math.sin(4 * L0 * RAD) - 1.25 * e * e * Math.sin(2 * M * RAD));
  const minutes = date.getUTCHours() * 60 + date.getUTCMinutes();
  const solarTime = (((minutes + eqTime + 4 * lon) % 1440) + 1440) % 1440;
  const ha = (solarTime / 4 - 180) * RAD;
  const φ = lat * RAD;
  const cosZ = Math.min(1, Math.max(-1, Math.sin(φ) * Math.sin(decl) + Math.cos(φ) * Math.cos(decl) * Math.cos(ha)));
  const elevation = 90 - Math.acos(cosZ) * DEG;
  const azimuth = (Math.atan2(Math.sin(ha), Math.cos(ha) * Math.sin(φ) - Math.tan(decl) * Math.cos(φ)) * DEG + 540) % 360;
  return { azimuth, elevation };
}

/**
 * Clear-sky direct sun through a year, for every day of each month at 20-minute steps:
 * returns, per month, the sun's direction and the beam strength (W/m², Meinel's clear-sky
 * attenuation with air mass). Diffuse light is left out — it is what every slope gets alike.
 */
const SUN_YEAR = (() => {
  const months = [];
  for (let m = 0; m < 12; m++) {
    const samples = [];
    for (const day of [5, 15, 25]) {
      for (let minute = 0; minute < 1440; minute += 20) {
        const date = new Date(Date.UTC(2025, m, day, 0, minute));
        const { azimuth, elevation } = sunPosition(date, summit.lat, summit.lon);
        if (elevation <= 0.5) continue;
        const airMass = 1 / (Math.sin(elevation * RAD) + 0.50572 * (elevation + 6.07995) ** -1.6364);
        const beam = 1361 * 0.7 ** (airMass ** 0.678);
        const ce = Math.cos(elevation * RAD);
        samples.push([ce * Math.sin(azimuth * RAD), ce * Math.cos(azimuth * RAD), Math.sin(elevation * RAD), beam]);
      }
    }
    months.push(samples);
  }
  return months;
})();

/** Daily clear-sky direct energy (kWh/m²) on a plane of given slope and aspect, per month. */
function insolation(slope, aspect) {
  const s = slope * RAD;
  const a = aspect * RAD;
  // Unit normal of the plane: east, north, up.
  const n = [Math.sin(s) * Math.sin(a), Math.sin(s) * Math.cos(a), Math.cos(s)];
  return SUN_YEAR.map((samples) => {
    let energy = 0;
    for (const [x, y, z, beam] of samples) energy += Math.max(0, n[0] * x + n[1] * y + n[2] * z) * beam;
    // 3 days × 20-minute steps: average day, in kWh.
    return round(((energy / 3) * (20 / 60)) / 1000, 2);
  });
}

// The plan of the hill: every patch of ground coloured by how much direct sun it gets over a
// year compared with flat ground — the whole argument for planting on the south face, as a map.
const sunMap = (() => {
  const CROP = 14; // nodes each side of the summit
  const SCALE = 6; // pixels per node
  const N = 3600;
  const size = (2 * CROP) * SCALE;
  const gy0 = Math.round(-summit.lat * N);
  const gx0 = Math.round((summit.lon + 180) * N);
  const north = -(gy0 - CROP) / N;
  const west = (gx0 - CROP) / N - 180;
  const flat = insolation(0, 0).reduce((a, b) => a + b, 0);
  const pixels = new Uint8Array(size * size * 4);
  const dx = 4; // metres, for the slope
  const mLat = 1 / 111_195;
  const mLon = 1 / (111_195 * Math.cos(summit.lat * RAD));
  let lowest = Infinity, highest = -Infinity;
  for (let py = 0; py < size; py++) {
    for (let px = 0; px < size; px++) {
      const lat = north - (py + 0.5) / SCALE / N;
      const lon = west + (px + 0.5) / SCALE / N;
      const e = (dem.smoothElevation(lat, lon + dx * mLon) - dem.smoothElevation(lat, lon - dx * mLon)) / (2 * dx);
      const n = (dem.smoothElevation(lat + dx * mLat, lon) - dem.smoothElevation(lat - dx * mLat, lon)) / (2 * dx);
      const slope = Math.atan(Math.hypot(e, n)) * DEG;
      const aspect = (Math.atan2(-e, -n) * DEG + 360) % 360;
      const ratio = insolation(slope, aspect).reduce((a, b) => a + b, 0) / flat;
      lowest = Math.min(lowest, ratio);
      highest = Math.max(highest, ratio);
      // Diverging: amber where the ground gets more sun than flat ground, green where less,
      // neutral grey on the flat. Full strength at ±15%.
      const t = Math.max(-1, Math.min(1, (ratio - 1) / 0.15));
      const neutral = [138, 143, 150];
      const target = t > 0 ? [224, 154, 52] : [44, 150, 112];
      const k = Math.abs(t);
      const i = (py * size + px) * 4;
      for (let c = 0; c < 3; c++) pixels[i + c] = Math.round(neutral[c] + (target[c] - neutral[c]) * k);
      pixels[i + 3] = 235;
    }
  }
  writeFileSync(new URL("hill-sun.png", OUT_ASSETS), png(size, size, 4, pixels));

  // Contours of the hill body, every 2 m, from the smooth model on the same pixel grid:
  // marching squares, segments chained into lines, in metres from the north-west corner.
  const width = ((2 * CROP) / N) * 111_195 * Math.cos(summit.lat * RAD);
  const height = ((2 * CROP) / N) * 111_195;
  const heights = new Float64Array((size + 1) * (size + 1));
  for (let py = 0; py <= size; py++) {
    for (let px = 0; px <= size; px++) heights[py * (size + 1) + px] = dem.smoothElevation(north - py / SCALE / N, west + px / SCALE / N);
  }
  const contours = [];
  for (let level = hill.bodyContour; level < summit.elevation; level += 2) {
    const segments = [];
    const at = (px, py) => heights[py * (size + 1) + px];
    const cross = (x1, y1, h1, x2, y2, h2) => {
      const t = (level - h1) / (h2 - h1);
      return [x1 + (x2 - x1) * t, y1 + (y2 - y1) * t];
    };
    for (let py = 0; py < size; py++) {
      for (let px = 0; px < size; px++) {
        const corners = [[px, py], [px + 1, py], [px + 1, py + 1], [px, py + 1]];
        const h = corners.map(([x, y]) => at(x, y));
        const points = [];
        for (let k = 0; k < 4; k++) {
          const a = k, b = (k + 1) % 4;
          if ((h[a] >= level) !== (h[b] >= level)) points.push(cross(...corners[a], h[a], ...corners[b], h[b]));
        }
        if (points.length === 2) segments.push(points);
        else if (points.length === 4) segments.push([points[0], points[1]], [points[2], points[3]]);
      }
    }
    // Chain segments that share an end point.
    const key = ([x, y]) => `${x.toFixed(4)},${y.toFixed(4)}`;
    const ends = new Map();
    segments.forEach((segment, i) => segment.forEach((point) => ends.set(key(point), [...(ends.get(key(point)) ?? []), i])));
    const used = new Uint8Array(segments.length);
    for (let i = 0; i < segments.length; i++) {
      if (used[i]) continue;
      used[i] = 1;
      const line = [...segments[i]];
      for (const forward of [true, false]) {
        for (;;) {
          const tip = forward ? line[line.length - 1] : line[0];
          const next = (ends.get(key(tip)) ?? []).find((j) => !used[j]);
          if (next === undefined) break;
          used[next] = 1;
          const [a, b] = segments[next];
          const other = key(a) === key(tip) ? b : a;
          if (forward) line.push(other);
          else line.unshift(other);
        }
      }
      if (line.length < 6) continue;
      const metres = line.map(([x, y]) => [(x / size) * width, (y / size) * height]);
      // A closed loop starts where it ends, which Douglas–Peucker can't split: halve it first.
      const half = Math.floor(metres.length / 2);
      const simple = [...simplify(metres.slice(0, half + 1), 0.6), ...simplify(metres.slice(half), 0.6).slice(1)];
      contours.push({ level, points: simple.map(([x, y]) => [round(x, 1), round(y, 1)]) });
    }
  }

  return {
    width: round(width, 1),
    height: round(height, 1),
    contours,
    file: "/assets/cerro/hill-sun.png",
    north: round(north, 6),
    west: round(west, 6),
    south: round(north - (2 * CROP) / N, 6),
    east: round(west + (2 * CROP) / N, 6),
    range: [round(lowest, 3), round(highest, 3)],
  };
})();
console.log("sun map", sunMap);

const sunFaces = {
  flat: insolation(0, 0),
  south: insolation(hill.slope, hill.aspect),
  north: insolation(hill.slope, (hill.aspect + 180) % 360),
};
console.log("insolation kWh/m²/day by month", sunFaces);

const visibleKm2 = stats.visibleArea / 1e6;
{
  const near = osm
    .map((peak) => ({ peak, distance: bearingDistance(stats.farthest, peak).distance }))
    .sort((a, b) => a.distance - b.distance)[0];
  if (near && near.distance < 4000) stats.farthest.near = { name: near.peak.name, distance: round(near.distance / 1000, 1) };
}
writeFileSync(
  OUT_DATA,
  JSON.stringify({
    source: "Copernicus GLO-30 DEM (© DLR e.V. 2010–2014 and © Airbus 2014–2018, provided under COPERNICUS by the European Union and ESA); summits © OpenStreetMap contributors",
    generated: new Date().toISOString().slice(0, 10),
    method: { eye: EYE, refraction: REFRACTION, radius: RADIUS, rayStep: RAY_STEP, step: STEP, borderline: BORDERLINE },
    hill,
    viewpoint: { ...hill.summit, eye: round(EYE_HEIGHT, 1) },
    stats: {
      visibleArea: Math.round(visibleKm2),
      visibleShare: round(visibleKm2 / ((Math.PI * RADIUS * RADIUS) / 1e6), 4),
      above3000: Math.round(stats.above3000 / 1e6),
      farthest: { ...stats.farthest, distance: round(stats.farthest.distance / 1000, 1), lat: round(stats.farthest.lat, 4), lon: round(stats.farthest.lon, 4) },
      highestSkyline: round(Math.max(...skyline), 2),
      lowestSkyline: round(Math.min(...skyline), 2),
    },
    bands: BANDS,
    skyline: polyline(skyline),
    bandSkylines: bandSkylines.map(polyline),
    ridges,
    peaks,
    routes,
    mesh: { encoding: "centimetres, 24-bit big-endian RGB", cols: MESH_COLS, rings: MESH_RINGS, azStep: MESH_AZ_STEP, r0: MESH_R0, ratio: MESH_RATIO, effectiveRadius: Math.round(EFFECTIVE), file: "/assets/cerro/terrain.png" },
    viewshed: { ...viewshed, file: "/assets/cerro/viewshed.png" },
    sunMap,
    sunFaces,
  }),
);
console.log(`panorama.json: ${ridges.length} ridge lines, ${ridges.reduce((n, r) => n + r.points.length, 0)} points; visible ${visibleKm2.toFixed(0)} km²`);
