// Everything the post knows about the view from the hill, computed from the Copernicus GLO-30
// elevation model (30 m, scripts/lib/dem.mjs) and the named summits in OpenStreetMap
// (scripts/data/peaks-osm.json). The view itself is computed by scripts/lib/viewshed.mjs,
// the same code that draws the panorama of every summit of the log (scripts/panoramas.mjs);
// what is only about the hill — its shape, and the sun on its two faces — lives here.
//
// It writes:
//
//   src/data/panorama.json           the skyline, the skyline of each depth band, the ridge
//                                    lines that stand in front of one another, every named
//                                    summit with whether you can see it, and the hill itself
//   public/assets/cerro/terrain.png  the terrain around the hill on a polar grid, for the 3D
//                                    view (elevation packed in the red and green channels)
//   public/assets/cerro/viewshed.png everything that can be seen from the summit, as a map
//                                    overlay in Web Mercator
//   public/assets/cerro/hill-sun.png the direct sun on every patch of the hill over a year
//
// Run with: npm run terrain   (the first run downloads ~550 MB of elevation tiles)

import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { loadDem } from "./lib/dem.mjs";
import { readAscents, readSummits, readTracks, linkRegistry } from "./lib/ascents.mjs";
import {
  BANDS, DEG, EARTH, EYE, METHOD, RAD, bearingDistance, castRays, meshInfo, peaksInView, png, polyline,
  ridgeLines, round, routesInView, simplify, snapToTop as snap, terrainPng, viewBox, viewStats, viewshedPng,
} from "./lib/viewshed.mjs";

// ------------------------------------------------------------------------------ settings

/** The top of the hill, as found in the elevation model (re-snapped below). */
const SUMMIT_GUESS = { lat: -32.85886, lon: -70.6594 };
/** Centre of the planted area, for the slope and aspect of the south face. */
const PLANTING = { lat: -32.85806, lon: -70.65717 };

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

// ------------------------------------------------------------------------------ the model

console.time("dem");
const dem = await loadDem(viewBox(SUMMIT_GUESS));
console.timeEnd("dem");

const snapToTop = (point, radius) => snap(dem, point, radius);
const summit = snapToTop(SUMMIT_GUESS, 80);
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

// ------------------------------------------------------------------------------ the view

const view = castRays(dem, summit);
const ridges = ridgeLines(view);

const osm = JSON.parse(readFileSync(new URL("./data/peaks-osm.json", import.meta.url), "utf8")).peaks;
const named = new Set(NAMED_IN_POST);
const peaks = peaksInView(view, osm, {
  // Names repeat (there are seven Chivatos within 160 km): the post means the nearest one.
  isNamed: (peak, nearestOfName) => named.has(peak.name) && nearestOfName.get(peak.name) === peak,
  flankReach: FLANK_REACH,
});
for (const name of NAMED_IN_POST) {
  const p = peaks.find((peak) => peak.name === name && peak.named);
  if (!p) console.log(`  ${name}: not found`);
  else console.log(`  ${p.name.padEnd(28)} ${String(p.ele ?? p.model).padStart(5)} m ${p.distance.toFixed(1).padStart(6)} km az ${p.azimuth.toFixed(1).padStart(6)}  angle ${p.angle.toFixed(2)}  front ${p.horizon.toFixed(2)}  ${p.status}${p.showing ? ` (${p.showing} m showing)` : ""}${p.hiddenBy ? ` behind ${p.hiddenBy}` : ""}`);
}

// The summits of my log, and my routes as seen from the hill.
linkRegistry(peaks, readSummits(), readAscents(), bearingDistance);
const routes = routesInView(view, readTracks(), peaks);
for (const peak of peaks.filter((p) => p.flank)) {
  console.log(`  ${peak.name}: highest visible flank ${peak.flank.elevation} m, ${peak.flank.away} m from the top`);
}

// ------------------------------------------------------------------------------ outputs

mkdirSync(OUT_ASSETS, { recursive: true });
writeFileSync(new URL("terrain.png", OUT_ASSETS), terrainPng(view));
const shed = viewshedPng(view);
writeFileSync(new URL("viewshed.png", OUT_ASSETS), shed.file);
const viewshed = shed.bounds;
const { stats } = view;

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
    method: METHOD,
    hill,
    viewpoint: { ...hill.summit, eye: round(view.eyeHeight, 1) },
    stats: viewStats(view),
    bands: BANDS,
    skyline: polyline(view.skyline),
    bandSkylines: view.bandSkylines.map(polyline),
    ridges,
    peaks,
    routes,
    mesh: meshInfo("/assets/cerro/terrain.png"),
    viewshed: { ...viewshed, file: "/assets/cerro/viewshed.png" },
    sunMap,
    sunFaces,
  }),
);
console.log(`panorama.json: ${ridges.length} ridge lines, ${ridges.reduce((n, r) => n + r.points.length, 0)} points; visible ${visibleKm2.toFixed(0)} km²`);
