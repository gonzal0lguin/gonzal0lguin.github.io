// What can be seen from a point, computed from an elevation model (scripts/lib/dem.mjs) the way
// scripts/terrain.mjs does it for the hill and scripts/panoramas.mjs for every summit of the log.
//
// It casts one sight line every 0.05° of bearing and walks each one out to 160 km in 30 m
// steps, the resolution of the model itself. Every sample is turned into an elevation angle as
// seen from eye height, including earth curvature and standard atmospheric refraction. From that
// come the skyline, the skyline of each depth band, the silhouette edges where one ridge passes
// in front of another, the viewshed and a polar mesh of the terrain for the 3D view; and, given
// named places, whether each of them can be seen.

import { deflateSync } from "node:zlib";

// ------------------------------------------------------------------------------ settings

export const EYE = 1.7;
export const EARTH = 6371008.8;
/** Standard refraction coefficient: light bends with the earth by about 1/7 of its curvature. */
export const REFRACTION = 0.13;
export const EFFECTIVE = EARTH / (1 - REFRACTION);

export const RADIUS = 160_000;
export const RAY_STEP = 0.05;
export const RAYS = Math.round(360 / RAY_STEP);
export const STEP = 30;
export const START = 60;
export const STEPS = Math.floor((RADIUS - START) / STEP) + 1;

/** Depth bands for the drawing: the skyline of everything within each distance, in km. */
export const BANDS = [1.5, 4, 8, 15, 25, 40, 70, 160];

/** Polar grid for the 3D view. */
const MESH_AZ_STEP = 0.15;
const MESH_COLS = Math.round(360 / MESH_AZ_STEP);
const MESH_R0 = 12;
const MESH_RINGS = 440;
const MESH_RATIO = (RADIUS / MESH_R0) ** (1 / (MESH_RINGS - 1));
/** Beyond this distance a mesh cell keeps the highest ground inside it. */
const MESH_POOL_FROM = 8000;

/** Within this angle of the skyline a summit is too close to call: DEM error, refraction. */
export const BORDERLINE = 0.06;

export const METHOD = { eye: EYE, refraction: REFRACTION, radius: RADIUS, rayStep: RAY_STEP, step: STEP, borderline: BORDERLINE };

// ------------------------------------------------------------------------------ geometry

export const RAD = Math.PI / 180;
export const DEG = 180 / Math.PI;

export function bearingDistance(from, to) {
  const φ1 = from.lat * RAD;
  const φ2 = to.lat * RAD;
  const Δλ = (to.lon - from.lon) * RAD;
  const a = Math.sin((φ2 - φ1) / 2) ** 2 + Math.cos(φ1) * Math.cos(φ2) * Math.sin(Δλ / 2) ** 2;
  const distance = 2 * EARTH * Math.asin(Math.sqrt(a));
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  return { azimuth: (Math.atan2(y, x) * DEG + 360) % 360, distance };
}

export function destination(from, azimuth, distance) {
  const δ = distance / EARTH;
  const θ = azimuth * RAD;
  const φ1 = from.lat * RAD;
  const φ2 = Math.asin(Math.sin(φ1) * Math.cos(δ) + Math.cos(φ1) * Math.sin(δ) * Math.cos(θ));
  const λ2 = from.lon * RAD + Math.atan2(Math.sin(θ) * Math.sin(δ) * Math.cos(φ1), Math.cos(δ) - Math.sin(φ1) * Math.sin(φ2));
  return { lat: φ2 * DEG, lon: λ2 * DEG };
}

/** Apparent elevation angle, in degrees, of ground `height` m high and `distance` m away. */
export const angleOf = (height, distance, eye) => Math.atan((height - eye - (distance * distance) / (2 * EFFECTIVE)) / distance) * DEG;

export const round = (value, digits) => Number(value.toFixed(digits));

/** The box of model tiles a view from `point` needs. */
export const viewBox = (point) => ({
  south: point.lat - RADIUS / 111_000 - 0.05,
  north: point.lat + RADIUS / 111_000 + 0.05,
  west: point.lon - RADIUS / (111_000 * Math.cos(point.lat * RAD)) - 0.05,
  east: point.lon + RADIUS / (111_000 * Math.cos(point.lat * RAD)) + 0.05,
});

/** Highest model node within `radius` m of a point. */
export function snapToTop(dem, point, radius) {
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

/** Ramer–Douglas–Peucker on [x, y] pairs. */
export function simplify(points, tolerance) {
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

// ------------------------------------------------------------------------------ the rays

/**
 * Casts every sight line from `summit` (the viewpoint, already on the model's highest node).
 * `onVisible(lat, lon, h, d, area, k)` is called for every sample in view, for anything the
 * caller wants to add up along the way (glaciers, say).
 *
 * `occludeFrom` (m): ground nearer than this hides nothing. On a broad summit — a volcano's
 * dome, a long flat top — the rim of the top itself would otherwise hide every steep view
 * down from its exact centre, which is not what anyone standing up there sees: they walk to
 * the edge. Below it the ground still counts as in view.
 */
export function castRays(dem, summit, { onVisible, occludeFrom = START } = {}) {
  const EYE_HEIGHT = summit.elevation + EYE;
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

  const stats = { visibleArea: 0, above3000: 0, farthest: { distance: 0, azimuth: 0 }, sea: null };

  console.time("rays");
  const alpha = new Float64Array(STEPS);
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
      const d = distances[s];
      const a = angleOf(h, d, EYE_HEIGHT);
      alpha[s] = a;

      // Far away, each mesh cell keeps the highest ground inside it: that is what makes the
      // skyline. Near the viewpoint the mesh is finer than the model, so it samples directly (below).
      if (d > MESH_POOL_FROM) {
        const cell = meshCol * MESH_RINGS + ringOf[s];
        if (h > mesh[cell]) mesh[cell] = h;
      }

      if (d < occludeFrom) {
        visible[k * STEPS + s] = 1;
        for (let b = 0; b < BANDS.length; b++) if (a > bandSkylines[b][k]) bandSkylines[b][k] = a;
        continue;
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
        // The model reads 0 m over open sea (and outside its tiles, which here is the same thing).
        if (h === 0 && d > 10_000 && (!stats.sea || d < stats.sea.distance)) stats.sea = { distance: d, azimuth: k * RAY_STEP, lat, lon };
        if (onVisible) onVisible(lat, lon, h, d, area, k);
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

  /** Max elevation angle along a bearing between two distances, and where it happens. */
  function profileMax(azimuth, from, to) {
    let best = { angle: -90, distance: 0, lat: 0, lon: 0 };
    for (let d = Math.max(from, occludeFrom); d <= to; d += STEP) {
      const p = destination(summit, azimuth, d);
      const a = angleOf(dem.elevation(p.lat, p.lon), d, EYE_HEIGHT);
      if (a > best.angle) best = { angle: a, distance: d, lat: p.lat, lon: p.lon };
    }
    return best;
  }

  /** How far above (positive) or below the ground in front a point is, in degrees. */
  function sight(point, height) {
    const { azimuth, distance } = bearingDistance(summit, point);
    const angle = angleOf(height, distance, EYE_HEIGHT);
    const front = profileMax(azimuth, START, distance - Math.max(60, 0.002 * distance));
    return { azimuth, distance, angle, margin: angle - front.angle };
  }

  /** Whether the ground at a point is in view, from the rays themselves. */
  function seen(point) {
    const { azimuth, distance } = bearingDistance(summit, point);
    if (distance > RADIUS || distance < START) return false;
    const k = Math.round(azimuth / RAY_STEP) % RAYS;
    const s = Math.round((distance - START) / STEP);
    return !!(visible[k * STEPS + s] || visible[k * STEPS + Math.max(0, s - 1)] || visible[k * STEPS + Math.min(STEPS - 1, s + 1)]);
  }

  return { dem, summit, eyeHeight: EYE_HEIGHT, skyline, skylineDistance, bandSkylines, crests, visible, mesh, stats, profileMax, sight, seen };
}

// ------------------------------------------------------------------------------ ridge lines

/**
 * Links the silhouette edges of neighbouring rays into lines: an edge on one ray continues
 * as the edge on the next ray at about the same distance and angle. What comes out is the
 * drawing a panorama artist would make — one line per ridge that stands in front of another.
 */
function linkRidges(crests) {
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

export function ridgeLines(view) {
  console.time("ridges");
  const ridges = linkRidges(view.crests)
    // A ridge has to run far enough to be worth a line. Near the viewpoint that bar is higher:
    // the model is a surface model, so rows of trees and roofs on the valley floor make little
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
  return ridges;
}

/** One value per ray as a simplified [azimuth, angle] polyline that closes at 360°. */
export const polyline = (values) =>
  simplify(Array.from(values, (a, k) => [k * RAY_STEP, a]).concat([[360, values[0]]]), 0.004).map(([x, y]) => [round(x, 3), round(y, 3)]);

// ------------------------------------------------------------------------------ the summits

export const parseEle = (value) => {
  const n = parseFloat(String(value ?? "").replace(",", "."));
  return Number.isFinite(n) && n > 0 && n < 7000 ? Math.round(n) : null;
};

/**
 * Every OSM summit within reach, placed on the model and judged: visible, borderline or
 * hidden. `isNamed(peak, nearestOfName)` picks the ones the page talks about, which are always
 * kept and get a flank searched for when they don't show.
 */
export function peaksInView(view, osm, { isNamed, flankReach = {} }) {
  const { dem, summit, eyeHeight, profileMax } = view;
  console.time("peaks");
  let peaks = osm
    .map((peak) => {
      const { distance } = bearingDistance(summit, peak);
      if (distance < 800 || distance > RADIUS - 2000) return null;
      const top = snapToTop(dem, peak, 150);
      const { azimuth, distance: d } = bearingDistance(summit, top);
      const angle = angleOf(top.elevation, d, eyeHeight);
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

  // Names repeat (there are seven Chivatos within 160 km): a name means the nearest one.
  const nearestOfName = new Map();
  for (const peak of peaks) {
    const current = nearestOfName.get(peak.name);
    if (!current || peak.distance < current.distance) nearestOfName.set(peak.name, peak);
  }
  for (const peak of peaks) peak.named = !!isNamed(peak, nearestOfName);

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

  // A hidden summit is not always a hidden mountain: its flank or its ridge can still show.
  // For each named summit, find the highest visible ground within 2.5 km of the top.
  console.time("flanks");
  for (const peak of peaks.filter((p) => p.named && p.status !== "visible")) {
    const N = 3600;
    const reach = flankReach[peak.name] ?? 2500;
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
        const seen = view.sight({ lat, lon }, h);
        if (seen.margin > BORDERLINE) best = { lat: round(lat, 5), lon: round(lon, 5), elevation: Math.round(h), away: Math.round(away), azimuth: round(seen.azimuth, 2), angle: round(seen.angle, 3) };
      }
    }
    peak.flank = best;
  }
  console.timeEnd("flanks");
  return peaks;
}

// ------------------------------------------------------------------------------ routes

/**
 * GPS tracks as seen from the viewpoint: where each one runs across the panorama, and which
 * stretches of it are in view. `tracks` are { slug, title, date, points: [{ lat, lon }] }.
 */
/** Trail nearer than this to the viewpoint (m) is left out of the drawing. */
const NEAR_TRAIL = 1000;

export function routesInView(view, tracks, peaks) {
  const { dem, summit } = view;
  console.time("routes");
  const routes = [];
  for (const { slug, title, date, segments } of tracks) {
    // One point every ~40 m of trail is plenty at these distances.
    const thinned = segments.map((segment) => {
      const kept = [];
      for (const point of segment) {
        if (!kept.length || bearingDistance(kept[kept.length - 1], point).distance >= 40) kept.push(point);
      }
      return kept;
    });
    const points = thinned.flat();
    const near = Math.min(...points.map((p) => bearingDistance(summit, p).distance));
    if (near > RADIUS - 2000) continue;

    let length = 0, seenLength = 0;
    const projected = thinned.map((segment) =>
      segment.map((point, i) => {
        const h = dem.elevation(point.lat, point.lon) + EYE;
        const seen = view.sight(point, h);
        const step = i ? bearingDistance(segment[i - 1], point).distance : 0;
        length += step;
        if (seen.margin > 0) seenLength += step;
        return [seen.azimuth, seen.angle, seen.margin > 0 ? 1 : 0, seen.distance];
      }),
    );
    // Simplify each run of visible or hidden trail separately, so the switch points survive.
    // A new segment (another day, another file) starts a new run: the gap between them wasn't walked.
    // Trail right under the viewpoint isn't drawn: there each step swings tens of degrees of
    // bearing and angle, and the line would cut straight across the strip.
    const runs = [];
    for (const segment of projected) {
      let run = null;
      for (const point of segment) {
        if (point[3] < NEAR_TRAIL) run = null;
        else if (run && run.visible === point[2]) run.points.push(point);
        else {
          run = { visible: point[2], points: run ? [run.points[run.points.length - 1], point] : [point] };
          runs.push(run);
        }
      }
    }
    const summits = peaks
      .filter((peak) => points.some((point) => bearingDistance(point, peak).distance < 150))
      .map((peak) => peak.name);
    for (const peak of peaks) if (summits.includes(peak.name)) peak.ascent ??= { slug, title };
    routes.push({
      slug,
      title,
      date,
      distance: [round(near / 1000, 1), round(Math.max(...projected.flat().map((p) => p[3])) / 1000, 1)],
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
  return routes;
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
export function png(width, height, channels, pixels) {
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

/**
 * The 3D terrain: one column per bearing, one row per ring, elevation in centimetres packed
 * big-endian into R, G and B. Whole metres would do for the mountains but not for the ground
 * at your feet, where the rings are only a few metres apart: there even a 10 cm step tilts
 * the surface by degrees and draws terraces into the shading.
 */
export function terrainPng(view) {
  const pixels = new Uint8Array(MESH_COLS * MESH_RINGS * 3);
  for (let r = 0; r < MESH_RINGS; r++) {
    for (let c = 0; c < MESH_COLS; c++) {
      const v = Math.max(0, Math.min(0xffffff, Math.round(view.mesh[c * MESH_RINGS + r] * 100)));
      const i = (r * MESH_COLS + c) * 3;
      pixels[i] = v >> 16;
      pixels[i + 1] = (v >> 8) & 0xff;
      pixels[i + 2] = v & 0xff;
    }
  }
  const file = png(MESH_COLS, MESH_RINGS, 3, pixels);
  console.log(`terrain.png ${MESH_COLS}×${MESH_RINGS}, ${(file.length / 1024).toFixed(0)} KB`);
  return file;
}

/** What the 3D view needs to read terrainPng's file, which lives at `file`. */
export const meshInfo = (file) => ({
  encoding: "centimetres, 24-bit big-endian RGB",
  cols: MESH_COLS,
  rings: MESH_RINGS,
  azStep: MESH_AZ_STEP,
  r0: MESH_R0,
  ratio: MESH_RATIO,
  effectiveRadius: Math.round(EFFECTIVE),
  file,
});

/**
 * The viewshed, rasterised in Web Mercator so a Leaflet image overlay lands exactly. With
 * `ice(lat, lon)`, glacier ice is drawn too: bright where it is in view, faint where it isn't.
 */
export function viewshedPng(view, { ice } = {}) {
  const { summit } = view;
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
      const { distance } = bearingDistance(summit, { lat, lon });
      if (distance > RADIUS || distance < START) continue;
      const inView = view.seen({ lat, lon });
      const glacier = ice ? ice(lat, lon) : false;
      if (!inView && !glacier) continue;
      const i = (y * width + x) * 4;
      if (glacier) {
        pixels[i] = inView ? 64 : 120;
        pixels[i + 1] = inView ? 200 : 170;
        pixels[i + 2] = 255;
        pixels[i + 3] = inView ? 235 : 110;
      } else {
        pixels[i] = 242;
        pixels[i + 1] = 166;
        pixels[i + 2] = 28;
        pixels[i + 3] = 205;
      }
    }
  }
  const file = png(width, height, 4, pixels);
  console.log(`viewshed.png ${width}×${height}, ${(file.length / 1024).toFixed(0)} KB`);
  return { file, bounds: { south: round(south, 5), north: round(north, 5), west: round(west, 5), east: round(east, 5) } };
}

/** The summary numbers of a view: area in view, the farthest ground, the sea if it shows. */
export function viewStats(view) {
  const { stats, skyline } = view;
  const visibleKm2 = stats.visibleArea / 1e6;
  return {
    visibleArea: Math.round(visibleKm2),
    visibleShare: round(visibleKm2 / ((Math.PI * RADIUS * RADIUS) / 1e6), 4),
    above3000: Math.round(stats.above3000 / 1e6),
    farthest: { ...stats.farthest, distance: round(stats.farthest.distance / 1000, 1), lat: round(stats.farthest.lat, 4), lon: round(stats.farthest.lon, 4) },
    highestSkyline: round(Math.max(...skyline), 2),
    lowestSkyline: round(Math.min(...skyline), 2),
  };
}
