// Build-time GPX parsing and route analysis for the mountaineering pages.
// Reads a .gpx file from public/, computes stats (distance, elevation gain, moving time,
// days/camps, heart rate...) and returns a compact payload for the RouteMap component.

import { readFileSync } from "node:fs";
import path from "node:path";

/** Tuple per point sent to the browser: [lat, lon, ele, distance m, seconds since start, heart rate, grade %] */
export type RoutePoint = [number, number, number | null, number, number | null, number | null, number | null];

export interface RouteDay {
  /** Unix seconds */
  start: number;
  end: number;
  /** Distance along the route (m) where the day starts / ends */
  fromDistance: number;
  toDistance: number;
  distance: number;
  gain: number | null;
  loss: number | null;
  movingTime: number;
}

export interface RouteMarker {
  lat: number;
  lon: number;
  ele: number | null;
  /** Distance along the route (m) */
  distance: number;
}

export interface RouteStats {
  distance: number;
  gain: number | null;
  loss: number | null;
  maxEle: number | null;
  minEle: number | null;
  /** Total elapsed time and time spent moving (s); null when the file has no usable timestamps */
  totalTime: number | null;
  movingTime: number | null;
  avgHr: number | null;
  maxHr: number | null;
  /** Steepest sustained grade over ~100 m, as a fraction */
  maxGrade: number | null;
  /** Hiking time estimate following DIN 33466 (the German Alpine Club's formula), in seconds */
  estimatedTime: number | null;
  shape: "loop" | "out-and-back" | "one-way";
}

export interface RouteData {
  name: string | null;
  /** Unix seconds of the first point, if timestamps exist */
  startTime: number | null;
  points: RoutePoint[];
  stats: RouteStats;
  days: RouteDay[];
  summit: RouteMarker | null;
  camps: RouteMarker[];
  waypoints: { lat: number; lon: number; name: string }[];
  hasElevation: boolean;
  hasTime: boolean;
  hasHeartRate: boolean;
}

interface RawPoint {
  lat: number;
  lon: number;
  ele: number | null;
  time: number | null; // unix seconds
  hr: number | null;
}

// A pause longer than this splits the route into days (an overnight camp or bivouac).
const DAY_BREAK_SECONDS = 3 * 3600;
// Ignore elevation wiggles smaller than this when adding up gain/loss.
const ELEVATION_HYSTERESIS_M = 3;
// Segments slower than this (3D speed) or with gaps longer than MAX_MOVING_GAP don't count as moving.
const MIN_MOVING_SPEED_MS = 0.2;
const MAX_MOVING_GAP_S = 300;
const GRADE_WINDOW_M = 100;
const MAX_CLIENT_POINTS = 1500;

const cache = new Map<string, RouteData>();

/** Load and analyze a GPX file referenced by its public URL, e.g. "/assets/tracks/route.gpx". */
export function loadRoute(publicPath: string): RouteData {
  const cached = cache.get(publicPath);
  if (cached) return cached;

  const file = path.join(process.cwd(), "public", publicPath.replace(/^\/+/, ""));
  let xml: string;
  try {
    xml = readFileSync(file, "utf8");
  } catch {
    throw new Error(`Route file not found: "${publicPath}" (expected at ${file})`);
  }
  const data = analyze(parseGpx(xml));
  cache.set(publicPath, data);
  return data;
}

// --- Parsing

function parseGpx(xml: string) {
  const readPoints = (tag: string) => {
    const points: RawPoint[] = [];
    const re = new RegExp(`<${tag}\\b([^>]*?)(?:\\/>|>([\\s\\S]*?)<\\/${tag}>)`, "g");
    for (const [, attrs, body = ""] of xml.matchAll(re)) {
      const lat = Number(attrs.match(/\blat\s*=\s*["']([^"']+)["']/)?.[1]);
      const lon = Number(attrs.match(/\blon\s*=\s*["']([^"']+)["']/)?.[1]);
      if (!Number.isFinite(lat) || !Number.isFinite(lon)) continue;
      const ele = Number(body.match(/<ele>\s*([^<]+?)\s*<\/ele>/)?.[1]);
      const time = Date.parse(body.match(/<time>\s*([^<]+?)\s*<\/time>/)?.[1] ?? "");
      const hr = Number(body.match(/<(?:\w+:)?hr>\s*(\d+)\s*</)?.[1]);
      points.push({
        lat,
        lon,
        ele: Number.isFinite(ele) ? ele : null,
        time: Number.isFinite(time) ? time / 1000 : null,
        hr: Number.isFinite(hr) && hr > 0 ? hr : null,
        // carry the raw tag body for waypoint names
        ...(tag === "wpt" ? { name: decode(body.match(/<name>([\s\S]*?)<\/name>/)?.[1] ?? "") } : {}),
      });
    }
    return points;
  };

  let points = readPoints("trkpt");
  if (points.length < 2) points = readPoints("rtept");
  if (points.length < 2) throw new Error("GPX file has no track or route with at least 2 points");

  const waypoints = (readPoints("wpt") as (RawPoint & { name: string })[])
    .filter((w) => w.name)
    .map((w) => ({ lat: w.lat, lon: w.lon, name: w.name }));

  const name = decode(xml.match(/<(?:metadata|trk|rte)\b[^>]*>[\s\S]*?<name>([\s\S]*?)<\/name>/)?.[1] ?? "") || null;
  return { name, points, waypoints };
}

function decode(text: string) {
  return text
    .replace(/<!\[CDATA\[([\s\S]*?)\]\]>/g, "$1")
    .replace(/&lt;/g, "<")
    .replace(/&gt;/g, ">")
    .replace(/&quot;/g, '"')
    .replace(/&apos;/g, "'")
    .replace(/&amp;/g, "&")
    .trim();
}

// --- Analysis

function haversine(a: RawPoint, b: RawPoint) {
  const R = 6371008.8;
  const toRad = Math.PI / 180;
  const dLat = (b.lat - a.lat) * toRad;
  const dLon = (b.lon - a.lon) * toRad;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * toRad) * Math.cos(b.lat * toRad) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(h));
}

/** Elevation gain and loss with a hysteresis threshold, so sensor noise doesn't add up. */
function gainLoss(elevations: (number | null)[]) {
  let gain = 0;
  let loss = 0;
  let ref: number | null = null;
  for (const ele of elevations) {
    if (ele == null) continue;
    if (ref == null) ref = ele;
    else if (ele - ref >= ELEVATION_HYSTERESIS_M) (gain += ele - ref), (ref = ele);
    else if (ref - ele >= ELEVATION_HYSTERESIS_M) (loss += ref - ele), (ref = ele);
  }
  return { gain: Math.round(gain), loss: Math.round(loss) };
}

/** DIN 33466: 4 km/h on the flat, 300 m/h up, 500 m/h down; the smaller part counts half. */
function din33466(distance: number, gain: number, loss: number) {
  const horizontal = distance / 1000 / 4;
  const vertical = gain / 300 + loss / 500;
  return (Math.max(horizontal, vertical) + Math.min(horizontal, vertical) / 2) * 3600;
}

function analyze({ name, points: raw, waypoints }: ReturnType<typeof parseGpx>): RouteData {
  const n = raw.length;
  const dist = new Array<number>(n).fill(0);
  for (let i = 1; i < n; i++) dist[i] = dist[i - 1] + haversine(raw[i - 1], raw[i]);
  const total = dist[n - 1];

  const hasElevation = raw.filter((p) => p.ele != null).length > n / 2;
  const times = raw.map((p) => p.time);
  // Only trust timestamps that are present and in order (some exported routes have random ones).
  const hasTime = times.every((t, i) => t != null && (i === 0 || t >= (times[i - 1] as number)));
  const hasHeartRate = raw.filter((p) => p.hr != null).length > n / 2;

  // Grade over a window of ~GRADE_WINDOW_M centered on each point.
  const grade: (number | null)[] = raw.map((_, i) => {
    if (!hasElevation) return null;
    let a = i;
    let b = i;
    while (dist[b] - dist[a] < GRADE_WINDOW_M && (a > 0 || b < n - 1)) {
      if (a > 0) a--;
      if (b < n - 1 && dist[b] - dist[a] < GRADE_WINDOW_M) b++;
    }
    const ea = raw[a].ele;
    const eb = raw[b].ele;
    const run = dist[b] - dist[a];
    return ea != null && eb != null && run > 0 ? (eb - ea) / run : null;
  });

  // Days: split wherever the recording pauses for longer than DAY_BREAK_SECONDS.
  const dayRanges: [number, number][] = [];
  if (hasTime) {
    let start = 0;
    for (let i = 1; i < n; i++) {
      if ((raw[i].time as number) - (raw[i - 1].time as number) > DAY_BREAK_SECONDS) {
        dayRanges.push([start, i - 1]);
        start = i;
      }
    }
    dayRanges.push([start, n - 1]);
  }

  const movingTimeBetween = (from: number, to: number) => {
    let moving = 0;
    for (let i = from + 1; i <= to; i++) {
      const dt = (raw[i].time as number) - (raw[i - 1].time as number);
      if (dt <= 0 || dt > MAX_MOVING_GAP_S) continue;
      const dh = dist[i] - dist[i - 1];
      const dv = raw[i].ele != null && raw[i - 1].ele != null ? (raw[i].ele as number) - (raw[i - 1].ele as number) : 0;
      if (Math.hypot(dh, dv) / dt >= MIN_MOVING_SPEED_MS) moving += dt;
    }
    return moving;
  };

  const days: RouteDay[] = dayRanges.map(([from, to]) => {
    const { gain, loss } = gainLoss(raw.slice(from, to + 1).map((p) => p.ele));
    return {
      start: Math.round(raw[from].time as number),
      end: Math.round(raw[to].time as number),
      fromDistance: Math.round(dist[from]),
      toDistance: Math.round(dist[to]),
      distance: Math.round(dist[to] - dist[from]),
      gain: hasElevation ? gain : null,
      loss: hasElevation ? loss : null,
      movingTime: Math.round(movingTimeBetween(from, to)),
    };
  });

  const marker = (i: number): RouteMarker => ({ lat: raw[i].lat, lon: raw[i].lon, ele: raw[i].ele, distance: Math.round(dist[i]) });
  const camps = dayRanges.slice(0, -1).map(([, to]) => marker(to));

  let summit: RouteMarker | null = null;
  const elevations = raw.map((p) => p.ele).filter((e): e is number => e != null);
  if (hasElevation) {
    let top = 0;
    raw.forEach((p, i) => {
      if (p.ele != null && p.ele > (raw[top].ele ?? -Infinity)) top = i;
    });
    summit = marker(top);
  }

  const { gain, loss } = gainLoss(raw.map((p) => p.ele));
  const hrs = raw.map((p) => p.hr).filter((h): h is number => h != null);
  const sustainedGrades = grade.filter((g): g is number => g != null).map(Math.abs);

  const stats: RouteStats = {
    distance: Math.round(total),
    gain: hasElevation ? gain : null,
    loss: hasElevation ? loss : null,
    maxEle: hasElevation ? Math.max(...elevations) : null,
    minEle: hasElevation ? Math.min(...elevations) : null,
    totalTime: hasTime ? Math.round((raw[n - 1].time as number) - (raw[0].time as number)) : null,
    movingTime: hasTime ? days.reduce((acc, d) => acc + d.movingTime, 0) : null,
    avgHr: hasHeartRate ? Math.round(hrs.reduce((a, b) => a + b, 0) / hrs.length) : null,
    maxHr: hasHeartRate ? Math.max(...hrs) : null,
    maxGrade: sustainedGrades.length ? Math.max(...sustainedGrades) : null,
    estimatedTime: hasElevation ? Math.round(din33466(total, gain, loss)) : Math.round(din33466(total, 0, 0)),
    shape: routeShape(raw, dist),
  };

  // Thin out the points sent to the browser, keeping one every `step` meters.
  const step = Math.max(5, total / MAX_CLIENT_POINTS);
  const keep: number[] = [0];
  for (let i = 1; i < n - 1; i++) if (dist[i] - dist[keep[keep.length - 1]] >= step) keep.push(i);
  keep.push(n - 1);

  const t0 = raw[0].time;
  const points: RoutePoint[] = keep.map((i) => [
    round(raw[i].lat, 6),
    round(raw[i].lon, 6),
    raw[i].ele == null ? null : round(raw[i].ele as number, 1),
    Math.round(dist[i]),
    hasTime ? Math.round((raw[i].time as number) - (t0 as number)) : null,
    hasHeartRate ? raw[i].hr : null,
    grade[i] == null ? null : round((grade[i] as number) * 100, 1),
  ]);

  return {
    name,
    startTime: hasTime ? Math.round(t0 as number) : null,
    points,
    stats,
    days: days.length > 1 ? days : [],
    summit,
    camps,
    waypoints,
    hasElevation,
    hasTime,
    hasHeartRate,
  };
}

/** Loop (ends where it starts), out-and-back (comes back on its own trail) or one-way. */
function routeShape(raw: RawPoint[], dist: number[]): RouteStats["shape"] {
  const total = dist[dist.length - 1];
  if (haversine(raw[0], raw[raw.length - 1]) > Math.max(250, total * 0.05)) return "one-way";

  // How much of the second half retraces the first half (within 40 m)?
  const half = dist.findIndex((d) => d >= total / 2);
  const outbound = raw.slice(0, half).filter((_, i) => i % 3 === 0);
  const inbound = raw.slice(half).filter((_, i) => i % 3 === 0);
  const retraced = inbound.filter((p) => outbound.some((q) => haversine(p, q) < 40)).length;
  return retraced / Math.max(1, inbound.length) > 0.6 ? "out-and-back" : "loop";
}

function round(value: number, decimals: number) {
  const f = 10 ** decimals;
  return Math.round(value * f) / f;
}
