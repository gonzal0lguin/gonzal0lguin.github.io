import { WAYPOINT_KINDS, waypointKind, type WaypointKind } from "./waypoint-kinds";

// GPX parsing, stitching and route analysis. No Node APIs, so the same code runs at build
// time for the posts (src/lib/gpx.ts reads the files) and in the browser for the route tool.

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
  waypoints: Waypoint[];
  hasElevation: boolean;
  hasTime: boolean;
  hasHeartRate: boolean;
}

export interface RawPoint {
  lat: number;
  lon: number;
  ele: number | null;
  time: number | null; // unix seconds
  hr: number | null;
}

/** A marked point: from the file's <wpt> elements, or added in the route tool. */
export interface Waypoint {
  lat: number;
  lon: number;
  name: string;
  kind: WaypointKind;
  ele?: number | null;
  /** Unix seconds */
  time?: number | null;
  desc?: string;
}

export interface GpxTrack {
  name: string | null;
  points: RawPoint[];
  waypoints: Waypoint[];
}

// A pause longer than this splits the route into days (an overnight camp or bivouac).
const DAY_BREAK_SECONDS = 3 * 3600;
// Ignore elevation wiggles smaller than this when adding up gain/loss.
const ELEVATION_HYSTERESIS_M = 3;
// Segments slower than this (3D speed) or with gaps longer than MAX_MOVING_GAP don't count as moving.
const MIN_MOVING_SPEED_MS = 0.2;
const MAX_MOVING_GAP_S = 300;
const SPEED_WINDOW_S = 60;
const GRADE_WINDOW_M = 100;
const MAX_CLIENT_POINTS = 1500;

// --- Parsing

/** Points of the file's tracks (or of its route, if it has no track), plus named waypoints. */
export function parseGpx(xml: string): GpxTrack {
  const readPoints = (tag: string) => {
    const points: (RawPoint & { body: string })[] = [];
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
        body,
      });
    }
    return points;
  };
  const strip = ({ body, ...point }: RawPoint & { body: string }): RawPoint => point;
  const tagText = (body: string, tag: string) => decode(body.match(new RegExp(`<${tag}>([\\s\\S]*?)<\\/${tag}>`))?.[1] ?? "");

  let points = readPoints("trkpt").map(strip);
  if (points.length < 2) points = readPoints("rtept").map(strip);

  const waypoints: Waypoint[] = readPoints("wpt").flatMap(({ lat, lon, ele, time, body }) => {
    const name = tagText(body, "name");
    if (!name) return [];
    const desc = tagText(body, "desc") || tagText(body, "cmt");
    return [{ lat, lon, name, ele, time, kind: waypointKind(tagText(body, "type"), tagText(body, "sym")), ...(desc ? { desc } : {}) }];
  });

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

/**
 * How the route's highest point is shown. It's the summit, unless a summit is marked elsewhere
 * (one that wasn't reached, or a separate peak): then it's just the high point. If a marked
 * summit sits on it, the marked one is shown instead.
 */
export function highPointRole(summit: RouteMarker | null, waypoints: Waypoint[]): "summit" | "highPoint" | null {
  if (!summit) return null;
  const marked = waypoints.filter((w) => w.kind === "summit");
  if (marked.some((w) => haversine(w, summit) < 200)) return null;
  return marked.length ? "highPoint" : "summit";
}

/** Meters between two points */
export function haversine(a: { lat: number; lon: number }, b: { lat: number; lon: number }) {
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

/**
 * Join a route recorded in several files into one. When every file has timestamps they are put
 * in chronological order (whatever order they were given in) and points that overlap the previous
 * file are dropped; otherwise the given order is kept. The pauses between files then show up as
 * stops, or as a new day when they're long enough.
 */
export function stitchTracks(tracks: GpxTrack[]): GpxTrack {
  const parts = tracks.filter((t) => t.points.length > 0);
  if (parts.length <= 1) return parts[0] ?? tracks[0];
  const timed = parts.every((t) => t.points.every((p) => p.time != null));
  if (timed) parts.sort((a, b) => (a.points[0].time as number) - (b.points[0].time as number));

  const points: RawPoint[] = [];
  for (const part of parts) {
    for (const p of part.points) {
      const last = points[points.length - 1];
      if (timed && last && (p.time as number) <= (last.time as number)) continue;
      points.push(p);
    }
  }
  const seen = new Set<string>();
  const waypoints = parts
    .flatMap((t) => t.waypoints)
    .filter((w) => {
      const id = `${w.name}|${w.lat.toFixed(5)}|${w.lon.toFixed(5)}`;
      if (seen.has(id)) return false;
      seen.add(id);
      return true;
    });
  return { name: parts.find((t) => t.name)?.name ?? null, points, waypoints };
}

/** A GPX 1.1 document with the track as a single segment (heart rate as a Garmin extension). */
export function toGpx({ name, points, waypoints }: GpxTrack): string {
  const hasHr = points.some((p) => p.hr != null);
  const lines = [
    '<?xml version="1.0" encoding="UTF-8"?>',
    `<gpx version="1.1" creator="gonzal0lguin.github.io" xmlns="http://www.topografix.com/GPX/1/1"${hasHr ? ' xmlns:gpxtpx="http://www.garmin.com/xmlschemas/TrackPointExtension/v1"' : ""}>`,
    ...waypoints.map((w) => {
      let body = "";
      if (w.ele != null) body += `<ele>${round(w.ele, 1)}</ele>`;
      if (w.time != null) body += `<time>${isoTime(w.time)}</time>`;
      body += `<name>${escapeXml(w.name)}</name>`;
      if (w.desc) body += `<desc>${escapeXml(w.desc)}</desc>`;
      body += `<sym>${WAYPOINT_KINDS[w.kind].sym}</sym><type>${w.kind}</type>`;
      return `  <wpt lat="${w.lat}" lon="${w.lon}">${body}</wpt>`;
    }),
    "  <trk>",
    ...(name ? [`    <name>${escapeXml(name)}</name>`] : []),
    "    <trkseg>",
    ...points.map((p) => {
      let body = "";
      if (p.ele != null) body += `<ele>${p.ele}</ele>`;
      if (p.time != null) body += `<time>${isoTime(p.time)}</time>`;
      if (p.hr != null) body += `<extensions><gpxtpx:TrackPointExtension><gpxtpx:hr>${p.hr}</gpxtpx:hr></gpxtpx:TrackPointExtension></extensions>`;
      return `      <trkpt lat="${p.lat}" lon="${p.lon}">${body}</trkpt>`;
    }),
    "    </trkseg>",
    "  </trk>",
    "</gpx>",
    "",
  ];
  return lines.join("\n");
}

const isoTime = (unixSeconds: number) => new Date(unixSeconds * 1000).toISOString().replace(".000Z", "Z");

function escapeXml(text: string) {
  return text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

export function analyzeTrack({ name, points: raw, waypoints }: GpxTrack): RouteData {
  if (raw.length < 2) throw new Error("GPX file has no track or route with at least 2 points");
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

  // Each segment is judged by the speed over about a minute around it: in dense recordings
  // (a point every few seconds) a slow, steep climb moves so little between points that
  // segment by segment it would look like standing still.
  const time = (i: number) => raw[i].time as number;
  const movingTimeBetween = (from: number, to: number) => {
    let moving = 0;
    for (let i = from + 1; i <= to; i++) {
      const dt = time(i) - time(i - 1);
      if (dt <= 0 || dt > MAX_MOVING_GAP_S) continue;
      let a = i - 1;
      let b = i;
      while (a > from && time(i - 1) - time(a) < SPEED_WINDOW_S / 2 && time(a) - time(a - 1) <= MAX_MOVING_GAP_S) a--;
      while (b < to && time(b) - time(i) < SPEED_WINDOW_S / 2 && time(b + 1) - time(b) <= MAX_MOVING_GAP_S) b++;
      const dh = dist[b] - dist[a];
      const dv = raw[a].ele != null && raw[b].ele != null ? (raw[b].ele as number) - (raw[a].ele as number) : 0;
      if (Math.hypot(dh, dv) / (time(b) - time(a)) >= MIN_MOVING_SPEED_MS) moving += dt;
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
