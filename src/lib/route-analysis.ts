// Deeper analysis of a route, computed in the browser from the points RouteMap already has:
// sections, climbs and descents, splits, stops, grade and altitude distributions, heart rate
// zones, effort, sunlight and a time plan. Pure functions, no DOM.

import type { RouteData, RoutePoint } from "./gpx-core";

export const LAT = 0, LON = 1, ELE = 2, DIST = 3, TIME = 4, HR = 5, GRADE = 6;

// Gaps longer than this between two points count as stopped (same rule as gpx-core's moving time).
const MAX_MOVING_GAP_S = 300;
const MIN_MOVING_SPEED_MS = 0.2;

// --- Sections

export interface SectionStats {
  from: number;
  to: number;
  distance: number;
  gain: number;
  loss: number;
  /** Net elevation change over distance, as a fraction */
  avgGrade: number | null;
  /** Steepest point grade inside the section (already smoothed over ~100 m), fraction */
  maxGrade: number | null;
  minEle: number | null;
  maxEle: number | null;
  duration: number | null;
  movingTime: number | null;
  /** Average speed while moving, m/s */
  speed: number | null;
  /** Vertical speed: meters climbed (or descended) per hour of moving time */
  vam: number | null;
  avgHr: number | null;
}

/** Stats for points[from..to], inclusive. */
export function sectionStats(points: RoutePoint[], from: number, to: number): SectionStats {
  const a = Math.max(0, Math.min(from, to));
  const b = Math.min(points.length - 1, Math.max(from, to));
  let gain = 0, loss = 0, moving = 0, ref: number | null = null;
  let minEle: number | null = null, maxEle: number | null = null, maxGrade: number | null = null;
  let hrSum = 0, hrCount = 0;
  for (let i = a; i <= b; i++) {
    const p = points[i];
    const ele = p[ELE];
    if (ele != null) {
      minEle = minEle == null ? ele : Math.min(minEle, ele);
      maxEle = maxEle == null ? ele : Math.max(maxEle, ele);
      if (ref == null) ref = ele;
      else if (ele - ref >= 3) (gain += ele - ref), (ref = ele);
      else if (ref - ele >= 3) (loss += ref - ele), (ref = ele);
    }
    if (p[GRADE] != null && (maxGrade == null || Math.abs(p[GRADE]!) > Math.abs(maxGrade))) maxGrade = p[GRADE]!;
    if (p[HR] != null) (hrSum += p[HR]!), hrCount++;
    if (i > a) moving += movingSeconds(points, i);
  }
  const distance = points[b][DIST] - points[a][DIST];
  const eA = points[a][ELE];
  const eB = points[b][ELE];
  const timed = points[a][TIME] != null && points[b][TIME] != null;
  const net = eA != null && eB != null ? eB - eA : null;
  return {
    from: a,
    to: b,
    distance,
    gain: Math.round(gain),
    loss: Math.round(loss),
    avgGrade: net != null && distance > 0 ? net / distance : null,
    maxGrade: maxGrade == null ? null : maxGrade / 100,
    minEle,
    maxEle,
    duration: timed ? points[b][TIME]! - points[a][TIME]! : null,
    movingTime: timed ? moving : null,
    speed: timed && moving > 0 ? distance / moving : null,
    vam: timed && moving > 0 ? (Math.max(gain, loss) / moving) * 3600 : null,
    avgHr: hrCount ? Math.round(hrSum / hrCount) : null,
  };
}

/** Seconds of movement between points i-1 and i (0 when stopped or recording was paused). */
function movingSeconds(points: RoutePoint[], i: number) {
  const t0 = points[i - 1][TIME];
  const t1 = points[i][TIME];
  if (t0 == null || t1 == null) return 0;
  const dt = t1 - t0;
  if (dt <= 0 || dt > MAX_MOVING_GAP_S) return 0;
  const dh = points[i][DIST] - points[i - 1][DIST];
  const e0 = points[i - 1][ELE];
  const e1 = points[i][ELE];
  const dv = e0 != null && e1 != null ? e1 - e0 : 0;
  return Math.hypot(dh, dv) / dt >= MIN_MOVING_SPEED_MS ? dt : 0;
}

// --- Per-point series for coloring the route

/** Speed in km/h, averaged over about two minutes; null without timestamps or while stopped. */
export function speedSeries(points: RoutePoint[]) {
  return windowed(points, 60, (a, b, dt) => ((points[b][DIST] - points[a][DIST]) / dt) * 3.6);
}

/** Vertical speed in m/h (positive climbing, negative descending), over about three minutes. */
export function verticalSpeedSeries(points: RoutePoint[]) {
  return windowed(points, 90, (a, b, dt) => {
    const e0 = points[a][ELE];
    const e1 = points[b][ELE];
    return e0 == null || e1 == null ? null : ((e1 - e0) / dt) * 3600;
  });
}

function windowed(points: RoutePoint[], halfWindow: number, value: (a: number, b: number, dt: number) => number | null) {
  if (points[0][TIME] == null) return points.map(() => null);
  const n = points.length;
  return points.map((p, i) => {
    const t = p[TIME]!;
    let a = i;
    let b = i;
    while (a > 0 && t - points[a - 1][TIME]! <= halfWindow && points[a][TIME]! - points[a - 1][TIME]! <= MAX_MOVING_GAP_S) a--;
    while (b < n - 1 && points[b + 1][TIME]! - t <= halfWindow && points[b + 1][TIME]! - points[b][TIME]! <= MAX_MOVING_GAP_S) b++;
    const dt = points[b][TIME]! - points[a][TIME]!;
    if (dt < 20) return null;
    return value(a, b, dt);
  });
}

// --- Climbs and descents

export interface Climb extends SectionStats {
  direction: "up" | "down";
}

/**
 * Significant climbs and descents: stretches where the elevation keeps going one way, allowing
 * small counter-moves. A stretch counts when it gains (or loses) at least `minChange` meters.
 */
export function detectClimbs(points: RoutePoint[], totalGain: number): Climb[] {
  const ele = points.map((p) => p[ELE]);
  if (ele.every((e) => e == null)) return [];
  const minChange = Math.max(60, totalGain * 0.08);
  const result: Climb[] = [];
  // Track the running low and high; a reversal larger than `tolerance` closes the current stretch.
  let low = firstIndex(ele);
  let high = low;
  let direction: "up" | "down" | null = null;
  const tolerance = (size: number) => Math.max(25, size * 0.15);

  const close = (from: number, to: number, dir: "up" | "down") => {
    const change = Math.abs((ele[to] as number) - (ele[from] as number));
    if (change >= minChange) result.push({ ...sectionStats(points, from, to), direction: dir });
  };

  for (let i = low + 1; i < points.length; i++) {
    const e = ele[i];
    if (e == null) continue;
    if (direction !== "down") {
      if (e > (ele[high] as number)) high = i;
      if (e < (ele[low] as number) && direction == null) low = high = i;
      if (direction == null && (ele[high] as number) - (ele[low] as number) >= 25) direction = "up";
      if (direction === "up" && (ele[high] as number) - e > tolerance((ele[high] as number) - (ele[low] as number))) {
        close(low, high, "up");
        direction = "down";
        low = high; // `low` now marks the start of the descent
        high = i;   // and `high` its lowest point so far
      }
    } else {
      if (e < (ele[high] as number)) high = i;
      if (e - (ele[high] as number) > tolerance((ele[low] as number) - (ele[high] as number))) {
        close(low, high, "down");
        direction = "up";
        low = high;
        high = i;
      }
    }
  }
  if (direction) close(low, high, direction);
  return result;
}

function firstIndex(values: (number | null)[]) {
  return Math.max(0, values.findIndex((v) => v != null));
}

// --- Splits

export interface Split extends SectionStats {
  index: number;
}

/** Stats for every `every` meters along the route (the last split may be shorter). */
export function splits(points: RoutePoint[], every: number): Split[] {
  const total = points[points.length - 1][DIST];
  const result: Split[] = [];
  let from = 0;
  for (let k = 1; from < points.length - 1; k++) {
    const target = Math.min(k * every, total);
    let to = from;
    while (to < points.length - 1 && points[to][DIST] < target) to++;
    if (points[to][DIST] - points[from][DIST] < every * 0.15 && result.length) {
      // A tiny leftover: fold it into the previous split.
      const last = result.pop()!;
      result.push({ ...sectionStats(points, last.from, to), index: last.index });
    } else {
      result.push({ ...sectionStats(points, from, to), index: k });
    }
    from = to;
  }
  return result;
}

export function splitDistance(total: number) {
  return total <= 12000 ? 1000 : total <= 30000 ? 2000 : total <= 60000 ? 5000 : 10000;
}

// --- Stops

export interface Stop {
  index: number;
  start: number;
  duration: number;
  lat: number;
  lon: number;
  ele: number | null;
  distance: number;
}

/**
 * Places where no progress was made for at least `minSeconds`: windows of points that stay within
 * 40 m along the route, minus the time walking those meters takes. Longer pauses than `maxSeconds`
 * are the overnight camps, already shown as days.
 */
export function detectStops(points: RoutePoint[], startTime: number | null, minSeconds = 420, maxSeconds = 3 * 3600): Stop[] {
  if (startTime == null || points[0][TIME] == null) return [];
  const stops: Stop[] = [];
  let s = 0;
  while (s < points.length - 1) {
    let e = s;
    while (e < points.length - 1 && points[e + 1][DIST] - points[s][DIST] < 40) e++;
    const duration = points[e][TIME]! - points[s][TIME]! - (points[e][DIST] - points[s][DIST]) / 1.0;
    if (e > s && duration >= minSeconds && duration < maxSeconds) {
      const p = points[s];
      const previous = stops[stops.length - 1];
      // Wandering around a camp or a summit: one stop, not several short ones.
      if (previous && p[DIST] - previous.distance < 200 && startTime + p[TIME]! - (previous.start + previous.duration) < 600) {
        previous.duration = startTime + points[e][TIME]! - previous.start;
      } else {
        stops.push({ index: s, start: startTime + p[TIME]!, duration, lat: p[LAT], lon: p[LON], ele: p[ELE], distance: p[DIST] });
      }
      s = e;
    } else {
      s++;
    }
  }
  return stops;
}

// --- Distributions

export interface Band {
  /** Lower and upper limits; ±Infinity at the ends */
  from: number;
  to: number;
  distance: number;
  /** Seconds, null without timestamps */
  time: number | null;
}

const GRADE_LIMITS = [-Infinity, -30, -20, -10, -3, 3, 10, 20, 30, Infinity];

/** Distance (and time) spent in each grade band, steep descents to steep climbs. Grades in %. */
export function gradeBands(points: RoutePoint[]): Band[] {
  const bands = GRADE_LIMITS.slice(0, -1).map((from, k) => ({ from, to: GRADE_LIMITS[k + 1], distance: 0, time: points[0][TIME] == null ? null : 0 }));
  for (let i = 1; i < points.length; i++) {
    const g = points[i][GRADE];
    if (g == null) continue;
    const band = bands.find((b) => g >= b.from && g < b.to)!;
    band.distance += points[i][DIST] - points[i - 1][DIST];
    if (band.time != null) band.time += movingSeconds(points, i);
  }
  return bands;
}

/** Distance and moving time per altitude band of `step` meters. */
export function altitudeBands(points: RoutePoint[], step: number): Band[] {
  const map = new Map<number, Band>();
  for (let i = 1; i < points.length; i++) {
    const e = points[i][ELE];
    if (e == null) continue;
    const from = Math.floor(e / step) * step;
    const band = map.get(from) ?? { from, to: from + step, distance: 0, time: points[0][TIME] == null ? null : 0 };
    band.distance += points[i][DIST] - points[i - 1][DIST];
    if (band.time != null) band.time += movingSeconds(points, i);
    map.set(from, band);
  }
  return [...map.values()].sort((a, b) => b.from - a.from);
}

export function altitudeStep(minEle: number, maxEle: number) {
  const span = maxEle - minEle;
  return span > 2500 ? 500 : span > 1000 ? 250 : span > 400 ? 100 : 50;
}

/** Moving time in each heart rate zone (50-60-70-80-90-100 % of `maxHr`). */
export function heartRateZones(points: RoutePoint[], maxHr: number) {
  const zones = [0.5, 0.6, 0.7, 0.8, 0.9].map((from, k) => ({ zone: k + 1, from: Math.round(from * maxHr), to: Math.round((from + 0.1) * maxHr), time: 0 }));
  let below = 0;
  for (let i = 1; i < points.length; i++) {
    const hr = points[i][HR];
    const t0 = points[i - 1][TIME];
    const t1 = points[i][TIME];
    if (hr == null || t0 == null || t1 == null || t1 - t0 > MAX_MOVING_GAP_S) continue;
    const zone = zones.findLast((z) => hr >= z.from);
    if (zone) zone.time += t1 - t0;
    else below += t1 - t0;
  }
  return { zones, below };
}

// --- Effort

/**
 * "Effort kilometers" (Leistungskilometer, used by the Swiss Alpine Club): distance plus one
 * kilometer per 100 m of ascent and per 150 m of descent. Comparable across terrain.
 */
export function effortKm(distance: number, gain: number, loss: number) {
  return distance / 1000 + gain / 100 + loss / 150;
}

/** Fraction of sea-level oxygen pressure available at `ele` meters (standard atmosphere). */
export function oxygenFraction(ele: number) {
  return (1 - 2.25577e-5 * ele) ** 5.25588;
}

/** DIN 33466 walking time for a stretch, in seconds. */
export function dinSeconds(distance: number, gain: number, loss: number) {
  const horizontal = distance / 1000 / 4;
  const vertical = gain / 300 + loss / 500;
  return (Math.max(horizontal, vertical) + Math.min(horizontal, vertical) / 2) * 3600;
}

/**
 * DIN time from the start to every point, summed over legs of ~250 m (the formula is meant for
 * legs with a consistent character, not a whole mixed route).
 */
export function cumulativeDin(points: RoutePoint[]) {
  const result = new Array<number>(points.length).fill(0);
  let legStart = 0;
  let acc = 0;
  let gain = 0, loss = 0;
  let ref: number | null = null;
  for (let i = 1; i < points.length; i++) {
    // Same 3 m hysteresis as the totals, so GPS noise doesn't add up.
    const e = points[i][ELE];
    if (e != null) {
      if (ref == null) ref = e;
      else if (e - ref >= 3) (gain += e - ref), (ref = e);
      else if (ref - e >= 3) (loss += ref - e), (ref = e);
    }
    const legDistance = points[i][DIST] - points[legStart][DIST];
    const leg = dinSeconds(legDistance, gain, loss);
    result[i] = acc + leg;
    if (legDistance >= 250) {
      acc += leg;
      legStart = i;
      gain = loss = 0;
    }
  }
  return result;
}

/** How your moving time compares with DIN 33466 (0.8 = 20 % faster than the norm). */
export function paceFactor(route: RouteData) {
  if (route.stats.movingTime == null || !route.hasElevation) return null;
  const din = cumulativeDin(route.points);
  const expected = din[din.length - 1];
  return expected > 0 ? route.stats.movingTime / expected : null;
}

// --- Sun

/**
 * Sunrise and sunset (unix seconds) on the UTC day containing `unixSeconds` at a location,
 * following NOAA's simplified solar position equations. null when the sun never rises or sets.
 */
export function sunTimes(unixSeconds: number, lat: number, lon: number) {
  const rad = Math.PI / 180;
  const day = Math.floor(unixSeconds / 86400 + 0.5 + lon / 360); // local solar day
  const n = day - 10957; // days since 2000-01-01 12:00 UTC (≈ J2000)
  const meanAnomaly = (357.5291 + 0.98560028 * (n - lon / 360)) % 360;
  const center = 1.9148 * Math.sin(meanAnomaly * rad) + 0.02 * Math.sin(2 * meanAnomaly * rad) + 0.0003 * Math.sin(3 * meanAnomaly * rad);
  const eclipticLon = (meanAnomaly + center + 180 + 102.9372) % 360;
  const transit = 2451545 + (n - lon / 360) + 0.0053 * Math.sin(meanAnomaly * rad) - 0.0069 * Math.sin(2 * eclipticLon * rad);
  const declination = Math.asin(Math.sin(eclipticLon * rad) * Math.sin(23.44 * rad));
  const cosHour = (Math.sin(-0.833 * rad) - Math.sin(lat * rad) * Math.sin(declination)) / (Math.cos(lat * rad) * Math.cos(declination));
  if (cosHour < -1 || cosHour > 1) return null;
  const hour = Math.acos(cosHour) / rad / 360;
  const toUnix = (julian: number) => Math.round((julian - 2440587.5) * 86400);
  return { sunrise: toUnix(transit - hour), sunset: toUnix(transit + hour), noon: toUnix(transit) };
}
