// Places a post's photos on its GPS route, for the photo layer of the route map.
// Where each photo was taken comes from, in order of preference:
//   1. a fragment written after its path in the Markdown (see src/plugins/remark-media.mjs):
//      #km=5.8 · #time=12:41 · #time=2026-06-13T12:41 · #-33.75,-70.10 · #off
//   2. its EXIF GPS position, saved by `npm run photos` in src/data/photo-locations.json
//   3. its EXIF capture time, matched against the route's timestamps (cameras without GPS)

import savedLocations from "../data/photo-locations.json";
import type { RouteData } from "./gpx-core";
import { fromLocal, localParts } from "./route-format";

/** As collected by remark-media into the entry's frontmatter */
export interface PostPhoto {
  path: string;
  src: string;
  alt: string;
  caption: string;
  geo: string | null;
}

export interface RoutePhoto {
  src: string;
  alt: string;
  caption: string;
  lat: number;
  lon: number;
  /** Distance along the route (m) when the photo was taken on or near it */
  distance: number | null;
  /** Unix seconds, when known */
  time: number | null;
}

interface SavedLocation {
  lat?: number;
  lon?: number;
  ele?: number;
  time?: string;
}

const LAT = 0, LON = 1, DIST = 3, TIME = 4;
// A photo farther than this from the route is still on the map, but not on the profile.
const MAX_SNAP_M = 300;

export function placePhotos(photos: PostPhoto[] = [], route: RouteData, timezone: string): RoutePhoto[] {
  const saved = savedLocations as Record<string, SavedLocation>;
  return photos.flatMap((photo) => {
    const at = locate(photo.geo, saved[photo.path], route, timezone);
    return at ? [{ src: photo.src, alt: photo.alt, caption: photo.caption, ...at }] : [];
  });
}

function locate(geo: string | null, saved: SavedLocation | undefined, route: RouteData, timezone: string) {
  const fragment = (geo ?? "").trim();
  if (fragment === "off") return null;

  const km = fragment.match(/^km=([\d.]+)$/);
  if (km) return atDistance(route, Number(km[1]) * 1000);

  const time = fragment.match(/^time=(.+)$/);
  if (time) return atTime(route, parseTime(time[1], route, timezone));

  const coords = fragment.match(/^(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)$/);
  if (coords) return atPosition(route, Number(coords[1]), Number(coords[2]));

  if (saved?.lat != null && saved.lon != null) {
    const placed = atPosition(route, saved.lat, saved.lon);
    return { ...placed, time: saved.time ? parseTime(saved.time, route, timezone) : null };
  }
  if (saved?.time) return atTime(route, parseTime(saved.time, route, timezone));
  return null;
}

function atDistance(route: RouteData, distance: number) {
  const points = route.points;
  const i = Math.max(0, points.findIndex((p) => p[DIST] >= distance));
  const p = points[distance >= points[points.length - 1][DIST] ? points.length - 1 : i];
  return { lat: p[LAT], lon: p[LON], distance: p[DIST], time: route.startTime != null && p[TIME] != null ? route.startTime + p[TIME] : null };
}

function atTime(route: RouteData, unix: number | null) {
  if (unix == null || route.startTime == null) return null;
  const points = route.points;
  const t = unix - route.startTime;
  // Ten minutes of slack at either end for camera clocks that are a bit off.
  if (t < -600 || t > (points[points.length - 1][TIME] ?? -Infinity) + 600) return null;
  let best = 0;
  for (let i = 1; i < points.length; i++) if (Math.abs((points[i][TIME] ?? Infinity) - t) < Math.abs((points[best][TIME] ?? Infinity) - t)) best = i;
  const p = points[best];
  return { lat: p[LAT], lon: p[LON], distance: p[DIST], time: unix };
}

function atPosition(route: RouteData, lat: number, lon: number) {
  let best = 0;
  let bestDistance = Infinity;
  route.points.forEach((p, i) => {
    const d = metersBetween(lat, lon, p[LAT], p[LON]);
    if (d < bestDistance) (bestDistance = d), (best = i);
  });
  return { lat, lon, distance: bestDistance <= MAX_SNAP_M ? route.points[best][DIST] : null, time: null as number | null };
}

/**
 * "2026-06-13T12:41[:SS][±hh:mm|Z]" or a bare "12:41". Without an offset the time is local to
 * the post's timezone; a bare clock time is the first moment on the route showing it.
 */
function parseTime(text: string, route: RouteData, timezone: string): number | null {
  const value = text.trim();
  const clock = value.match(/^(\d{1,2}):(\d\d)$/);
  if (clock) {
    if (route.startTime == null) return null;
    const end = route.startTime + (route.points[route.points.length - 1][TIME] ?? 0);
    const firstDay = localParts(route.startTime, timezone);
    for (let day = 0; day < 30; day++) {
      const candidate = fromLocal(firstDay.year, firstDay.month, firstDay.day + day, Number(clock[1]), Number(clock[2]), 0, timezone);
      if (candidate > end) return null;
      if (candidate >= route.startTime) return candidate;
    }
    return null;
  }
  const full = value.match(/^(\d{4})-(\d\d)-(\d\d)[T ](\d\d):(\d\d)(?::(\d\d)(?:\.\d+)?)?(Z|[+-]\d\d:?\d\d)?$/);
  if (!full) return null;
  const [, y, mo, d, h, mi, s = "0", offset] = full;
  if (offset) return Math.round(Date.parse(`${y}-${mo}-${d}T${h}:${mi}:${s.padStart(2, "0")}${offset.length === 5 ? `${offset.slice(0, 3)}:${offset.slice(3)}` : offset}`) / 1000);
  return fromLocal(Number(y), Number(mo), Number(d), Number(h), Number(mi), Number(s), timezone);
}

function metersBetween(lat1: number, lon1: number, lat2: number, lon2: number) {
  const r = Math.PI / 180;
  const h = Math.sin(((lat2 - lat1) * r) / 2) ** 2 + Math.cos(lat1 * r) * Math.cos(lat2 * r) * Math.sin(((lon2 - lon1) * r) / 2) ** 2;
  return 2 * 6371008.8 * Math.asin(Math.sqrt(h));
}
