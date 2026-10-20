// Number/time formatting for routes, shared by the server-rendered stats and the map script.

export function formatDistance(meters: number, locale: string, decimals = 1) {
  if (meters < 1000) return `${Math.round(meters).toLocaleString(locale)} m`;
  return `${(meters / 1000).toLocaleString(locale, { minimumFractionDigits: decimals, maximumFractionDigits: decimals })} km`;
}

export function formatElevation(meters: number, locale: string) {
  return `${Math.round(meters).toLocaleString(locale)} m`;
}

export function formatDuration(seconds: number) {
  const minutes = Math.round(seconds / 60);
  const h = Math.floor(minutes / 60);
  const m = minutes % 60;
  if (h === 0) return `${m} min`;
  return m === 0 ? `${h} h` : `${h} h ${m.toString().padStart(2, "0")} min`;
}

export function formatPercent(fraction: number, locale: string) {
  return `${Math.round(fraction * 100).toLocaleString(locale)} %`;
}

export function formatClock(unixSeconds: number, locale: string, timeZone: string) {
  return new Intl.DateTimeFormat(locale, { hour: "2-digit", minute: "2-digit", hourCycle: "h23", timeZone }).format(unixSeconds * 1000);
}

export function formatDay(unixSeconds: number, locale: string, timeZone: string) {
  return new Intl.DateTimeFormat(locale, { weekday: "short", day: "numeric", month: "short", timeZone }).format(unixSeconds * 1000);
}

export function formatDateRange(fromSeconds: number, toSeconds: number, locale: string, timeZone: string) {
  const format = new Intl.DateTimeFormat(locale, { day: "numeric", month: "short", year: "numeric", timeZone });
  return format.formatRange(fromSeconds * 1000, toSeconds * 1000);
}

/** Minutes per kilometer, e.g. "12:30 min/km" */
export function formatPace(secondsPerKm: number) {
  const total = Math.round(secondsPerKm);
  return `${Math.floor(total / 60)}:${String(total % 60).padStart(2, "0")} min/km`;
}

export function formatSpeed(metersPerSecond: number, locale: string) {
  return `${(metersPerSecond * 3.6).toLocaleString(locale, { maximumFractionDigits: 1 })} km/h`;
}

export function formatCoords(lat: number, lon: number) {
  return `${lat.toFixed(5)}, ${lon.toFixed(5)}`;
}

// Free 3D views of any point, with a pin on it: the camera looks at it from ~2.5 km away,
// tilted towards the horizon.

/** Google Earth on the web, in 3D; searching the coordinates drops a pin on them */
export function googleEarthUrl(lat: number, lon: number, ele: number | null = null) {
  const at = `${lat.toFixed(6)},${lon.toFixed(6)}`;
  return `https://earth.google.com/web/search/${at}/@${at},${Math.round(ele ?? 0)}a,2500d,35y,0h,65t,0r`;
}

/** Google Maps with the satellite (globe) view, tilted, and the point as a dropped pin */
export function googleMaps3dUrl(lat: number, lon: number, ele: number | null = null) {
  const at = `${lat.toFixed(6)},${lon.toFixed(6)}`;
  return `https://www.google.com/maps/place/${at}/@${at},${Math.round((ele ?? 0) + 1800)}a,35y,0h,65t/data=!3m1!1e3`;
}

/** Unix seconds of a wall-clock time in `timezone`. */
export function fromLocal(year: number, month: number, day: number, hour: number, minute: number, second: number, timezone: string) {
  const asUtc = Date.UTC(year, month - 1, day, hour, minute, second) / 1000;
  let guess = asUtc;
  // The zone's offset at the guess, applied twice to settle across DST changes.
  for (let k = 0; k < 2; k++) {
    const p = localParts(guess, timezone);
    const shown = Date.UTC(p.year, p.month - 1, p.day, p.hour, p.minute, p.second) / 1000;
    guess += asUtc - shown;
  }
  return guess;
}

/** Calendar and clock parts of a moment in `timezone`. */
export function localParts(unix: number, timezone: string) {
  const parts = new Intl.DateTimeFormat("en-US", {
    timeZone: timezone,
    hourCycle: "h23",
    year: "numeric",
    month: "numeric",
    day: "numeric",
    hour: "numeric",
    minute: "numeric",
    second: "numeric",
  }).formatToParts(unix * 1000);
  const get = (type: string) => Number(parts.find((x) => x.type === type)?.value);
  return { year: get("year"), month: get("month"), day: get("day"), hour: get("hour"), minute: get("minute"), second: get("second") };
}
