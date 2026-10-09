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
