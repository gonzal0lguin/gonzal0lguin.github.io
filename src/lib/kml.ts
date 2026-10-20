// KML of a route, to open it whole in Google Earth (web, desktop or phone): the track draped on
// the terrain (one line per day), start/finish, summit, camps, marked points and photos. Used by
// the posts (src/pages/routes/[slug].kml.ts) and the route tool (in the browser).

import { highPointRole, type RouteData } from "./gpx-core";
import { formatElevation } from "./route-format";
import { WAYPOINT_KINDS, type WaypointKind } from "./waypoint-kinds";
import { languages, useTranslations, type Lang } from "../i18n/ui";

export interface KmlOptions {
  lang: Lang;
  title?: string | null;
  /** Photos with absolute URLs, so Google Earth can load them */
  photos?: { src: string; alt: string; caption: string; lat: number; lon: number }[];
}

const LAT = 0, LON = 1, DIST = 3;

// Google Earth's own icons, white so the style's color tints them.
const ICON_BASE = "https://maps.google.com/mapfiles/kml/shapes/";
const KIND_ICONS: Record<WaypointKind | "start" | "finish" | "photo" | "highPoint", string> = {
  camp: "campground",
  summit: "mountains",
  highPoint: "triangle",
  view: "binoculars",
  water: "water",
  hut: "lodging",
  crux: "hiker",
  danger: "caution",
  parking: "parking_lot",
  note: "info-i",
  start: "flag",
  finish: "flag",
  photo: "camera",
};
const ICON_COLORS: Partial<Record<keyof typeof KIND_ICONS, string>> = { start: "#16a34a", finish: "#111827", highPoint: "#78716c", photo: "#ffffff" };
const DAY_COLORS = ["#ef4444", "#2563eb", "#16a34a", "#9333ea", "#ea580c", "#0891b2"];

const esc = (text: string) => text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

/** KML colors are aabbggrr */
const kmlColor = (hex: string, alpha = "ff") => `${alpha}${hex.slice(5, 7)}${hex.slice(3, 5)}${hex.slice(1, 3)}`;

export function toKml(route: RouteData, { lang, title, photos = [] }: KmlOptions): string {
  const t = useTranslations(lang);
  const locale = languages[lang].locale;
  const { points } = route;
  const name = title || route.name || t("tools.route.defaultName");

  // One line per day (the whole route on single-day trips).
  const ranges = route.days.length ? route.days.map((d) => [d.fromDistance, d.toDistance] as const) : [[0, Infinity] as const];
  const lines = ranges.map(([from, to], k) => {
    const coords = points.filter((p) => p[DIST] >= from && p[DIST] <= to).map((p) => `${p[LON]},${p[LAT]}`);
    return `
    <Placemark>
      <name>${esc(ranges.length > 1 ? t("route.day", { n: k + 1 }) : name)}</name>
      <styleUrl>#line-${k % DAY_COLORS.length}</styleUrl>
      <LineString><tessellate>1</tessellate><altitudeMode>clampToGround</altitudeMode><coordinates>${coords.join(" ")}</coordinates></LineString>
    </Placemark>`;
  });

  const placemark = (kind: keyof typeof KIND_ICONS, label: string, lat: number, lon: number, description = "") => `
    <Placemark>
      <name>${esc(label)}</name>${description ? `\n      <description><![CDATA[${description}]]></description>` : ""}
      <styleUrl>#${kind}</styleUrl>
      <Point><coordinates>${lon},${lat}</coordinates></Point>
    </Placemark>`;
  const elevation = (ele: number | null | undefined) => (ele != null ? formatElevation(ele, locale) : "");

  const first = points[0];
  const last = points[points.length - 1];
  const oneWay = route.stats.shape === "one-way";
  const role = highPointRole(route.summit, route.waypoints);
  const marks = [
    placemark("start", oneWay ? t("route.start") : t("route.startFinish"), first[LAT], first[LON]),
    ...(oneWay ? [placemark("finish", t("route.finish"), last[LAT], last[LON])] : []),
    ...(route.summit && role
      ? [placemark(role, `${t(role === "summit" ? "route.summit" : "route.highPoint")} · ${elevation(route.summit.ele)}`, route.summit.lat, route.summit.lon)]
      : []),
    ...route.camps.map((c) => placemark("camp", t("route.camp"), c.lat, c.lon, elevation(c.ele))),
    ...route.waypoints.map((w) =>
      placemark(w.kind, w.name, w.lat, w.lon, [`<b>${esc(t(`route.kind.${w.kind}`))}</b>`, elevation(w.ele), w.desc && esc(w.desc).replace(/\n/g, "<br>")].filter(Boolean).join("<br>")),
    ),
  ];
  const photoMarks = photos.map((p) =>
    placemark("photo", p.caption || p.alt || t("route.photos"), p.lat, p.lon, `<img src="${esc(p.src)}" width="360"><br>${esc(p.caption || p.alt)}`),
  );

  const styles = [
    ...DAY_COLORS.map((c, k) => `<Style id="line-${k}"><LineStyle><color>${kmlColor(c)}</color><width>4</width></LineStyle></Style>`),
    ...Object.entries(KIND_ICONS).map(([kind, icon]) => {
      const color = ICON_COLORS[kind as keyof typeof KIND_ICONS] ?? WAYPOINT_KINDS[kind as WaypointKind].color;
      return `<Style id="${kind}"><IconStyle><color>${kmlColor(color)}</color><scale>1.1</scale><Icon><href>${ICON_BASE}${icon}.png</href></Icon></IconStyle></Style>`;
    }),
  ];

  // Open looking at the whole route, from the south and tilted.
  const lats = points.map((p) => p[LAT]);
  const lons = points.map((p) => p[LON]);
  const [south, north, west, east] = [Math.min(...lats), Math.max(...lats), Math.min(...lons), Math.max(...lons)];
  const midLat = (south + north) / 2;
  const span = Math.max((north - south) * 111_000, (east - west) * 111_000 * Math.cos((midLat * Math.PI) / 180));
  const lookAt = `<LookAt><longitude>${(west + east) / 2}</longitude><latitude>${midLat}</latitude><altitude>0</altitude><heading>0</heading><tilt>55</tilt><range>${Math.round(span * 1.8 + 1500)}</range><altitudeMode>relativeToGround</altitudeMode></LookAt>`;

  return `<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
<Document>
  <name>${esc(name)}</name>
  ${lookAt}
  ${styles.join("\n  ")}
  <Folder><name>${esc(t("route.kml.track"))}</name><open>1</open>${lines.join("")}
  </Folder>
  <Folder><name>${esc(t("route.kml.points"))}</name><open>1</open>${marks.join("")}
  </Folder>${photoMarks.length ? `\n  <Folder><name>${esc(t("route.photos"))}</name>${photoMarks.join("")}\n  </Folder>` : ""}
</Document>
</kml>
`;
}
