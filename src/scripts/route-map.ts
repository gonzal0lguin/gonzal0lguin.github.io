// Browser side of the route card (src/lib/route-card.ts): Leaflet map + SVG elevation profile,
// kept in sync. Hovering either one moves a cursor on both; "replay" animates the cursor along
// the route; dragging on the profile selects a section. On top of that: marked points (editable
// in the route tool), the post's photos, a 3D terrain view (src/scripts/route-3d.ts) and the
// analysis panel (src/scripts/route-panels.ts).

import L from "leaflet";
import "leaflet/dist/leaflet.css";
import "../styles/route-map.css";
import type { RouteData, RoutePoint, Waypoint } from "../lib/gpx-core";
import type { RoutePhoto } from "../lib/photos";
import { sectionStats, speedSeries, verticalSpeedSeries } from "../lib/route-analysis";
import {
  formatClock,
  formatCoords,
  formatDistance,
  formatDuration,
  formatElevation,
  formatPercent,
  formatSpeed,
  googleEarthUrl,
  googleMaps3dUrl,
} from "../lib/route-format";
import { WAYPOINT_KIND_IDS, WAYPOINT_KINDS, type WaypointKind } from "../lib/waypoint-kinds";
import { useTranslations, type Lang } from "../i18n/ui";
import type { View3D } from "./route-3d";

export type Mode = "grade" | "elevation" | "speed" | "vspeed" | "hr";
export type Payload = RouteData & { lang: Lang; locale: string; timezone: string; photos: RoutePhoto[]; editable: boolean };
type Stops = [number, string][];

const LAT = 0, LON = 1, ELE = 2, DIST = 3, TIME = 4, HR = 5, GRADE = 6;
const REPLAY_MS = 18000;
const PHOTOS_KEY = "route-map:photos";

// Color scales over 0..1 (grade: absolute fraction, 0.5 = 50 %). Vertical speed is diverging:
// descending blue, level gray, climbing red.
const SCALES: Record<Mode, Stops> = {
  grade: [[0, "#1a9850"], [0.08, "#91cf60"], [0.15, "#fee08b"], [0.25, "#fc8d59"], [0.35, "#d73027"], [0.5, "#7b1fa2"]],
  elevation: [[0, "#440154"], [0.25, "#3b528b"], [0.5, "#21918c"], [0.75, "#5ec962"], [1, "#fde725"]],
  speed: [[0, "#0d0887"], [0.25, "#7e03a8"], [0.5, "#cc4778"], [0.75, "#f89540"], [1, "#f0f921"]],
  vspeed: [[0, "#2166ac"], [0.3, "#92c5de"], [0.5, "#bdbdbd"], [0.7, "#f4a582"], [1, "#b2182b"]],
  hr: [[0, "#2563eb"], [0.35, "#22c55e"], [0.6, "#facc15"], [0.8, "#f97316"], [1, "#dc2626"]],
};
const VSPEED_RANGE = 1000; // m/h at either end of the vertical speed scale

const TILES = {
  topo: {
    url: "https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png",
    options: { maxZoom: 17, attribution: 'Map data © <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>, SRTM · Style © <a href="https://opentopomap.org">OpenTopoMap</a> (CC-BY-SA)' },
  },
  satellite: {
    url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    options: { maxZoom: 18, attribution: "Imagery © Esri, Maxar, Earthstar Geographics" },
  },
  streets: {
    url: "https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    options: { maxZoom: 19, attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors' },
  },
};

const ICONS = {
  flag: '<path d="M5 21V4m0 0h11l-2 4 2 4H5" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linejoin="round"/>',
  finish: '<path d="M5 21V4h14v9H5" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linejoin="round"/><path d="M5 4h4.7v4.5H5zm4.7 4.5h4.6V13H9.7zm4.6-4.5H19v4.5h-4.7z" fill="currentColor"/>',
  camera: '<path d="M4 8h3l2-3h6l2 3h3v11H4z" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linejoin="round"/><circle cx="12" cy="13" r="3.5" fill="none" stroke="currentColor" stroke-width="2.2"/>',
};

const cleanups = new Map<HTMLElement, () => void>();
document.addEventListener("astro:before-swap", () => {
  cleanups.forEach((cleanup) => cleanup());
  cleanups.clear();
});

export function initRouteMaps() {
  document.querySelectorAll<HTMLElement>("[data-route-map]:not([data-ready])").forEach((root) => {
    root.dataset.ready = "";
    cleanups.set(root, createRouteMap(root));
  });
}

/** Tear down the maps inside `container` before its content is replaced. */
export function destroyRouteMaps(container: Element) {
  cleanups.forEach((cleanup, root) => {
    if (!container.contains(root)) return;
    cleanup();
    cleanups.delete(root);
  });
}

/** What the analysis panel can ask of the map. */
export interface RouteMapApi {
  data: Payload;
  t: ReturnType<typeof useTranslations>;
  waypoints: () => Waypoint[];
  /** Route point nearest to a waypoint, if it is on (or near) the route */
  waypointIndex: (w: { lat: number; lon: number }) => number | null;
  setCursor: (i: number) => void;
  hideCursor: () => void;
  select: (from: number, to: number, zoom?: boolean) => void;
  highlight: (from: number, to: number) => void;
  clearHighlight: () => void;
  focusWaypoint: (index: number) => void;
  focusPoint: (i: number) => void;
  addWaypointAt: (i: number, kind?: WaypointKind, name?: string) => void;
  editWaypoint: (index: number) => void;
}

function createRouteMap(root: HTMLElement) {
  const data: Payload = JSON.parse(root.querySelector("[data-route-data]")!.textContent!);
  const { points, locale, timezone, editable } = data;
  const t = useTranslations(data.lang);
  const total = points[points.length - 1][DIST];
  const $ = <T extends Element = HTMLElement>(selector: string) => root.querySelector<T>(selector);
  const cleanupFns: (() => void)[] = [];

  let waypoints: Waypoint[] = data.waypoints.map((w) => ({ ...w }));

  const elevations = points.map((p) => p[ELE]).filter((e): e is number => e != null);
  const heartRates = points.map((p) => p[HR]).filter((h): h is number => h != null);
  const speeds = speedSeries(points);
  const vspeeds = verticalSpeedSeries(points);
  const sortedSpeeds = speeds.filter((s): s is number => s != null).sort((a, b) => a - b);
  const range = {
    ele: [Math.min(...elevations), Math.max(...elevations)] as const,
    hr: heartRates.length ? ([Math.min(...heartRates), Math.max(...heartRates)] as const) : ([0, 1] as const),
    // Top of the speed scale at the 95th percentile, so a few fast bits don't wash out the rest.
    speed: [0, Math.max(3, sortedSpeeds[Math.floor(sortedSpeeds.length * 0.95)] ?? 5)] as const,
  };

  let mode: Mode = "grade";
  let cursorIndex: number | null = null;
  let selection: [number, number] | null = null;
  let highlighted: [number, number] | null = null;
  let marking = false;
  let view3d: View3D | null = null;
  let is3d = false;
  let panels: { refresh: () => void } | null = null;

  // --- Color helpers

  const rawValue = (i: number, m: Mode): number | null => {
    const p = points[i];
    if (m === "grade") return p[GRADE] == null ? null : Math.min(Math.abs(p[GRADE]!) / 100, 0.5);
    if (m === "elevation") return p[ELE] == null ? null : normalize(p[ELE]!, range.ele);
    if (m === "speed") return speeds[i] == null ? null : Math.min(1, normalize(speeds[i]!, range.speed));
    if (m === "vspeed") return vspeeds[i] == null ? null : Math.max(0, Math.min(1, vspeeds[i]! / (2 * VSPEED_RANGE) + 0.5));
    return p[HR] == null ? null : normalize(p[HR]!, range.hr);
  };
  // Grade and heart rate are noisy point to point; average them over neighbours so colors read as zones.
  const valuesCache = new Map<Mode, (number | null)[]>();
  const values = (m: Mode) => {
    if (!valuesCache.has(m)) {
      const raw = points.map((_, i) => rawValue(i, m));
      valuesCache.set(m, m === "grade" ? smooth(raw, 8) : m === "hr" ? smooth(raw, 6) : raw);
    }
    return valuesCache.get(m)!;
  };
  const colorAtIndex = (i: number, m: Mode) => {
    const v = values(m)[i];
    return v == null ? "#9ca3af" : colorAt(SCALES[m], v);
  };
  const colors = () => points.map((_, i) => colorAtIndex(i, mode));

  // --- Formatting

  const timeAt = (i: number) => (data.startTime != null && points[i][TIME] != null ? data.startTime + points[i][TIME]! : null);
  const clockAt = (i: number) => {
    const at = timeAt(i);
    if (at == null) return null;
    const day = data.days.findIndex((d) => at >= d.start && at <= d.end);
    return `${day >= 0 ? `${t("route.day", { n: day + 1 })} ` : ""}${formatClock(at, locale, timezone)}`;
  };
  const kmLabel = (i: number) => `km ${(points[i][DIST] / 1000).toLocaleString(locale, { maximumFractionDigits: 2 })}`;

  // --- Map

  const mapEl = $("[data-map]")!;
  const map = L.map(mapEl, {
    scrollWheelZoom: false,
    dragging: !L.Browser.mobile,
    preferCanvas: true,
    zoomSnap: 0.25,
  });
  const baseLayers = {
    [t("route.layer.topo")]: L.tileLayer(TILES.topo.url, TILES.topo.options),
    [t("route.layer.satellite")]: L.tileLayer(TILES.satellite.url, TILES.satellite.options),
    [t("route.layer.streets")]: L.tileLayer(TILES.streets.url, TILES.streets.options),
  };
  baseLayers[t("route.layer.topo")].addTo(map);
  L.control.layers(baseLayers, undefined, { position: "topright" }).addTo(map);
  L.control.scale({ imperial: false, position: "bottomright" }).addTo(map);
  map.createPane("selection").style.zIndex = "390";

  // Only capture the scroll wheel (and one-finger drags on phones) after the visitor clicks the map.
  const activate = () => {
    map.scrollWheelZoom.enable();
    map.dragging.enable();
  };
  mapEl.addEventListener("mouseleave", () => map.scrollWheelZoom.disable());
  // Focusing a field in a popup that crosses the map's edge can scroll the map or the card sideways
  // (browsers without overflow: clip); Leaflet never expects that, so undo it.
  for (const box of [mapEl, root]) {
    box.addEventListener("scroll", () => {
      box.scrollTop = 0;
      box.scrollLeft = 0;
    });
  }

  const latlngs = points.map((p) => L.latLng(p[LAT], p[LON]));
  const bounds = L.latLngBounds(latlngs);
  // Extra room at the top and bottom for the overlaid controls, legend and readout.
  const fitPadding: L.FitBoundsOptions = { paddingTopLeft: [40, 64], paddingBottomRight: [40, 80] };
  const fit = () => {
    if (is3d) view3d?.fit();
    else map.fitBounds(bounds, fitPadding);
  };
  map.fitBounds(bounds, fitPadding);

  L.polyline(latlngs, { color: "#111827", weight: 8, opacity: 0.5, interactive: false }).addTo(map);
  const selectionLine = L.polyline([], { pane: "selection", color: "#ffffff", weight: 13, opacity: 0.85, lineCap: "round", interactive: false }).addTo(map);
  const highlightLine = L.polyline([], { pane: "selection", color: "#ffffff", weight: 11, opacity: 0.55, lineCap: "round", interactive: false }).addTo(map);
  const routeLayer = L.layerGroup().addTo(map);

  const drawRoute = () => {
    routeLayer.clearLayers();
    // Merge consecutive segments of (nearly) the same color into one polyline.
    let run: L.LatLng[] = [latlngs[0]];
    let runColor = colorAtIndex(0, mode);
    for (let i = 1; i < points.length; i++) {
      const color = colorAtIndex(i, mode);
      run.push(latlngs[i]);
      if (color !== runColor || i === points.length - 1) {
        L.polyline(run, { color: runColor, weight: 4.5, opacity: 1, lineCap: "round", interactive: false }).addTo(routeLayer);
        run = [latlngs[i]];
        runColor = color;
      }
    }
  };
  drawRoute();

  // --- Popups: any point can be opened in Google Earth / Google Maps 3D

  const pin = (icon: string, color: string, size = 26) =>
    L.divIcon({
      className: "",
      html: `<div class="route-pin" style="width:${size}px;height:${size}px;background:${color}"><svg viewBox="0 0 24 24" width="${size * 0.55}" height="${size * 0.55}">${icon}</svg></div>`,
      iconSize: [size, size],
      iconAnchor: [size / 2, size / 2],
      popupAnchor: [0, -size / 2],
    });

  const pointDetails = (lat: number, lon: number, ele: number | null | undefined, i: number | null, withKm = true) => {
    const facts = [i != null && withKm && kmLabel(i), ele != null && formatElevation(ele, locale), i != null && clockAt(i)].filter(Boolean);
    return `
      ${facts.length ? `<p class="route-popup__facts">${esc(facts.join(" · "))}</p>` : ""}
      <p class="route-popup__coords">${formatCoords(lat, lon)}</p>
      <div class="route-popup__links">
        <a href="${googleEarthUrl(lat, lon, ele)}" target="_blank" rel="noopener">${esc(t("route.openEarth"))} ↗</a>
        <a href="${googleMaps3dUrl(lat, lon, ele)}" target="_blank" rel="noopener">${esc(t("route.openMaps"))} ↗</a>
        <button type="button" data-action="copy" data-coords="${formatCoords(lat, lon)}">${esc(t("route.copyCoords"))}</button>
      </div>`;
  };

  // Pan popups clear of the toolbar on top and the legend and scale at the bottom, which sit above the map.
  const popupOptions: L.PopupOptions = { className: "route-popup", maxWidth: 280, minWidth: 200, autoPanPaddingTopLeft: [10, 60], autoPanPaddingBottomRight: [60, 104] };

  const nearestIndex = (lat: number, lon: number) => {
    let best = 0;
    let bestDistance = Infinity;
    const target = L.latLng(lat, lon);
    for (let i = 0; i < latlngs.length; i++) {
      const d = target.distanceTo(latlngs[i]);
      if (d < bestDistance) (bestDistance = d), (best = i);
    }
    return { i: best, meters: bestDistance };
  };
  const waypointIndex = (w: { lat: number; lon: number }) => {
    const { i, meters } = nearestIndex(w.lat, w.lon);
    return meters <= 150 ? i : null;
  };

  // Start/finish, summit, camps.
  const first = points[0];
  const last = points[points.length - 1];
  const returnsToStart = data.stats.shape !== "one-way";
  const fixedMarker = (latlng: L.LatLngExpression, icon: L.DivIcon, title: string, i: number, zIndexOffset: number) =>
    L.marker(latlng, { icon, zIndexOffset })
      .bindTooltip(title, { direction: "top", offset: [0, -14], className: "route-label" })
      .bindPopup(() => `<p class="route-popup__title">${esc(title)}</p>${pointDetails(points[i][LAT], points[i][LON], points[i][ELE], i)}`, popupOptions)
      .addTo(map);
  fixedMarker([first[LAT], first[LON]], pin(ICONS.flag, "#16a34a"), returnsToStart ? t("route.startFinish") : t("route.start"), 0, 500);
  if (!returnsToStart) fixedMarker([last[LAT], last[LON]], pin(ICONS.finish, "#111827"), t("route.finish"), points.length - 1, 500);
  if (data.summit) {
    const text = `${t("route.summit")} · ${formatElevation(data.summit.ele ?? 0, locale)}`;
    const marker = fixedMarker([data.summit.lat, data.summit.lon], pin(WAYPOINT_KINDS.summit.icon, WAYPOINT_KINDS.summit.color, 30), text, nearestIndex(data.summit.lat, data.summit.lon).i, 1000);
    // Always-visible label on wide maps; on phones it would cover the controls, so show it on tap.
    if (mapEl.clientWidth >= 560) marker.unbindTooltip().bindTooltip(text, { direction: "right", offset: [16, 0], className: "route-label", permanent: true });
  }
  data.camps.forEach((camp) => {
    const text = `${t("route.camp")}${camp.ele != null ? ` · ${formatElevation(camp.ele, locale)}` : ""}`;
    fixedMarker([camp.lat, camp.lon], pin(WAYPOINT_KINDS.camp.icon, WAYPOINT_KINDS.camp.color, 28), text, nearestIndex(camp.lat, camp.lon).i, 800);
  });
  const kmStep = total <= 15000 ? 1000 : total <= 40000 ? 5000 : 10000;
  // On out-and-back routes the way back retraces the way up, so only mark the way up.
  const kmUntil = data.stats.shape === "out-and-back" ? total / 2 : total;
  for (let km = kmStep, i = 0; km < kmUntil; km += kmStep) {
    while (i < points.length - 1 && points[i][DIST] < km) i++;
    const icon = L.divIcon({ className: "", html: `<div class="route-km" style="width:18px;height:18px">${km / 1000}</div>`, iconSize: [18, 18], iconAnchor: [9, 9] });
    L.marker(latlngs[i], { icon, interactive: false, keyboard: false }).addTo(map);
  }

  // --- Marked points (waypoints)

  const waypointLayer = L.layerGroup().addTo(map);
  let waypointMarkers: L.Marker[] = [];

  const kindLabel = (kind: WaypointKind) => t(`route.kind.${kind}`);

  const waypointPopup = (index: number) => {
    const w = waypoints[index];
    const i = waypointIndex(w);
    return `
      <p class="route-popup__kind" style="--kind:${WAYPOINT_KINDS[w.kind].color}">${esc(kindLabel(w.kind))}</p>
      <p class="route-popup__title">${esc(w.name)}</p>
      ${w.desc ? `<p class="route-popup__desc">${esc(w.desc)}</p>` : ""}
      ${pointDetails(w.lat, w.lon, w.ele ?? (i != null ? points[i][ELE] : null), i)}
      ${
        editable
          ? `<div class="route-popup__actions">
              <button type="button" class="btn btn-xs" data-action="edit-wp" data-index="${index}">${esc(t("route.edit"))}</button>
              <button type="button" class="btn btn-ghost btn-xs text-error" data-action="delete-wp" data-index="${index}">${esc(t("route.delete"))}</button>
            </div>`
          : ""
      }`;
  };

  const editorHtml = (index: number) => {
    const w = waypoints[index];
    return `
      <form class="route-editor" data-editor="${index}">
        <fieldset class="route-editor__kinds">
          <legend class="sr-only">${esc(t("route.kindLabel"))}</legend>
          ${WAYPOINT_KIND_IDS.map(
            (kind) => `
            <label title="${esc(kindLabel(kind))}">
              <input type="radio" name="kind" value="${kind}" ${kind === w.kind ? "checked" : ""} class="sr-only" data-kind-label="${esc(kindLabel(kind))}" />
              <span style="--kind:${WAYPOINT_KINDS[kind].color}"><svg viewBox="0 0 24 24" width="15" height="15">${WAYPOINT_KINDS[kind].icon}</svg></span>
              <span class="sr-only">${esc(kindLabel(kind))}</span>
            </label>`,
          ).join("")}
        </fieldset>
        <p class="route-editor__kind-name" data-kind-name>${esc(kindLabel(w.kind))}</p>
        <input name="name" class="input input-bordered input-sm w-full" maxlength="80" value="${esc(w.name)}" placeholder="${esc(t("route.name"))}" aria-label="${esc(t("route.name"))}" />
        <textarea name="desc" class="textarea textarea-bordered textarea-sm mt-2 w-full" rows="2" maxlength="500" placeholder="${esc(t("route.notePlaceholder"))}" aria-label="${esc(t("route.notePlaceholder"))}">${esc(w.desc ?? "")}</textarea>
        <div class="route-popup__actions">
          <button type="button" class="btn btn-ghost btn-xs text-error" data-action="delete-wp" data-index="${index}">${esc(t("route.delete"))}</button>
          <button type="submit" class="btn btn-primary btn-xs">${esc(t("route.save"))}</button>
        </div>
      </form>`;
  };

  const drawWaypoints = () => {
    waypointLayer.clearLayers();
    waypointMarkers = waypoints.map((w, index) => {
      const marker = L.marker([w.lat, w.lon], {
        icon: pin(WAYPOINT_KINDS[w.kind].icon, WAYPOINT_KINDS[w.kind].color, 26),
        zIndexOffset: 900,
        draggable: editable,
        autoPan: true,
      })
        .bindTooltip(w.name, { direction: "top", offset: [0, -14], className: "route-label" })
        .bindPopup(() => waypointPopup(index), popupOptions)
        .addTo(waypointLayer);
      marker.on("dragend", () => {
        const { lat, lng } = marker.getLatLng();
        const near = nearestIndex(lat, lng);
        // Dropped next to the route: stick to it, and take its elevation and time.
        const onRoute = map.latLngToContainerPoint(latlngs[near.i]).distanceTo(map.latLngToContainerPoint([lat, lng])) < 16;
        waypoints[index] = onRoute
          ? { ...w, lat: points[near.i][LAT], lon: points[near.i][LON], ele: points[near.i][ELE], time: timeAt(near.i) }
          : { ...w, lat, lon: lng, ele: near.meters < 60 ? points[near.i][ELE] : null, time: null };
        waypointsChanged();
      });
      return marker;
    });
    view3d?.setWaypoints(waypoints);
  };

  const waypointsChanged = () => {
    drawWaypoints();
    drawProfile();
    panels?.refresh();
    root.dispatchEvent(new CustomEvent("route:waypoints", { detail: waypoints.map((w) => ({ ...w })) }));
  };

  const addWaypoint = (w: Waypoint) => {
    waypoints.push({ ...w, name: w.name || `${kindLabel(w.kind)} ${waypoints.filter((x) => x.kind === w.kind).length + 1}` });
    waypointsChanged();
    editWaypoint(waypoints.length - 1);
  };
  const addWaypointAt = (i: number, kind: WaypointKind = "note", name?: string) => {
    const p = points[i];
    addWaypoint({ lat: p[LAT], lon: p[LON], ele: p[ELE], time: timeAt(i), kind, name: name ?? "" });
  };
  function editWaypoint(index: number) {
    if (is3d) toggle3d(false);
    const marker = waypointMarkers[index];
    if (!marker) return;
    marker.unbindPopup().bindPopup(editorHtml(index), { ...popupOptions, minWidth: 250 }).openPopup();
    marker.once("popupclose", () => marker.unbindPopup().bindPopup(() => waypointPopup(index), popupOptions));
    setTimeout(() => {
      const input = root.querySelector<HTMLInputElement>(`[data-editor="${index}"] input[name="name"]`);
      input?.focus({ preventScroll: true });
      input?.select();
    }, 50);
  }
  const focusWaypoint = (index: number) => {
    const w = waypoints[index];
    if (!w) return;
    if (is3d) return view3d?.flyTo(w.lat, w.lon);
    map.setView([w.lat, w.lon], Math.max(map.getZoom(), 14));
    waypointMarkers[index]?.openPopup();
  };

  drawWaypoints();

  // Popup buttons and the waypoint editor (event delegation: popups are created on demand).
  root.addEventListener("click", (event) => {
    const button = (event.target as HTMLElement).closest<HTMLElement>("[data-action]");
    if (!button) return;
    const index = Number(button.dataset.index);
    if (button.dataset.action === "copy") {
      navigator.clipboard?.writeText(button.dataset.coords ?? "").then(() => (button.textContent = t("route.copied")));
    } else if (button.dataset.action === "edit-wp") {
      map.closePopup();
      editWaypoint(index);
    } else if (button.dataset.action === "delete-wp") {
      map.closePopup();
      waypoints.splice(index, 1);
      waypointsChanged();
    } else if (button.dataset.action === "mark-here") {
      map.closePopup();
      addWaypointAt(Number(button.dataset.point));
    } else if (button.dataset.action === "photo-post") {
      const target = articleImages[Number(button.dataset.photo)]?.[0];
      target?.scrollIntoView({ behavior: "smooth", block: "center" });
      target?.closest("figure")?.animate([{ outline: "4px solid oklch(var(--p))" }, { outline: "4px solid transparent" }], { duration: 1800 });
    }
  });
  root.addEventListener("change", (event) => {
    const radio = event.target as HTMLInputElement;
    if (radio.name !== "kind") return;
    const name = radio.closest("form")?.querySelector("[data-kind-name]");
    if (name) name.textContent = radio.dataset.kindLabel ?? "";
  });
  root.addEventListener("submit", (event) => {
    const form = (event.target as HTMLElement).closest<HTMLFormElement>("[data-editor]");
    if (!form) return;
    event.preventDefault();
    const index = Number(form.dataset.editor);
    const fields = new FormData(form);
    const kind = (fields.get("kind") as WaypointKind) || waypoints[index].kind;
    const desc = String(fields.get("desc") ?? "").trim();
    waypoints[index] = { ...waypoints[index], kind, name: String(fields.get("name") ?? "").trim() || kindLabel(kind), desc: desc || undefined };
    map.closePopup();
    waypointsChanged();
  });

  // --- Photos

  const photoLayer = L.layerGroup();
  // The same photos in the post, to link both ways.
  const article = root.closest("article") ?? document;
  const articleImages = data.photos.map((photo) =>
    [...article.querySelectorAll<HTMLImageElement>("img")].filter((img) => img.getAttribute("src") === photo.src && !root.contains(img)),
  );
  const photoMarkers = data.photos.map((photo, k) => {
    const icon = L.divIcon({
      className: "",
      html: `<div class="route-photo" style="background-image:url('${encodeURI(photo.src)}')"></div>`,
      iconSize: [38, 38],
      iconAnchor: [19, 19],
      popupAnchor: [0, -18],
    });
    const caption = photo.caption || photo.alt;
    const facts = [
      photo.distance != null && `km ${(photo.distance / 1000).toLocaleString(locale, { maximumFractionDigits: 1 })}`,
      photo.time != null && formatClock(photo.time, locale, timezone),
    ].filter(Boolean);
    return L.marker([photo.lat, photo.lon], { icon, zIndexOffset: 700, title: caption })
      .bindPopup(
        () => `
          <div class="route-popup__photo" data-lightbox><img src="${esc(photo.src)}" alt="${esc(photo.alt)}" /></div>
          ${caption ? `<p class="route-popup__desc">${esc(caption)}</p>` : ""}
          ${facts.length ? `<p class="route-popup__facts">${esc(facts.join(" · "))}</p>` : ""}
          ${articleImages[k].length ? `<div class="route-popup__actions"><button type="button" class="btn btn-xs" data-action="photo-post" data-photo="${k}">${esc(t("route.photoInPost"))}</button></div>` : ""}`,
        { ...popupOptions, maxWidth: 260 },
      )
      .addTo(photoLayer);
  });

  const photosButton = $<HTMLButtonElement>("[data-photos]");
  let photosOn = data.photos.length > 0 && readStorage(PHOTOS_KEY) !== "off";
  const setPhotos = (on: boolean) => {
    photosOn = on && data.photos.length > 0;
    if (photosOn) photoLayer.addTo(map);
    else photoLayer.remove();
    photosButton?.setAttribute("aria-pressed", String(photosOn));
    photosButton?.classList.toggle("btn-active", photosOn);
    view3d?.setPhotos(photosOn ? data.photos : []);
    drawProfile();
  };
  photosButton?.addEventListener("click", () => {
    setPhotos(!photosOn);
    writeStorage(PHOTOS_KEY, photosOn ? "on" : "off");
  });

  // A small "on the map" button on every photo of the post that has a location.
  const photoPins: HTMLElement[] = [];
  articleImages.forEach((images, k) =>
    images.forEach((img) => {
      const figure = img.closest("figure");
      if (!figure) return;
      const button = document.createElement("button");
      button.type = "button";
      button.className = "route-photo-pin";
      button.title = t("route.photoOnMap");
      button.innerHTML = `<svg viewBox="0 0 24 24" width="14" height="14" aria-hidden="true"><path d="M12 21s-6-5.6-6-11a6 6 0 0 1 12 0c0 5.4-6 11-6 11z" fill="currentColor"/><circle cx="12" cy="10" r="2.2" fill="#fff"/></svg><span>${esc(t("route.photoOnMap"))}</span>`;
      button.addEventListener("click", (event) => {
        event.stopPropagation();
        if (!photosOn) setPhotos(true);
        if (is3d) toggle3d(false);
        root.scrollIntoView({ behavior: "smooth", block: "start" });
        const photo = data.photos[k];
        map.setView([photo.lat, photo.lon], Math.max(map.getZoom(), 14));
        setTimeout(() => photoMarkers[k].openPopup(), 400);
        if (photo.distance != null) setCursor(indexAtDistance(points, photo.distance));
      });
      figure.classList.add("route-photo-figure");
      figure.append(button);
      photoPins.push(button);
    }),
  );
  cleanupFns.push(() => photoPins.forEach((button) => button.remove()));

  // --- Hover and click on the map

  const cursorMarker = L.circleMarker([0, 0], { radius: 7, color: "#111827", weight: 3, fillColor: "#fff", fillOpacity: 1, interactive: false });

  const nearestOnScreen = (containerPoint: L.Point) => {
    let best = -1;
    let bestDistance = Infinity;
    for (let i = 0; i < latlngs.length; i++) {
      const d = map.latLngToContainerPoint(latlngs[i]).distanceTo(containerPoint);
      if (d < bestDistance) (bestDistance = d), (best = i);
    }
    return { i: best, px: bestDistance };
  };

  map.on("mousemove", (event: L.LeafletMouseEvent) => {
    if (playing) return;
    const { i, px } = nearestOnScreen(event.containerPoint);
    if (px < 28) setCursor(i);
    else hideCursor();
  });
  map.on("mouseout", () => !playing && hideCursor());

  map.on("click", (event: L.LeafletMouseEvent) => {
    activate();
    const { i, px } = nearestOnScreen(event.containerPoint);
    if (marking) {
      // On the route: snap to it. Elsewhere (a view across the valley): exactly where clicked.
      if (px < 24) addWaypointAt(i);
      else {
        const near = nearestIndex(event.latlng.lat, event.latlng.lng);
        addWaypoint({ lat: event.latlng.lat, lon: event.latlng.lng, ele: near.meters < 60 ? points[near.i][ELE] : null, time: null, kind: "note", name: "" });
      }
      return;
    }
    if (px < 18) openPointPopup(i);
  });

  const openPointPopup = (i: number) => {
    const p = points[i];
    const grade = p[GRADE] != null ? ` · ${p[GRADE]! >= 0 ? "↗" : "↘"} ${Math.abs(Math.round(p[GRADE]!))} %` : "";
    L.popup(popupOptions)
      .setLatLng(latlngs[i])
      .setContent(
        `<p class="route-popup__title">${esc(kmLabel(i) + grade)}</p>
        ${pointDetails(p[LAT], p[LON], p[ELE], i, false)}
        ${editable ? `<div class="route-popup__actions"><button type="button" class="btn btn-primary btn-xs" data-action="mark-here" data-point="${i}">${esc(t("route.markHere"))}</button></div>` : ""}`,
      )
      .openOn(map);
  };

  // --- Elevation profile (SVG)

  const profileEl = $("[data-profile]");
  let profileScale: { x: (d: number) => number; y: (e: number) => number } | null = null;
  const uid = Math.random().toString(36).slice(2, 8);

  function drawProfile() {
    if (!profileEl) return;
    const W = profileEl.clientWidth;
    const H = profileEl.clientHeight;
    if (!W || !H) return;
    const showHr = mode === "hr" && heartRates.length > 0;
    const m = { l: 42, r: showHr ? 34 : 8, t: 24, b: 20 };
    const [eMin, eMax] = range.ele;
    const eStep = niceStep(eMax - eMin, 3);
    const lo = Math.floor(eMin / eStep) * eStep;
    const hi = Math.ceil(eMax / eStep) * eStep;
    const x = (d: number) => m.l + (d / total) * (W - m.l - m.r);
    const y = (e: number) => H - m.b - ((e - lo) / (hi - lo || 1)) * (H - m.t - m.b);
    const top = m.t - 18;
    const base = H - m.b;
    profileScale = { x, y };

    const withEle = points.filter((p) => p[ELE] != null);
    const line = withEle.map((p, i) => `${i ? "L" : "M"}${x(p[DIST]).toFixed(1)},${y(p[ELE]!).toFixed(1)}`).join("");
    const area = `${line}L${x(withEle[withEle.length - 1][DIST]).toFixed(1)},${base}L${x(withEle[0][DIST]).toFixed(1)},${base}Z`;

    // Horizontal gradient that follows the active color mode along the route.
    const stops = Array.from({ length: 160 }, (_, k) => {
      const i = Math.round((k / 159) * (points.length - 1));
      return `<stop offset="${((points[i][DIST] / total) * 100).toFixed(2)}%" stop-color="${colorAtIndex(i, mode)}"/>`;
    }).join("");

    const parts: string[] = [];
    parts.push(`<defs><linearGradient id="rg-${uid}" gradientUnits="userSpaceOnUse" x1="${m.l}" x2="${W - m.r}" y1="0" y2="0">${stops}</linearGradient></defs>`);

    // Day bands and camps.
    data.days.forEach((day, i) => {
      const x0 = x(day.fromDistance);
      const x1 = x(day.toDistance);
      if (i % 2 === 1) parts.push(`<rect x="${x0}" y="${top}" width="${x1 - x0}" height="${base - top}" fill="currentColor" opacity="0.05"/>`);
      parts.push(`<text x="${x0 + 4}" y="${m.t - 8}" font-size="10" font-weight="600" fill="currentColor" opacity="0.55">${esc(t("route.day", { n: i + 1 }))}</text>`);
    });
    data.camps.forEach((camp) => {
      const cx = x(camp.distance);
      parts.push(`<line x1="${cx}" x2="${cx}" y1="${top}" y2="${base}" stroke="#7c3aed" stroke-dasharray="3 3" opacity="0.8"/>`);
      parts.push(`<svg x="${cx - 7}" y="${(camp.ele != null ? y(camp.ele) : base) - 20}" width="14" height="14" viewBox="0 0 24 24" color="#7c3aed">${WAYPOINT_KINDS.camp.icon}</svg>`);
    });

    // Selection and highlight (moved without redrawing everything).
    parts.push(`<rect data-sel y="${top}" height="${base - top}" fill="oklch(var(--p))" opacity="0.16" style="display:none"/>`);
    parts.push(`<rect data-hl y="${top}" height="${base - top}" fill="currentColor" opacity="0.08" style="display:none"/>`);

    // Elevation grid.
    for (let e = lo; e <= hi + 0.1; e += eStep) {
      parts.push(`<line x1="${m.l}" x2="${W - m.r}" y1="${y(e)}" y2="${y(e)}" stroke="currentColor" opacity="0.08"/>`);
      parts.push(`<text x="${m.l - 6}" y="${y(e) + 3}" font-size="10" text-anchor="end" fill="currentColor" opacity="0.6">${Math.round(e).toLocaleString(locale)}</text>`);
    }
    const kmTick = niceStep(total / 1000, Math.max(2, Math.floor((W - m.l - m.r) / 70))) * 1000;
    for (let d = 0; d <= total + 1; d += kmTick) {
      parts.push(`<text x="${x(d)}" y="${H - 5}" font-size="10" text-anchor="middle" fill="currentColor" opacity="0.6">${(d / 1000).toLocaleString(locale)}${d === 0 ? " km" : ""}</text>`);
    }

    parts.push(`<path d="${area}" fill="url(#rg-${uid})" opacity="0.38"/>`);
    parts.push(`<path d="${line}" fill="none" stroke="url(#rg-${uid})" stroke-width="2.2" stroke-linejoin="round"/>`);

    if (showHr) {
      const [hMin, hMax] = range.hr;
      const yh = (h: number) => base - ((h - hMin) / (hMax - hMin || 1)) * (base - m.t);
      const hrSmooth = smooth(points.map((p) => p[HR]), 8);
      const hrLine = points
        .map((p, i) => [p[DIST], hrSmooth[i]] as const)
        .filter(([, h]) => h != null)
        .map(([d, h], i) => `${i ? "L" : "M"}${x(d).toFixed(1)},${yh(h!).toFixed(1)}`)
        .join("");
      parts.push(`<path d="${hrLine}" fill="none" stroke="#dc2626" stroke-width="1.4" stroke-linejoin="round" opacity="0.85"/>`);
      parts.push(`<text x="${W - m.r + 4}" y="${m.t + 3}" font-size="10" fill="#dc2626">${hMax}</text>`);
      parts.push(`<text x="${W - m.r + 4}" y="${base}" font-size="10" fill="#dc2626">${hMin}</text>`);
      parts.push(`<text x="${W - m.r + 4}" y="${(m.t + base) / 2 + 3}" font-size="9" fill="#dc2626" opacity="0.8">♥</text>`);
    }

    // Marked points as small pins above the line, photos as camera ticks on the baseline.
    waypoints.forEach((w) => {
      const i = waypointIndex(w);
      if (i == null || points[i][ELE] == null) return;
      const cx = x(points[i][DIST]);
      const cy = y(points[i][ELE]!);
      parts.push(`<g color="${WAYPOINT_KINDS[w.kind].color}"><title>${esc(w.name)}</title><circle cx="${cx}" cy="${cy - 11}" r="7" fill="currentColor" stroke="oklch(var(--b2))" stroke-width="2"/><svg x="${cx - 4.5}" y="${cy - 15.5}" width="9" height="9" viewBox="0 0 24 24" color="#fff">${WAYPOINT_KINDS[w.kind].icon}</svg></g>`);
    });
    if (photosOn) {
      data.photos.forEach((photo) => {
        if (photo.distance == null) return;
        const cx = x(photo.distance);
        parts.push(`<g opacity="0.75"><title>${esc(photo.caption || photo.alt)}</title><svg x="${cx - 5}" y="${base - 11}" width="10" height="10" viewBox="0 0 24 24" color="currentColor">${ICONS.camera}</svg></g>`);
      });
    }

    if (data.summit?.ele != null) {
      const sx = x(data.summit.distance);
      const sy = y(data.summit.ele);
      parts.push(`<path d="M${sx - 5},${sy - 4} L${sx},${sy - 12} L${sx + 5},${sy - 4}Z" fill="#d97706"/>`);
      const anchor = sx > W - 70 ? "end" : sx < m.l + 40 ? "start" : "middle";
      parts.push(`<text x="${sx}" y="${sy - 15}" font-size="11" font-weight="700" text-anchor="${anchor}" fill="currentColor">${formatElevation(data.summit.ele, locale)}</text>`);
    }

    parts.push(`<g data-cursor style="display:none"><line y1="${top}" y2="${base}" stroke="currentColor" stroke-width="1" opacity="0.5"/><circle r="4.5" fill="#fff" stroke="#111827" stroke-width="2.5"/></g>`);

    profileEl.innerHTML = `<svg width="${W}" height="${H}" class="block overflow-visible text-base-content">${parts.join("")}</svg>`;
    if (cursorIndex != null) placeProfileCursor(cursorIndex);
    placeRect("[data-sel]", selection);
    placeRect("[data-hl]", highlighted);
  }

  function placeRect(selector: string, span: [number, number] | null) {
    const rect = profileEl?.querySelector<SVGRectElement>(selector);
    if (!rect || !profileScale) return;
    if (!span) return void (rect.style.display = "none");
    const x0 = profileScale.x(points[Math.min(...span)][DIST]);
    const x1 = profileScale.x(points[Math.max(...span)][DIST]);
    rect.setAttribute("x", String(x0));
    rect.setAttribute("width", String(Math.max(1, x1 - x0)));
    rect.style.display = "";
  }

  const placeProfileCursor = (i: number) => {
    const group = profileEl?.querySelector<SVGGElement>("[data-cursor]");
    if (!group || !profileScale || points[i][ELE] == null) return;
    const cx = profileScale.x(points[i][DIST]);
    group.style.display = "";
    group.querySelector("line")!.setAttribute("x1", String(cx));
    group.querySelector("line")!.setAttribute("x2", String(cx));
    group.querySelector("circle")!.setAttribute("cx", String(cx));
    group.querySelector("circle")!.setAttribute("cy", String(profileScale.y(points[i][ELE]!)));
  };

  const indexFromProfileX = (clientX: number) => {
    if (!profileEl || !profileScale) return 0;
    const rect = profileEl.getBoundingClientRect();
    const left = profileScale.x(0);
    const right = profileScale.x(total);
    const d = ((clientX - rect.left - left) / (right - left)) * total;
    return indexAtDistance(points, Math.max(0, Math.min(total, d)));
  };

  // Profile: hover moves the cursor, a click centers the map there (or marks it), a drag selects.
  let drag: { start: number; x: number; moved: boolean } | null = null;
  if (profileEl) {
    profileEl.addEventListener("pointerdown", (event) => {
      profileEl.setPointerCapture(event.pointerId);
      drag = { start: indexFromProfileX(event.clientX), x: event.clientX, moved: false };
    });
    profileEl.addEventListener("pointermove", (event) => {
      const i = indexFromProfileX(event.clientX);
      if (drag) {
        if (Math.abs(event.clientX - drag.x) > 6) drag.moved = true;
        if (drag.moved) select(drag.start, i);
        setCursor(i);
      } else if (playing) replayDistance = points[i][DIST];
      else setCursor(i);
    });
    profileEl.addEventListener("pointerup", (event) => {
      const i = indexFromProfileX(event.clientX);
      if (drag && !drag.moved) {
        if (marking) addWaypointAt(i);
        else {
          setCursor(i);
          if (is3d) view3d?.flyTo(points[i][LAT], points[i][LON]);
          else map.panTo(latlngs[i]);
        }
      }
      drag = null;
    });
    profileEl.addEventListener("pointercancel", () => (drag = null));
    profileEl.addEventListener("pointerleave", () => !playing && !drag && hideCursor());
  }

  // --- Cursor + readout

  const readout = $("[data-readout]")!;

  function setCursor(i: number) {
    cursorIndex = i;
    const p = points[i];
    cursorMarker.setLatLng(latlngs[i]);
    if (!map.hasLayer(cursorMarker)) cursorMarker.addTo(map);
    placeProfileCursor(i);
    view3d?.setCursor(i);

    const parts = [formatDistance(p[DIST], locale, 2)];
    if (p[ELE] != null) parts.push(formatElevation(p[ELE]!, locale));
    if (mode === "speed" && speeds[i] != null) parts.push(`${speeds[i]!.toLocaleString(locale, { maximumFractionDigits: 1 })} km/h`);
    else if (mode === "vspeed" && vspeeds[i] != null) parts.push(`${vspeeds[i]! >= 0 ? "↑" : "↓"} ${Math.abs(Math.round(vspeeds[i]!)).toLocaleString(locale)} m/h`);
    else if (p[GRADE] != null) parts.push(`${p[GRADE]! >= 0 ? "↗" : "↘"} ${Math.abs(Math.round(p[GRADE]!))} %`);
    const clock = clockAt(i);
    if (clock) parts.push(clock);
    if (p[HR] != null) parts.push(`♥ ${p[HR]} ${t("route.bpm")}`);
    readout.textContent = parts.join("  ·  ");
    readout.hidden = false;
  }

  function hideCursor() {
    cursorIndex = null;
    cursorMarker.remove();
    readout.hidden = true;
    view3d?.setCursor(null);
    const group = profileEl?.querySelector<SVGGElement>("[data-cursor]");
    if (group) group.style.display = "none";
  }

  // --- Section selection

  const selectionBar = $("[data-selection]");
  const selectionStats = $("[data-selection-stats]");
  const selectHint = $("[data-select-hint]");

  function select(from: number, to: number, zoom = false) {
    const a = Math.min(from, to);
    const b = Math.max(from, to);
    if (b - a < 1) return;
    selection = [a, b];
    selectionLine.setLatLngs(latlngs.slice(a, b + 1));
    placeRect("[data-sel]", selection);
    view3d?.setSelection(a, b);
    const s = sectionStats(points, a, b);
    const facts = [
      formatDistance(s.distance, locale, 2),
      `+${formatElevation(s.gain, locale)} −${formatElevation(s.loss, locale)}`,
      s.avgGrade != null && `${t("route.avgGrade")} ${formatPercent(s.avgGrade, locale)}`,
      s.movingTime != null && `${formatDuration(s.movingTime)} ${t("route.moving").toLowerCase()}`,
      s.duration != null && s.movingTime != null && s.duration - s.movingTime > 120 && `${formatDuration(s.duration)} ${t("route.total").toLowerCase()}`,
      s.vam != null && Math.max(s.gain, s.loss) > 30 && `${s.gain >= s.loss ? "↑" : "↓"} ${Math.round(s.vam).toLocaleString(locale)} m/h`,
      s.speed != null && formatSpeed(s.speed, locale),
      s.avgHr != null && `♥ ${s.avgHr} ${t("route.bpm")}`,
    ].filter(Boolean) as string[];
    if (selectionStats) selectionStats.innerHTML = facts.map((f) => `<span>${esc(f)}</span>`).join("");
    if (selectionBar) selectionBar.hidden = false;
    if (selectHint) selectHint.hidden = true;
    if (zoom) zoomToSelection();
  }
  function zoomToSelection() {
    if (!selection) return;
    if (is3d) view3d?.fitRange(selection[0], selection[1]);
    else map.fitBounds(L.latLngBounds(latlngs.slice(selection[0], selection[1] + 1)), fitPadding);
  }
  const clearSelection = () => {
    selection = null;
    selectionLine.setLatLngs([]);
    placeRect("[data-sel]", null);
    view3d?.setSelection(null);
    if (selectionBar) selectionBar.hidden = true;
  };
  $("[data-selection-zoom]")?.addEventListener("click", zoomToSelection);
  $("[data-selection-clear]")?.addEventListener("click", clearSelection);

  const highlight = (from: number, to: number) => {
    highlighted = [Math.min(from, to), Math.max(from, to)];
    highlightLine.setLatLngs(latlngs.slice(highlighted[0], highlighted[1] + 1));
    placeRect("[data-hl]", highlighted);
  };
  const clearHighlight = () => {
    highlighted = null;
    highlightLine.setLatLngs([]);
    placeRect("[data-hl]", null);
  };

  // --- Color mode + legend

  const legendBar = $("[data-legend-bar]")!;
  const legendMin = $("[data-legend-min]")!;
  const legendMax = $("[data-legend-max]")!;
  const updateLegend = () => {
    const stops = SCALES[mode];
    const maxDomain = stops[stops.length - 1][0];
    legendBar.style.background = `linear-gradient(to right, ${stops.map(([v, c]) => `${c} ${(v / maxDomain) * 100}%`).join(", ")})`;
    const [min, max] =
      mode === "grade"
        ? ["0 %", "50 %+"]
        : mode === "elevation"
          ? [formatElevation(range.ele[0], locale), formatElevation(range.ele[1], locale)]
          : mode === "speed"
            ? ["0 km/h", `${range.speed[1].toLocaleString(locale, { maximumFractionDigits: 1 })}+ km/h`]
            : mode === "vspeed"
              ? [`↓ ${VSPEED_RANGE.toLocaleString(locale)} m/h`, `↑ ${VSPEED_RANGE.toLocaleString(locale)} m/h`]
              : [`${range.hr[0]} ${t("route.bpm")}`, `${range.hr[1]} ${t("route.bpm")}`];
    legendMin.textContent = min;
    legendMax.textContent = max;
  };
  updateLegend();

  $<HTMLSelectElement>("[data-mode-select]")?.addEventListener("change", (event) => {
    mode = (event.target as HTMLSelectElement).value as Mode;
    drawRoute();
    drawProfile();
    updateLegend();
    view3d?.setColors(colors());
    if (cursorIndex != null) setCursor(cursorIndex);
  });

  // --- Marking mode (route tool)

  const markButton = $<HTMLButtonElement>("[data-mark]");
  const markHint = $("[data-mark-hint]");
  function setMarking(on: boolean) {
    marking = on;
    if (on && is3d) toggle3d(false);
    markButton?.setAttribute("aria-pressed", String(on));
    markButton?.classList.toggle("btn-primary", on);
    mapEl.classList.toggle("route-map__map--marking", on);
    if (markHint) markHint.hidden = !on;
  }
  markButton?.addEventListener("click", () => setMarking(!marking));
  // The hint would cover an open popup (the editor, mostly).
  map.on("popupopen", () => markHint && (markHint.hidden = true));
  map.on("popupclose", () => markHint && (markHint.hidden = !marking));

  const onKey = (event: KeyboardEvent) => {
    if (event.key !== "Escape" || !root.matches(":hover, :focus-within")) return;
    if (marking) setMarking(false);
    else if (selection) clearSelection();
  };
  document.addEventListener("keydown", onKey);
  cleanupFns.push(() => document.removeEventListener("keydown", onKey));

  // --- 3D (MapLibre and the terrain are only downloaded the first time it's opened)

  const button3d = $<HTMLButtonElement>("[data-3d]");
  const container3d = $("[data-map3d]");
  let loading3d: Promise<void> | null = null;
  async function toggle3d(on = !is3d) {
    if (!container3d) return;
    is3d = on;
    button3d?.setAttribute("aria-pressed", String(on));
    button3d?.classList.toggle("btn-primary", on);
    container3d.hidden = !on;
    if (on) setMarking(false);
    if (on && !view3d) {
      loading3d ??= import("./route-3d").then(({ create3dView }) => {
        view3d = create3dView(container3d, {
          points,
          colors: colors(),
          waypoints,
          fixed: [
            ...(data.summit ? [{ lat: data.summit.lat, lon: data.summit.lon, kind: "summit" as const, title: `${t("route.summit")} · ${formatElevation(data.summit.ele ?? 0, locale)}` }] : []),
            ...data.camps.map((camp) => ({ lat: camp.lat, lon: camp.lon, kind: "camp" as const, title: t("route.camp") })),
          ],
          photos: photosOn ? data.photos : [],
          labels: { satellite: t("route.layer.satellite"), topo: t("route.layer.topo"), failed: t("route.view3dFailed") },
          onHover: (i) => {
            if (playing) return;
            if (i == null) hideCursor();
            else setCursor(i);
          },
          onPhoto: (k) => {
            toggle3d(false);
            map.setView([data.photos[k].lat, data.photos[k].lon], Math.max(map.getZoom(), 14));
            setTimeout(() => photoMarkers[k].openPopup(), 300);
          },
        });
        if (selection) view3d.setSelection(selection[0], selection[1]);
      });
      await loading3d;
    }
    if (on) view3d?.resize();
    else map.invalidateSize();
  }
  button3d?.addEventListener("click", () => toggle3d());

  // --- Replay

  const playButton = $<HTMLButtonElement>("[data-play]");
  let playing = false;
  let replayDistance = 0;
  let frame = 0;
  let lastTime = 0;

  const stopReplay = () => {
    playing = false;
    cancelAnimationFrame(frame);
    delete root.dataset.playing;
    playButton?.setAttribute("aria-label", t("route.play"));
    playButton?.setAttribute("title", t("route.play"));
    view3d?.stopFollow();
  };

  const step = (now: number) => {
    // In 3D the flight is slower, so the terrain can be taken in.
    replayDistance += ((now - lastTime) / (is3d ? REPLAY_MS * 2.5 : REPLAY_MS)) * total;
    lastTime = now;
    const i = indexAtDistance(points, Math.min(replayDistance, total));
    setCursor(i);
    if (is3d) view3d?.follow(i);
    else if (!map.getBounds().pad(-0.2).contains(latlngs[i])) map.panTo(latlngs[i], { animate: true, duration: 0.6 });
    if (replayDistance >= total) return stopReplay();
    frame = requestAnimationFrame(step);
  };

  playButton?.addEventListener("click", () => {
    if (playing) return stopReplay();
    playing = true;
    root.dataset.playing = "";
    playButton.setAttribute("aria-label", t("route.pause"));
    playButton.setAttribute("title", t("route.pause"));
    replayDistance = cursorIndex != null && cursorIndex < points.length - 1 ? points[cursorIndex][DIST] : 0;
    lastTime = performance.now();
    frame = requestAnimationFrame(step);
  });

  // --- Analysis panel (its code is loaded when it's first opened)

  const scrollToMap = () => mapEl.scrollIntoView({ behavior: "smooth", block: "center" });
  const api: RouteMapApi = {
    data,
    t,
    waypoints: () => waypoints,
    waypointIndex,
    setCursor,
    hideCursor: () => !playing && hideCursor(),
    select: (from, to, zoom) => {
      select(from, to, zoom);
      if (zoom) scrollToMap();
    },
    highlight,
    clearHighlight,
    focusWaypoint: (index) => {
      scrollToMap();
      focusWaypoint(index);
    },
    focusPoint: (i) => {
      scrollToMap();
      setCursor(i);
      if (is3d) view3d?.flyTo(points[i][LAT], points[i][LON]);
      else map.setView(latlngs[i], Math.max(map.getZoom(), 14));
    },
    addWaypointAt: (i, kind, name) => {
      scrollToMap();
      addWaypointAt(i, kind, name);
    },
    editWaypoint: (index) => {
      scrollToMap();
      focusWaypoint(index);
      editWaypoint(index);
    },
  };
  const analysis = $("[data-analysis]");
  const analysisBody = $("[data-analysis-body]");
  const analysisToggle = $<HTMLButtonElement>("[data-analysis-toggle]");
  const openAnalysis = async (open: boolean) => {
    if (!analysis || !analysisBody) return;
    analysis.toggleAttribute("data-open", open);
    analysisBody.hidden = !open;
    analysisToggle?.setAttribute("aria-expanded", String(open));
    if (open && !panels) {
      const { createPanels } = await import("./route-panels");
      panels = createPanels(analysisBody, api);
    }
  };
  analysisToggle?.addEventListener("click", () => openAnalysis(!analysis?.hasAttribute("data-open")));
  if (analysis?.hasAttribute("data-open")) openAnalysis(true);

  // --- Buttons, resize, fullscreen

  $("[data-recenter]")?.addEventListener("click", fit);

  const fullscreenButton = $<HTMLButtonElement>("[data-fullscreen]");
  if (!document.fullscreenEnabled) fullscreenButton?.setAttribute("hidden", "");
  fullscreenButton?.addEventListener("click", () => {
    if (document.fullscreenElement) document.exitFullscreen();
    else root.requestFullscreen();
  });
  const onResize = () => {
    map.invalidateSize();
    view3d?.resize();
    drawProfile();
  };
  document.addEventListener("fullscreenchange", onResize);

  const resizeObserver = new ResizeObserver(onResize);
  resizeObserver.observe(root);
  setPhotos(photosOn);

  return () => {
    stopReplay();
    resizeObserver.disconnect();
    document.removeEventListener("fullscreenchange", onResize);
    cleanupFns.forEach((fn) => fn());
    view3d?.destroy();
    map.remove();
  };
}

// --- Helpers

export function esc(value: unknown) {
  return String(value).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");
}

function readStorage(key: string) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null;
  }
}

function writeStorage(key: string, value: string) {
  try {
    localStorage.setItem(key, value);
  } catch {
    // Private mode or storage disabled: the setting just isn't remembered.
  }
}

function smooth(values: (number | null)[], radius: number) {
  return values.map((v, i) => {
    if (v == null) return null;
    let sum = 0;
    let count = 0;
    for (let j = Math.max(0, i - radius); j <= Math.min(values.length - 1, i + radius); j++) {
      if (values[j] != null) (sum += values[j]!), count++;
    }
    return sum / count;
  });
}

function normalize(value: number, [min, max]: readonly [number, number]) {
  return max > min ? (value - min) / (max - min) : 0;
}

export function indexAtDistance(points: RoutePoint[], d: number) {
  let lo = 0;
  let hi = points.length - 1;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (points[mid][DIST] < d) lo = mid + 1;
    else hi = mid;
  }
  return lo > 0 && d - points[lo - 1][DIST] < points[lo][DIST] - d ? lo - 1 : lo;
}

/** A round tick step (1, 2, 2.5 or 5 × 10^n) giving about `count` intervals over `span`. */
function niceStep(span: number, count: number) {
  const raw = span / Math.max(1, count);
  const power = 10 ** Math.floor(Math.log10(raw || 1));
  return ([1, 2, 2.5, 5, 10].find((f) => f * power >= raw) ?? 10) * power;
}

function colorAt(stops: Stops, value: number) {
  if (value <= stops[0][0]) return stops[0][1];
  for (let i = 1; i < stops.length; i++) {
    const [v1, c1] = stops[i];
    if (value <= v1) {
      const [v0, c0] = stops[i - 1];
      // Quantize to 1/24 steps so neighbouring segments share colors and merge into fewer polylines.
      const t = Math.round(((value - v0) / (v1 - v0)) * 24) / 24;
      return mix(c0, c1, t);
    }
  }
  return stops[stops.length - 1][1];
}

/** Color of a grade (fraction) on the map's grade scale, for charts that should match it. */
export function gradeColor(gradeFraction: number) {
  return colorAt(SCALES.grade, Math.min(Math.abs(gradeFraction), 0.5));
}

function mix(a: string, b: string, t: number) {
  const pa = parseInt(a.slice(1), 16);
  const pb = parseInt(b.slice(1), 16);
  const channel = (shift: number) => Math.round(((pa >> shift) & 255) * (1 - t) + ((pb >> shift) & 255) * t);
  return `#${((1 << 24) | (channel(16) << 16) | (channel(8) << 8) | channel(0)).toString(16).slice(1)}`;
}
