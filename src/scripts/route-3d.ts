// 3D view of a route: MapLibre GL with free terrain (AWS Terrain Tiles, from Mapzen) under Esri
// satellite imagery or OpenTopoMap. Loaded on demand by route-map.ts the first time "3D" is
// pressed, so pages without it never download MapLibre.

import maplibregl, { type GeoJSONSource, type LngLatLike, type Map as MapLibreMap } from "maplibre-gl";
import "maplibre-gl/dist/maplibre-gl.css";
import type { RoutePoint, Waypoint } from "../lib/gpx-core";
import type { RoutePhoto } from "../lib/photos";
import { WAYPOINT_KINDS } from "../lib/waypoint-kinds";

export interface View3D {
  setColors(colors: string[]): void;
  setCursor(i: number | null): void;
  /** Camera follows the route at point i, looking ahead (route replay) */
  follow(i: number): void;
  stopFollow(): void;
  setWaypoints(waypoints: (Waypoint & { color?: string })[]): void;
  setPhotos(photos: RoutePhoto[]): void;
  setSelection(from: number | null, to?: number): void;
  flyTo(lat: number, lon: number): void;
  fit(): void;
  fitRange(from: number, to: number): void;
  resize(): void;
  destroy(): void;
}

interface Options {
  points: RoutePoint[];
  colors: string[];
  waypoints: (Waypoint & { color?: string })[];
  /** Always-shown pins: summit, camps */
  fixed: { lat: number; lon: number; kind: keyof typeof WAYPOINT_KINDS; title: string }[];
  photos: RoutePhoto[];
  labels: { satellite: string; topo: string; failed: string };
  onHover: (i: number | null) => void;
  onPhoto: (index: number) => void;
}

const LAT = 0, LON = 1, DIST = 3;
const TERRAIN_TILES = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png";
const EXAGGERATION = 1.2;
// Closer than this the imagery is just blur; and the camera keeps this far above the ground.
const MAX_ZOOM = 17;
const MIN_CLEARANCE_M = 150;

export function create3dView(container: HTMLElement, options: Options): View3D {
  const { points, labels } = options;
  const total = points[points.length - 1][DIST];
  const coordinates = points.map((p) => [p[LON], p[LAT]] as [number, number]);
  const bounds = coordinates.reduce((b, c) => b.extend(c), new maplibregl.LngLatBounds(coordinates[0], coordinates[0]));

  const element = document.createElement("div");
  // Inline, because MapLibre's stylesheet makes its container position: relative.
  element.style.cssText = "position:absolute;inset:0";
  container.append(element);

  let map: MapLibreMap;
  try {
    map = new maplibregl.Map({
      container: element,
      bounds,
      fitBoundsOptions: { padding: { top: 70, bottom: 90, left: 40, right: 40 } },
      pitch: 55,
      maxPitch: 75,
      maxZoom: MAX_ZOOM,
      attributionControl: false,
      style: {
        version: 8,
        sources: {
          satellite: {
            type: "raster",
            tiles: ["https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"],
            tileSize: 256,
            maxzoom: 18,
            attribution: "Imagery © Esri, Maxar, Earthstar Geographics",
          },
          topo: {
            type: "raster",
            tiles: ["a", "b", "c"].map((s) => `https://${s}.tile.opentopomap.org/{z}/{x}/{y}.png`),
            tileSize: 256,
            maxzoom: 17,
            attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>, SRTM · <a href="https://opentopomap.org">OpenTopoMap</a>',
          },
          // Two sources on the same tiles, as MapLibre recommends for terrain + hillshade.
          terrain: { type: "raster-dem", tiles: [TERRAIN_TILES], tileSize: 256, maxzoom: 15, encoding: "terrarium", attribution: "Terrain: Mapzen / AWS Terrain Tiles" },
          hillshade: { type: "raster-dem", tiles: [TERRAIN_TILES], tileSize: 256, maxzoom: 15, encoding: "terrarium" },
          route: { type: "geojson", lineMetrics: true, data: { type: "Feature", properties: {}, geometry: { type: "LineString", coordinates } } },
          selection: { type: "geojson", data: emptyLine() },
        },
        layers: [
          { id: "background", type: "background", paint: { "background-color": "#9db4c0" } },
          { id: "satellite", type: "raster", source: "satellite" },
          { id: "topo", type: "raster", source: "topo", layout: { visibility: "none" } },
          { id: "hillshade", type: "hillshade", source: "hillshade", layout: { visibility: "none" }, paint: { "hillshade-exaggeration": 0.35, "hillshade-shadow-color": "#3d2f1f" } },
          { id: "route-casing", type: "line", source: "route", layout: { "line-cap": "round", "line-join": "round" }, paint: { "line-color": "#111827", "line-width": 8, "line-opacity": 0.55 } },
          { id: "selection", type: "line", source: "selection", layout: { "line-cap": "round", "line-join": "round" }, paint: { "line-color": "#ffffff", "line-width": 13, "line-opacity": 0.85 } },
          { id: "route", type: "line", source: "route", layout: { "line-cap": "round", "line-join": "round" }, paint: { "line-width": 4.5, "line-gradient": gradient(points, options.colors, total) } },
        ],
        terrain: { source: "terrain", exaggeration: EXAGGERATION },
        sky: {
          "sky-color": "#7fb0e6",
          "horizon-color": "#dbe8f5",
          "fog-color": "#eef3f8",
          "sky-horizon-blend": 0.6,
          "horizon-fog-blend": 0.6,
          "fog-ground-blend": 0.85,
          "atmosphere-blend": 0.6,
        },
      },
    });
  } catch {
    element.innerHTML = `<p class="grid h-full place-items-center p-6 text-center text-sm">${labels.failed}</p>`;
    return noop();
  }

  // The initial bounds are fitted looking straight down; refit with the tilt once the terrain is in.
  map.once("load", () => map.fitBounds(bounds, { padding: { top: 70, bottom: 90, left: 40, right: 40 }, pitch: 55, duration: 0 }));
  map.addControl(new maplibregl.NavigationControl({ visualizePitch: true }), "top-left");
  map.addControl(new maplibregl.AttributionControl({ compact: true }), "bottom-right");
  map.addControl(new maplibregl.ScaleControl({ unit: "metric" }), "bottom-right");

  // Base layer switch: satellite (default) or topographic with hillshade.
  const switcher = document.createElement("div");
  switcher.className = "route-3d__switch join absolute left-14 top-3 z-10 rounded-lg bg-base-100/90 shadow backdrop-blur";
  switcher.innerHTML = [
    ["satellite", labels.satellite],
    ["topo", labels.topo],
  ]
    .map(([id, label], k) => `<button type="button" class="btn btn-xs join-item border-0 ${k === 0 ? "btn-primary" : "btn-ghost"}" data-base="${id}">${label}</button>`)
    .join("");
  switcher.addEventListener("click", (event) => {
    const button = (event.target as HTMLElement).closest<HTMLButtonElement>("[data-base]");
    if (!button) return;
    const topo = button.dataset.base === "topo";
    map.setLayoutProperty("satellite", "visibility", topo ? "none" : "visible");
    map.setLayoutProperty("topo", "visibility", topo ? "visible" : "none");
    map.setLayoutProperty("hillshade", "visibility", topo ? "visible" : "none");
    switcher.querySelectorAll<HTMLButtonElement>("[data-base]").forEach((b) => {
      b.classList.toggle("btn-primary", b === button);
      b.classList.toggle("btn-ghost", b !== button);
    });
  });
  element.append(switcher);

  // Cursor
  const cursorEl = document.createElement("div");
  cursorEl.className = "route-3d__cursor";
  const cursor = new maplibregl.Marker({ element: cursorEl });
  let cursorShown = false;

  // Hover near the route moves the shared cursor.
  map.on("mousemove", (event) => {
    if (following) return;
    let best = -1;
    let bestDistance = Infinity;
    for (let i = 0; i < coordinates.length; i += 1) {
      const p = map.project(coordinates[i]);
      const d = Math.hypot(p.x - event.point.x, p.y - event.point.y);
      if (d < bestDistance) (bestDistance = d), (best = i);
    }
    options.onHover(bestDistance < 28 ? best : null);
  });
  map.on("mouseout", () => options.onHover(null));

  // Markers
  let waypointMarkers: maplibregl.Marker[] = [];
  let photoMarkers: maplibregl.Marker[] = [];
  const pinMarker = (w: { lat: number; lon: number; kind: keyof typeof WAYPOINT_KINDS; color?: string }, title: string, size = 26) => {
    const el = document.createElement("div");
    el.className = "route-pin";
    el.style.cssText = `width:${size}px;height:${size}px;background:${w.color ?? WAYPOINT_KINDS[w.kind].color}`;
    el.innerHTML = `<svg viewBox="0 0 24 24" width="${size * 0.55}" height="${size * 0.55}">${WAYPOINT_KINDS[w.kind].icon}</svg>`;
    el.title = title;
    return new maplibregl.Marker({ element: el })
      .setLngLat([w.lon, w.lat])
      .setPopup(new maplibregl.Popup({ offset: size / 2 + 4, closeButton: false }).setText(title))
      .addTo(map);
  };
  options.fixed.forEach((f) => pinMarker(f, f.title, 28));
  const setWaypoints = (waypoints: (Waypoint & { color?: string })[]) => {
    waypointMarkers.forEach((m) => m.remove());
    waypointMarkers = waypoints.map((w) => pinMarker(w, w.name, w.kind === "summit" ? 30 : 26));
  };
  const setPhotos = (photos: RoutePhoto[]) => {
    photoMarkers.forEach((m) => m.remove());
    photoMarkers = photos.map((photo, k) => {
      const el = document.createElement("button");
      el.type = "button";
      el.className = "route-photo";
      el.style.backgroundImage = `url('${encodeURI(photo.src)}')`;
      el.title = photo.caption || photo.alt;
      el.addEventListener("click", () => options.onPhoto(k));
      return new maplibregl.Marker({ element: el }).setLngLat([photo.lon, photo.lat]).addTo(map);
    });
  };
  setWaypoints(options.waypoints);
  setPhotos(options.photos);

  // Keep the camera above the ground. Zooming towards a slope, or tilting low near a ridge, can
  // put it inside the mountain; there MapLibre's wheel zoom breaks (it can't find the point under
  // the cursor) and every gesture feels stuck. So: zooming in stops when the camera nears the
  // ground, after each move it backs out until it clears the terrain, and if the wheel handler
  // breaks anyway it's reset.
  let unburying = false;
  const clearance = () => {
    const { terrain, transform } = map as unknown as {
      terrain?: { getElevationForLngLatZoom(lngLat: maplibregl.LngLat, zoom: number): number };
      transform: { tileZoom: number; getCameraPosition(): { lngLat: maplibregl.LngLat; altitude: number } };
    };
    if (!terrain) return Infinity;
    const camera = transform.getCameraPosition();
    return camera.altitude - terrain.getElevationForLngLatZoom(camera.lngLat, transform.tileZoom);
  };
  map.on("move", () => {
    if (following || unburying) return;
    if (clearance() < MIN_CLEARANCE_M / 2 && map.getZoom() < map.getMaxZoom()) map.setMaxZoom(map.getZoom());
  });
  map.on("moveend", () => {
    if (following || unburying) return;
    map.setMaxZoom(MAX_ZOOM);
    if (clearance() >= MIN_CLEARANCE_M) return;
    unburying = true;
    const from = { zoom: map.getZoom(), pitch: map.getPitch() };
    const to = { ...from };
    // Mostly back out; flatten the view only when it's tilted a lot.
    for (let k = 0; k < 80 && clearance() < MIN_CLEARANCE_M; k++) {
      to.zoom -= 0.1;
      if (to.pitch > 60) to.pitch -= 1;
      map.jumpTo(to);
    }
    map.jumpTo(from);
    unburying = false;
    map.easeTo({ ...to, duration: 500 });
  });
  const recoverWheel = (event: ErrorEvent) => {
    if (/Invalid LngLat/.test(event.message)) (map.scrollZoom as unknown as { reset(): void }).reset();
  };
  window.addEventListener("error", recoverWheel);

  // Following the route during replay: look ~400 m ahead, turning smoothly.
  let following = false;
  let bearing = map.getBearing();
  const follow = (i: number) => {
    following = true;
    let ahead = i;
    while (ahead < points.length - 1 && points[ahead][DIST] - points[i][DIST] < 400) ahead++;
    const target = ahead > i ? bearingBetween(points[i], points[ahead]) : bearing;
    bearing += shortestTurn(bearing, target) * 0.04;
    map.jumpTo({ center: coordinates[i], bearing, pitch: 64, zoom: 14.6 });
  };

  return {
    setColors: (colors) => map.setPaintProperty("route", "line-gradient", gradient(points, colors, total)),
    setCursor: (i) => {
      if (i == null) {
        cursor.remove();
        cursorShown = false;
        return;
      }
      cursor.setLngLat(coordinates[i]);
      if (!cursorShown) cursor.addTo(map), (cursorShown = true);
    },
    follow,
    stopFollow: () => (following = false),
    setWaypoints,
    setPhotos,
    setSelection: (from, to) => {
      const source = map.getSource<GeoJSONSource>("selection");
      if (!source) return;
      source.setData(from == null || to == null ? emptyLine() : { type: "Feature", properties: {}, geometry: { type: "LineString", coordinates: coordinates.slice(from, to + 1) } });
    },
    flyTo: (lat, lon) => map.flyTo({ center: [lon, lat] as LngLatLike, zoom: Math.max(map.getZoom(), 14.5), pitch: 60, duration: 1200 }),
    fit: () => map.fitBounds(bounds, { padding: { top: 70, bottom: 90, left: 40, right: 40 }, pitch: 55, duration: 1000 }),
    fitRange: (from, to) => {
      const part = coordinates.slice(from, to + 1);
      map.fitBounds(part.reduce((b, c) => b.extend(c), new maplibregl.LngLatBounds(part[0], part[0])), {
        padding: { top: 70, bottom: 90, left: 40, right: 40 },
        pitch: 60,
        duration: 1000,
      });
    },
    resize: () => map.resize(),
    destroy: () => {
      window.removeEventListener("error", recoverWheel);
      map.remove();
    },
  };
}

/** A line-gradient expression following the per-point colors along the route. */
function gradient(points: RoutePoint[], colors: string[], total: number) {
  const stops: (string | number)[] = [];
  let last = -1;
  const samples = Math.min(points.length, 240);
  for (let k = 0; k < samples; k++) {
    const i = Math.round((k / (samples - 1)) * (points.length - 1));
    const progress = total > 0 ? points[i][DIST] / total : k / (samples - 1);
    if (progress <= last) continue; // stops must strictly increase
    stops.push(progress, colors[i]);
    last = progress;
  }
  return ["interpolate", ["linear"], ["line-progress"], ...stops] as maplibregl.ExpressionSpecification;
}

function emptyLine() {
  return { type: "Feature" as const, properties: {}, geometry: { type: "LineString" as const, coordinates: [] as [number, number][] } };
}

function bearingBetween(a: RoutePoint, b: RoutePoint) {
  const r = Math.PI / 180;
  const y = Math.sin((b[LON] - a[LON]) * r) * Math.cos(b[LAT] * r);
  const x = Math.cos(a[LAT] * r) * Math.sin(b[LAT] * r) - Math.sin(a[LAT] * r) * Math.cos(b[LAT] * r) * Math.cos((b[LON] - a[LON]) * r);
  return (Math.atan2(y, x) / r + 360) % 360;
}

function shortestTurn(from: number, to: number) {
  return ((to - from + 540) % 360) - 180;
}

function noop(): View3D {
  const nothing = () => {};
  return {
    setColors: nothing,
    setCursor: nothing,
    follow: nothing,
    stopFollow: nothing,
    setWaypoints: nothing,
    setPhotos: nothing,
    setSelection: nothing,
    flyTo: nothing,
    fit: nothing,
    fitRange: nothing,
    resize: nothing,
    destroy: nothing,
  };
}
