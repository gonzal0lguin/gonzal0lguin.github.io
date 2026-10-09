// Browser side of <RouteMap>: Leaflet map + SVG elevation profile, kept in sync.
// Hovering either one moves a cursor on both; "replay" animates the cursor along the route.

import L from "leaflet";
import "leaflet/dist/leaflet.css";
import type { RouteData, RoutePoint } from "../lib/gpx";
import { formatClock, formatDistance, formatElevation } from "../lib/route-format";

type Mode = "grade" | "elevation" | "hr";
type Payload = RouteData & { file: string; locale: string; timezone: string; labels: Record<string, string> };
type Stops = [number, string][];

const LAT = 0, LON = 1, ELE = 2, DIST = 3, TIME = 4, HR = 5, GRADE = 6;
const REPLAY_MS = 18000;

// Color scales. Grade uses absolute fractions (0.15 = 15 %); the others are normalized 0..1.
const SCALES: Record<Mode, Stops> = {
  grade: [[0, "#1a9850"], [0.08, "#91cf60"], [0.15, "#fee08b"], [0.25, "#fc8d59"], [0.35, "#d73027"], [0.5, "#7b1fa2"]],
  elevation: [[0, "#440154"], [0.25, "#3b528b"], [0.5, "#21918c"], [0.75, "#5ec962"], [1, "#fde725"]],
  hr: [[0, "#2563eb"], [0.35, "#22c55e"], [0.6, "#facc15"], [0.8, "#f97316"], [1, "#dc2626"]],
};

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
  peak: '<path d="M2 20 9.5 7l4 6.5L16 9.5 22 20z" fill="currentColor"/>',
  tent: '<path d="M2 20h20M4 20 12 5l8 15M12 5l2-2M12 5l-2-2M9.5 20l2.5-5 2.5 5" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"/>',
};

const cleanups = new Set<() => void>();
document.addEventListener("astro:before-swap", () => {
  cleanups.forEach((cleanup) => cleanup());
  cleanups.clear();
});

export function initRouteMaps() {
  document.querySelectorAll<HTMLElement>("[data-route-map]:not([data-ready])").forEach((root) => {
    root.dataset.ready = "";
    cleanups.add(createRouteMap(root));
  });
}

function createRouteMap(root: HTMLElement) {
  const data: Payload = JSON.parse(root.querySelector("[data-route-data]")!.textContent!);
  const { points, labels, locale, timezone } = data;
  const total = points[points.length - 1][DIST];
  const $ = <T extends Element = HTMLElement>(selector: string) => root.querySelector<T>(selector);

  const elevations = points.map((p) => p[ELE]).filter((e): e is number => e != null);
  const heartRates = points.map((p) => p[HR]).filter((h): h is number => h != null);
  const range = {
    ele: [Math.min(...elevations), Math.max(...elevations)] as const,
    hr: heartRates.length ? ([Math.min(...heartRates), Math.max(...heartRates)] as const) : ([0, 1] as const),
  };

  let mode: Mode = "grade";
  let cursorIndex: number | null = null;

  // --- Color helpers

  const rawValue = (p: RoutePoint, m: Mode): number | null => {
    if (m === "grade") return p[GRADE] == null ? null : Math.min(Math.abs(p[GRADE]!) / 100, 0.5);
    if (m === "elevation") return p[ELE] == null ? null : normalize(p[ELE]!, range.ele);
    return p[HR] == null ? null : normalize(p[HR]!, range.hr);
  };
  // Grade and heart rate are noisy point to point; average them over neighbours so colors read as zones.
  const values: Record<Mode, (number | null)[]> = {
    grade: smooth(points.map((p) => rawValue(p, "grade")), 8),
    elevation: points.map((p) => rawValue(p, "elevation")),
    hr: smooth(points.map((p) => rawValue(p, "hr")), 6),
  };
  const colorAtIndex = (i: number, m: Mode) => {
    const v = values[m][i];
    return v == null ? "#9ca3af" : colorAt(SCALES[m], v);
  };

  // --- Map

  const mapEl = $("[data-map]")!;
  const map = L.map(mapEl, {
    scrollWheelZoom: false,
    dragging: !L.Browser.mobile,
    preferCanvas: true,
    zoomSnap: 0.25,
  });
  const baseLayers = {
    [labels.topo]: L.tileLayer(TILES.topo.url, TILES.topo.options),
    [labels.satellite]: L.tileLayer(TILES.satellite.url, TILES.satellite.options),
    [labels.streets]: L.tileLayer(TILES.streets.url, TILES.streets.options),
  };
  baseLayers[labels.topo].addTo(map);
  L.control.layers(baseLayers, undefined, { position: "topright" }).addTo(map);
  L.control.scale({ imperial: false, position: "bottomright" }).addTo(map);

  // Only capture the scroll wheel (and one-finger drags on phones) after the visitor clicks the map.
  const activate = () => {
    map.scrollWheelZoom.enable();
    map.dragging.enable();
  };
  map.on("click", activate);
  mapEl.addEventListener("mouseleave", () => map.scrollWheelZoom.disable());

  const latlngs = points.map((p) => L.latLng(p[LAT], p[LON]));
  const bounds = L.latLngBounds(latlngs);
  // Extra room at the top and bottom for the overlaid controls, legend and readout.
  const fit = () => map.fitBounds(bounds, { paddingTopLeft: [40, 64], paddingBottomRight: [40, 56] });
  fit();

  L.polyline(latlngs, { color: "#111827", weight: 8, opacity: 0.5, interactive: false }).addTo(map);
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

  // Markers: start/finish, summit, camps, km posts, waypoints.
  const pin = (icon: string, color: string, size = 26) =>
    L.divIcon({
      className: "",
      html: `<div class="route-pin" style="width:${size}px;height:${size}px;background:${color}"><svg viewBox="0 0 24 24" width="${size * 0.55}" height="${size * 0.55}">${icon}</svg></div>`,
      iconSize: [size, size],
      iconAnchor: [size / 2, size / 2],
    });
  const label = (marker: L.Marker, text: string, permanent = false) =>
    marker.bindTooltip(text, permanent
      ? { direction: "right", offset: [16, 0], className: "route-label", permanent }
      : { direction: "top", offset: [0, -14], className: "route-label" });

  const first = points[0];
  const last = points[points.length - 1];
  const returnsToStart = data.stats.shape !== "one-way";
  label(L.marker([first[LAT], first[LON]], { icon: pin(ICONS.flag, "#16a34a"), zIndexOffset: 500 }).addTo(map), returnsToStart ? labels.startFinish : labels.start);
  if (!returnsToStart) label(L.marker([last[LAT], last[LON]], { icon: pin(ICONS.finish, "#111827"), zIndexOffset: 500 }).addTo(map), labels.finish);
  if (data.summit) {
    const text = `${labels.summit} · ${formatElevation(data.summit.ele ?? 0, locale)}`;
    // Always-visible label on wide maps; on phones it would cover the controls, so show it on tap.
    label(L.marker([data.summit.lat, data.summit.lon], { icon: pin(ICONS.peak, "#d97706", 30), zIndexOffset: 1000 }).addTo(map), text, mapEl.clientWidth >= 560);
  }
  data.camps.forEach((camp) => {
    const text = `${labels.camp}${camp.ele != null ? ` · ${formatElevation(camp.ele, locale)}` : ""}`;
    label(L.marker([camp.lat, camp.lon], { icon: pin(ICONS.tent, "#7c3aed", 28), zIndexOffset: 800 }).addTo(map), text);
  });
  const kmStep = total <= 15000 ? 1000 : total <= 40000 ? 5000 : 10000;
  // On out-and-back routes the way back retraces the way up, so only mark the way up.
  const kmUntil = data.stats.shape === "out-and-back" ? total / 2 : total;
  for (let km = kmStep, i = 0; km < kmUntil; km += kmStep) {
    while (i < points.length - 1 && points[i][DIST] < km) i++;
    const icon = L.divIcon({ className: "", html: `<div class="route-km" style="width:18px;height:18px">${km / 1000}</div>`, iconSize: [18, 18], iconAnchor: [9, 9] });
    L.marker(latlngs[i], { icon, interactive: false, keyboard: false }).addTo(map);
  }
  data.waypoints.forEach((w) => {
    L.circleMarker([w.lat, w.lon], { radius: 4, color: "#fff", weight: 1.5, fillColor: "#0ea5e9", fillOpacity: 1 }).bindTooltip(w.name).addTo(map);
  });

  const cursorMarker = L.circleMarker([0, 0], { radius: 7, color: "#111827", weight: 3, fillColor: "#fff", fillOpacity: 1, interactive: false });

  // Hovering near the line on the map moves the cursor.
  map.on("mousemove", (event: L.LeafletMouseEvent) => {
    if (playing) return;
    let best = -1;
    let bestDistance = Infinity;
    for (let i = 0; i < latlngs.length; i++) {
      const d = map.latLngToContainerPoint(latlngs[i]).distanceTo(event.containerPoint);
      if (d < bestDistance) (bestDistance = d), (best = i);
    }
    if (bestDistance < 28) setCursor(best);
    else hideCursor();
  });
  map.on("mouseout", () => !playing && hideCursor());

  // --- Elevation profile (SVG)

  const profileEl = $("[data-profile]");
  let profileScale: { x: (d: number) => number; y: (e: number) => number } | null = null;
  const uid = Math.random().toString(36).slice(2, 8);

  const drawProfile = () => {
    if (!profileEl) return;
    const W = profileEl.clientWidth;
    const H = profileEl.clientHeight;
    if (!W || !H) return;
    const showHr = mode === "hr" && heartRates.length > 0;
    const m = { l: 42, r: showHr ? 34 : 8, t: 22, b: 20 };
    const [eMin, eMax] = range.ele;
    const eStep = niceStep(eMax - eMin, 3);
    const lo = Math.floor(eMin / eStep) * eStep;
    const hi = Math.ceil(eMax / eStep) * eStep;
    const x = (d: number) => m.l + (d / total) * (W - m.l - m.r);
    const y = (e: number) => H - m.b - ((e - lo) / (hi - lo || 1)) * (H - m.t - m.b);
    profileScale = { x, y };
    const base = H - m.b;

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
      if (i % 2 === 1) parts.push(`<rect x="${x0}" y="${m.t - 16}" width="${x1 - x0}" height="${base - m.t + 16}" fill="currentColor" opacity="0.05"/>`);
      parts.push(`<text x="${x0 + 4}" y="${m.t - 6}" font-size="10" font-weight="600" fill="currentColor" opacity="0.55">${labels.day.replace("{n}", String(i + 1))}</text>`);
    });
    data.camps.forEach((camp) => {
      const cx = x(camp.distance);
      parts.push(`<line x1="${cx}" x2="${cx}" y1="${m.t - 16}" y2="${base}" stroke="#7c3aed" stroke-dasharray="3 3" opacity="0.8"/>`);
      parts.push(`<svg x="${cx - 7}" y="${(camp.ele != null ? y(camp.ele) : base) - 20}" width="14" height="14" viewBox="0 0 24 24" color="#7c3aed">${ICONS.tent}</svg>`);
    });

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

    if (data.summit?.ele != null) {
      const sx = x(data.summit.distance);
      const sy = y(data.summit.ele);
      parts.push(`<path d="M${sx - 5},${sy - 4} L${sx},${sy - 12} L${sx + 5},${sy - 4}Z" fill="#d97706"/>`);
      const anchor = sx > W - 70 ? "end" : sx < m.l + 40 ? "start" : "middle";
      parts.push(`<text x="${sx}" y="${sy - 15}" font-size="11" font-weight="700" text-anchor="${anchor}" fill="currentColor">${formatElevation(data.summit.ele, locale)}</text>`);
    }

    parts.push(`<g data-cursor style="display:none"><line y1="${m.t - 16}" y2="${base}" stroke="currentColor" stroke-width="1" opacity="0.5"/><circle r="4.5" fill="#fff" stroke="#111827" stroke-width="2.5"/></g>`);

    profileEl.innerHTML = `<svg width="${W}" height="${H}" class="block overflow-visible text-base-content">${parts.join("")}</svg>`;
    if (cursorIndex != null) placeProfileCursor(cursorIndex);
  };

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

  if (profileEl) {
    profileEl.addEventListener("pointermove", (event) => {
      const i = indexFromProfileX(event.clientX);
      if (playing) replayDistance = points[i][DIST];
      else setCursor(i);
    });
    profileEl.addEventListener("pointerleave", () => !playing && hideCursor());
    profileEl.addEventListener("pointerdown", (event) => {
      const i = indexFromProfileX(event.clientX);
      setCursor(i);
      map.panTo(latlngs[i]);
    });
  }

  // --- Cursor + readout

  const readout = $("[data-readout]")!;

  function setCursor(i: number) {
    cursorIndex = i;
    const p = points[i];
    cursorMarker.setLatLng(latlngs[i]);
    if (!map.hasLayer(cursorMarker)) cursorMarker.addTo(map);
    placeProfileCursor(i);

    const parts = [formatDistance(p[DIST], locale, 2)];
    if (p[ELE] != null) parts.push(formatElevation(p[ELE]!, locale));
    if (p[GRADE] != null) parts.push(`${p[GRADE]! >= 0 ? "↗" : "↘"} ${Math.abs(Math.round(p[GRADE]!))} %`);
    if (p[TIME] != null && data.startTime != null) {
      const at = data.startTime + p[TIME]!;
      const day = data.days.findIndex((d) => at >= d.start && at <= d.end);
      parts.push(`${day >= 0 ? `${labels.day.replace("{n}", String(day + 1))} ` : ""}${formatClock(at, locale, timezone)}`);
    }
    if (p[HR] != null) parts.push(`♥ ${p[HR]} ${labels.bpm}`);
    readout.textContent = parts.join("  ·  ");
    readout.hidden = false;
  }

  function hideCursor() {
    cursorIndex = null;
    cursorMarker.remove();
    readout.hidden = true;
    const group = profileEl?.querySelector<SVGGElement>("[data-cursor]");
    if (group) group.style.display = "none";
  }

  // --- Color mode + legend

  const legendBar = $("[data-legend-bar]")!;
  const legendMin = $("[data-legend-min]")!;
  const legendMax = $("[data-legend-max]")!;
  const updateLegend = () => {
    const stops = SCALES[mode];
    const maxDomain = stops[stops.length - 1][0];
    legendBar.style.background = `linear-gradient(to right, ${stops.map(([v, c]) => `${c} ${(v / maxDomain) * 100}%`).join(", ")})`;
    if (mode === "grade") {
      legendMin.textContent = "0 %";
      legendMax.textContent = "50 %+";
    } else if (mode === "elevation") {
      legendMin.textContent = formatElevation(range.ele[0], locale);
      legendMax.textContent = formatElevation(range.ele[1], locale);
    } else {
      legendMin.textContent = `${range.hr[0]} ${labels.bpm}`;
      legendMax.textContent = `${range.hr[1]} ${labels.bpm}`;
    }
  };
  updateLegend();

  root.querySelectorAll<HTMLButtonElement>("[data-mode]").forEach((button) => {
    button.addEventListener("click", () => {
      mode = button.dataset.mode as Mode;
      root.querySelectorAll<HTMLButtonElement>("[data-mode]").forEach((b) => {
        const active = b === button;
        b.classList.toggle("btn-primary", active);
        b.classList.toggle("btn-ghost", !active);
        b.setAttribute("aria-pressed", String(active));
      });
      drawRoute();
      drawProfile();
      updateLegend();
    });
  });

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
    playButton?.setAttribute("aria-label", labels.play);
    playButton?.setAttribute("title", labels.play);
  };

  const step = (now: number) => {
    replayDistance += ((now - lastTime) / REPLAY_MS) * total;
    lastTime = now;
    const i = indexAtDistance(points, Math.min(replayDistance, total));
    setCursor(i);
    if (!map.getBounds().pad(-0.2).contains(latlngs[i])) map.panTo(latlngs[i], { animate: true, duration: 0.6 });
    if (replayDistance >= total) return stopReplay();
    frame = requestAnimationFrame(step);
  };

  playButton?.addEventListener("click", () => {
    if (playing) return stopReplay();
    playing = true;
    root.dataset.playing = "";
    playButton.setAttribute("aria-label", labels.pause);
    playButton.setAttribute("title", labels.pause);
    replayDistance = cursorIndex != null && cursorIndex < points.length - 1 ? points[cursorIndex][DIST] : 0;
    lastTime = performance.now();
    frame = requestAnimationFrame(step);
  });

  // --- Buttons, resize, fullscreen

  $("[data-recenter]")?.addEventListener("click", fit);

  const fullscreenButton = $<HTMLButtonElement>("[data-fullscreen]");
  if (!document.fullscreenEnabled) fullscreenButton?.setAttribute("hidden", "");
  fullscreenButton?.addEventListener("click", () => {
    if (document.fullscreenElement) document.exitFullscreen();
    else root.requestFullscreen();
  });
  const onFullscreenChange = () => {
    map.invalidateSize();
    drawProfile();
  };
  document.addEventListener("fullscreenchange", onFullscreenChange);

  const resizeObserver = new ResizeObserver(() => {
    map.invalidateSize();
    drawProfile();
  });
  resizeObserver.observe(root);
  drawProfile();

  return () => {
    stopReplay();
    resizeObserver.disconnect();
    document.removeEventListener("fullscreenchange", onFullscreenChange);
    map.remove();
  };
}

// --- Helpers

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

function indexAtDistance(points: RoutePoint[], d: number) {
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

function mix(a: string, b: string, t: number) {
  const pa = parseInt(a.slice(1), 16);
  const pb = parseInt(b.slice(1), 16);
  const channel = (shift: number) => Math.round(((pa >> shift) & 255) * (1 - t) + ((pb >> shift) & 255) * t);
  return `#${((1 << 24) | (channel(16) << 16) | (channel(8) << 8) | channel(0)).toString(16).slice(1)}`;
}
