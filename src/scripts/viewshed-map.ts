// The viewshed maps of src/components/ViewshedMap.astro: a grey relief, the computed viewshed
// as an image overlay (it was rasterised in Web Mercator, so it lands exactly), rings every
// 25 km and the summits worth naming. Any number per page; a map set up inside a hidden tab
// frames itself the first time it shows.

import L from "leaflet";
import "leaflet/dist/leaflet.css";

interface Data {
  viewpoint: { lat: number; lon: number; elevation: number };
  overlay: { south: number; north: number; west: number; east: number; file: string };
  peaks: { name: string; lat: number; lon: number; ele: number; distance: number; status: string; named: boolean; url: string | null }[];
  labels: Record<string, string>;
}

const maps = new Set<L.Map>();
document.addEventListener("astro:before-swap", () => {
  for (const map of maps) map.remove();
  maps.clear();
});

const esc = (value: unknown) => String(value).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

/** Sets up every viewshed map on the page (or inside `scope`) that isn't set up yet. */
export function initViewshedMap(scope: ParentNode = document) {
  for (const root of scope.querySelectorAll<HTMLElement>("[data-viewshed]:not([data-viewshed-ready])")) {
    root.dataset.viewshedReady = "";
    const container = root.querySelector<HTMLElement>("[data-viewshed-map]");
    const source = root.querySelector("[data-viewshed-data]");
    if (container && source) createMap(container, JSON.parse(source.textContent!));
  }
}

function createMap(container: HTMLElement, data: Data) {
  const { viewpoint, overlay, labels } = data;

  const map = L.map(container, { scrollWheelZoom: false, zoomSnap: 0.5 });
  maps.add(map);
  map.on("click", () => map.scrollWheelZoom.enable());
  container.addEventListener("mouseleave", () => map.scrollWheelZoom.disable());

  // A grey relief underneath, so the one colour on the map is the viewshed; place names on top.
  L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Elevation/World_Hillshade/MapServer/tile/{z}/{y}/{x}", {
    maxZoom: 16,
    attribution: "Hillshade © Esri · Viewshed: Copernicus GLO-30",
  }).addTo(map);
  L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}", {
    maxZoom: 16,
    attribution: "Places © Esri",
    pane: "shadowPane",
  }).addTo(map);

  const shed = L.imageOverlay(overlay.file, [
    [overlay.south, overlay.west],
    [overlay.north, overlay.east],
  ], { opacity: 0.9, interactive: false }).addTo(map);
  L.control.layers(undefined, { [labels.layer]: shed }, { collapsed: false }).addTo(map);
  L.control.scale({ imperial: false }).addTo(map);

  const centre = L.latLng(viewpoint.lat, viewpoint.lon);
  for (const km of [25, 50, 75]) {
    L.circle(centre, { radius: km * 1000, color: "#1e293b", weight: 1, opacity: 0.55, dashArray: "4 4", fill: false, interactive: false }).addTo(map);
    const label = L.latLng(viewpoint.lat + (km * 1000) / 111_195, viewpoint.lon);
    L.marker(label, { icon: L.divIcon({ className: "", html: `<span style="font:600 10px system-ui;color:#1e293b;text-shadow:0 0 3px #fff">${km} km</span>`, iconAnchor: [14, 6] }), interactive: false }).addTo(map);
  }

  L.circleMarker(centre, { radius: 7, color: "#fff", weight: 2, fillColor: "#d03b3b", fillOpacity: 1 })
    .addTo(map)
    // On hover (a tap on a phone) only: the red dot is plainly the viewpoint, and a standing
    // label covers the very ground the map is about.
    .bindTooltip(`${labels.summit} · ${viewpoint.elevation} m`, { direction: "right", offset: [8, 0] });

  for (const peak of data.peaks) {
    const hidden = peak.status === "hidden";
    const marker = L.circleMarker([peak.lat, peak.lon], {
      radius: 4,
      color: hidden ? "#64748b" : "#1e293b",
      weight: 1.5,
      dashArray: hidden ? "2 2" : undefined,
      fillColor: hidden ? "#ffffff" : "#2a78d6",
      fillOpacity: hidden ? 0.6 : 1,
    }).addTo(map);
    const name = peak.name.replace(/^(Cerro|Volcán) /, "");
    // Only the summits the post names keep a label on the map; the rest show theirs on hover.
    marker.bindTooltip(name, { permanent: peak.named, direction: "right", offset: [5, 0], className: `vs-peak${hidden ? " vs-peak--hidden" : ""}` });
    marker.bindPopup(
      `<strong>${esc(peak.name)}</strong><br>${peak.ele.toLocaleString(document.documentElement.lang || "es")} m · ${peak.distance.toFixed(1)} km<br>${esc(labels[peak.status])}` +
        (peak.url ? `<br><a href="${esc(peak.url)}">${esc(labels.climbed)}</a>` : ""),
    );
  }

  const frame = () => map.fitBounds(L.latLngBounds([viewpoint.lat - 0.45, viewpoint.lon - 0.55], [viewpoint.lat + 0.45, viewpoint.lon + 0.75]));
  frame();
  if (!container.clientWidth) {
    const observer = new ResizeObserver(() => {
      if (!container.clientWidth) return;
      observer.disconnect();
      map.invalidateSize();
      frame();
    });
    observer.observe(container);
  }
}
