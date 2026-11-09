// Overview map on the Mountaineering page: a pin per entry and the line of every GPS route.

import L from "leaflet";
import "leaflet/dist/leaflet.css";

interface Overview {
  markers: { title: string; url: string; lat: number; lng: number; img: string; attempt: boolean }[];
  routeLines: { url: string; title: string; latlngs: [number, number][] }[];
  viewLabel: string;
  resetLabel: string;
}

/** Entries farther apart than this are separate places: one map frame can't show them both. */
const CLUSTER_RADIUS_M = 300_000;

let map: L.Map | null = null;
document.addEventListener("astro:before-swap", () => {
  map?.remove();
  map = null;
});

export function initAscentsOverviewMap() {
  const container = document.getElementById("ascents-map");
  const dataEl = document.getElementById("ascents-map-data");
  if (!container || !dataEl || map) return; // Not on page 1, or already initialized
  const { markers, routeLines, viewLabel, resetLabel }: Overview = JSON.parse(dataEl.textContent!);

  map = L.map(container, { scrollWheelZoom: false }).setView([-33.4, -70.5], 7); // Roughly Santiago
  map.on("click", () => map?.scrollWheelZoom.enable());
  container.addEventListener("mouseleave", () => map?.scrollWheelZoom.disable());

  const topo = L.tileLayer("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", {
    maxZoom: 17,
    attribution: 'Map data © <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>, SRTM · Style © <a href="https://opentopomap.org">OpenTopoMap</a>',
  }).addTo(map);
  const streets = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '© <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  });
  L.control.layers({ Topo: topo, OSM: streets }).addTo(map);

  const pin = (attempt: boolean) =>
    L.divIcon({
      className: "",
      html: `<div style="width:16px;height:16px;border-radius:9999px;background:${attempt ? "#fff" : "#d97706"};border:3px solid ${attempt ? "#d97706" : "#fff"};box-shadow:0 1px 4px rgb(0 0 0 / .5)"></div>`,
      iconSize: [16, 16],
      iconAnchor: [8, 8],
    });

  routeLines.forEach((route) => {
    L.polyline(route.latlngs, { color: "#111827", weight: 6, opacity: 0.45, interactive: false }).addTo(map!);
    const line = L.polyline(route.latlngs, { color: "#f97316", weight: 3 }).addTo(map!);
    line.bindTooltip(route.title, { sticky: true });
    line.on("mouseover", () => line.setStyle({ weight: 6 }));
    line.on("mouseout", () => line.setStyle({ weight: 3 }));
    line.on("click", () => (window.location.href = route.url));
  });

  markers.forEach((m) => {
    L.marker([m.lat, m.lng], { icon: pin(m.attempt) })
      .addTo(map!)
      .bindPopup(
        `<div style="text-align:center; min-width: 150px; font-family: sans-serif;">
          <h3 style="font-weight:bold; margin-bottom:8px; font-size:1.1em;">${m.title}</h3>
          <a href="${m.url}" style="text-decoration:none;">
            <img src="${m.img}" alt="" style="width:100%; height:100px; object-fit:cover; border-radius:5px; margin-bottom:8px;" />
            <div style="width:100%; padding:6px; background:#4a00ff; color:white; border-radius:4px; font-weight:bold;">${viewLabel}</div>
          </a>
        </div>`,
      );
  });

  // Group the entries into places. The biggest group is what the map opens on; the others (the
  // south of Chile, another continent) get a small map of their own in the corner, because
  // framing them all together is how fifteen trips turn into one orange smudge.
  const places: { points: L.LatLng[]; names: string[] }[] = [];
  const add = (point: L.LatLng, name?: string) => {
    let place = places.find((p) => p.points[0].distanceTo(point) < CLUSTER_RADIUS_M);
    if (!place) places.push((place = { points: [], names: [] }));
    place.points.push(point);
    if (name) place.names.push(name);
  };
  for (const m of markers) add(L.latLng(m.lat, m.lng), m.title.split(" · ")[0]);
  for (const route of routeLines) for (const ll of route.latlngs) add(L.latLng(ll));
  if (!places.length) return;
  places.sort((a, b) => b.names.length - a.names.length || b.points.length - a.points.length);

  const home = L.latLngBounds(places[0].points);
  const padding: L.PointExpression = [36, 36];
  map.fitBounds(home, { padding, maxZoom: 12 });

  const others = places.slice(1).filter((place) => place.names.length);
  if (!others.length) return;
  const box = L.DomUtil.create("div", "map-insets", container);
  L.DomEvent.disableClickPropagation(box);
  L.DomEvent.disableScrollPropagation(box);

  const reset = L.DomUtil.create("button", "map-reset", container) as HTMLButtonElement;
  reset.type = "button";
  reset.textContent = resetLabel;
  reset.hidden = true;
  L.DomEvent.disableClickPropagation(reset);
  reset.addEventListener("click", () => {
    map?.flyToBounds(home, { padding, maxZoom: 12 });
    reset.hidden = true;
  });

  for (const place of others) {
    const bounds = L.latLngBounds(place.points);
    const inset = L.DomUtil.create("button", "map-inset", box) as HTMLButtonElement;
    inset.type = "button";
    inset.title = place.names.join(", ");
    const small = L.DomUtil.create("div", "map-inset__map", inset);
    L.DomUtil.create("span", "map-inset__name", inset).textContent = place.names[0].replace(/^(Cerro|Volcán) /, "");
    const mini = L.map(small, { zoomControl: false, attributionControl: false, dragging: false, scrollWheelZoom: false, doubleClickZoom: false, touchZoom: false, boxZoom: false, keyboard: false });
    L.tileLayer("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", { maxZoom: 17 }).addTo(mini);
    mini.fitBounds(bounds, { maxZoom: 10 });
    L.circleMarker(bounds.getCenter(), { radius: 5, color: "#fff", weight: 2, fillColor: "#d97706", fillOpacity: 1 }).addTo(mini);
    inset.addEventListener("click", () => {
      map?.flyToBounds(bounds, { padding, maxZoom: 12 });
      reset.hidden = false;
    });
  }
}
