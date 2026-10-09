// Overview map on the Mountaineering page: a pin per entry and the line of every GPS route.

import L from "leaflet";
import "leaflet/dist/leaflet.css";

interface Overview {
  markers: { title: string; url: string; lat: number; lng: number; img: string }[];
  routeLines: { url: string; latlngs: [number, number][] }[];
  viewLabel: string;
}

let map: L.Map | null = null;
document.addEventListener("astro:before-swap", () => {
  map?.remove();
  map = null;
});

export function initAscentsOverviewMap() {
  const container = document.getElementById("ascents-map");
  const dataEl = document.getElementById("ascents-map-data");
  if (!container || !dataEl || map) return; // Not on page 1, or already initialized
  const { markers, routeLines, viewLabel }: Overview = JSON.parse(dataEl.textContent!);

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

  const icon = L.divIcon({
    className: "",
    html: '<div style="width:16px;height:16px;border-radius:9999px;background:#d97706;border:3px solid #fff;box-shadow:0 1px 4px rgb(0 0 0 / .5)"></div>',
    iconSize: [16, 16],
    iconAnchor: [8, 8],
  });

  routeLines.forEach((route) => {
    L.polyline(route.latlngs, { color: "#111827", weight: 6, opacity: 0.45, interactive: false }).addTo(map!);
    const line = L.polyline(route.latlngs, { color: "#f97316", weight: 3 }).addTo(map!);
    line.on("click", () => (window.location.href = route.url));
  });

  markers.forEach((m) => {
    L.marker([m.lat, m.lng], { icon })
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

  // Frame the main cluster of entries; a far-away outlier (another continent) would otherwise
  // zoom the map out to the whole world. It's still on the map, one zoom-out away.
  const points = [...markers.map((m) => L.latLng(m.lat, m.lng)), ...routeLines.flatMap((r) => r.latlngs.map((ll) => L.latLng(ll)))];
  if (points.length) {
    const median = (values: number[]) => values.sort((a, b) => a - b)[Math.floor(values.length / 2)];
    const center = L.latLng(median(points.map((p) => p.lat)), median(points.map((p) => p.lng)));
    const nearby = points.filter((p) => p.distanceTo(center) < 1_500_000);
    map.fitBounds(L.latLngBounds(nearby.length ? nearby : points), { padding: [50, 50], maxZoom: 12 });
  }
}
