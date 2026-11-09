// The map of which of my summits can see each other (src/pages/[...lang]/ascents/horizon.astro).
// Everything is drawn thin and quiet; choosing a summit, on the map or in the list beside it,
// lights up its own lines and leaves the rest as background.

import L from "leaflet";
import "leaflet/dist/leaflet.css";

interface Node { id: string; name: string; ele: number; lat: number; lon: number; url: string; degree: number }
interface Edge { a: string; b: string; status: "visible" | "borderline"; distance: number }
interface Data { nodes: Node[]; edges: Edge[]; labels: { detail: string; rest: string; line: string } }

const SERIES = "#2a78d6";
let map: L.Map | null = null;
document.addEventListener("astro:before-swap", () => {
  map?.remove();
  map = null;
});

export function initHorizonMap() {
  const container = document.getElementById("horizon-map");
  const dataEl = document.getElementById("horizon-map-data");
  if (!container || !dataEl || map) return;
  const { nodes, edges, labels }: Data = JSON.parse(dataEl.textContent!);
  const byId = new Map(nodes.map((node) => [node.id, node]));

  map = L.map(container, { scrollWheelZoom: false });
  map.on("click", () => map?.scrollWheelZoom.enable());
  container.addEventListener("mouseleave", () => map?.scrollWheelZoom.disable());
  L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Elevation/World_Hillshade/MapServer/tile/{z}/{y}/{x}", { maxZoom: 16, attribution: "Hillshade © Esri" }).addTo(map);

  const lines = edges.map((edge) => {
    const [a, b] = [byId.get(edge.a)!, byId.get(edge.b)!];
    const line = L.polyline([[a.lat, a.lon], [b.lat, b.lon]], { color: SERIES, weight: 1.75, opacity: 0.6, dashArray: edge.status === "borderline" ? "4 6" : undefined }).addTo(map!);
    line.bindTooltip(labels.line.replace("{a}", a.name).replace("{b}", b.name).replace("{km}", String(Math.round(edge.distance))), { sticky: true });
    return { edge, line };
  });

  const markers = new Map(
    nodes.map((node) => {
      const marker = L.circleMarker([node.lat, node.lon], { radius: 5 + node.degree * 0.45, color: "#fff", weight: 2, fillColor: SERIES, fillOpacity: 1 }).addTo(map!);
      marker.bindTooltip(`${node.name} · ${node.ele.toLocaleString(document.documentElement.lang)} m`);
      marker.on("click", (event) => {
        L.DomEvent.stopPropagation(event);
        select(selected === node.id ? null : node.id);
      });
      return [node.id, marker] as const;
    }),
  );

  map.fitBounds(L.latLngBounds(nodes.map((node) => [node.lat, node.lon])), { padding: [30, 30], maxZoom: 11 });

  const buttons = [...document.querySelectorAll<HTMLButtonElement>("[data-node]")];
  const detail = document.querySelector<HTMLElement>("[data-hz-detail]")!;
  let selected: string | null = null;

  function select(id: string | null) {
    selected = id;
    for (const button of buttons) button.setAttribute("aria-pressed", String(button.dataset.node === id));
    for (const { edge, line } of lines) {
      const mine = id != null && (edge.a === id || edge.b === id);
      line.setStyle({ weight: mine ? 3.5 : 1.75, opacity: id == null ? 0.6 : mine ? 1 : 0.1 });
      if (mine) line.bringToFront();
    }
    for (const [nodeId, marker] of markers) {
      const linked = id == null || nodeId === id || edges.some((edge) => (edge.a === id && edge.b === nodeId) || (edge.b === id && edge.a === nodeId));
      marker.setStyle({ fillOpacity: linked ? 1 : 0.25, opacity: linked ? 1 : 0.4 });
      marker.bringToFront();
    }
    if (id == null) {
      detail.textContent = labels.rest;
      return;
    }
    const others = edges.filter((edge) => edge.a === id || edge.b === id).map((edge) => byId.get(edge.a === id ? edge.b : edge.a)!);
    detail.textContent = labels.detail
      .replace("{name}", byId.get(id)!.name)
      .replace("{n}", String(others.length))
      .replace("{list}", others.sort((a, b) => b.ele - a.ele).map((node) => node.name).join(", "));
  }

  for (const button of buttons) {
    button.addEventListener("click", () => select(selected === button.dataset.node ? null : button.dataset.node!));
  }
}
