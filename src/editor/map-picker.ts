// A small Leaflet map for picking a point: the entry's coordinates, or where a photo was
// taken. Draws the post's route when there is one, so a point can be placed against it.
import type { Map as LeafletMap, Marker, Polyline } from "leaflet";
import { el } from "./dom";

export interface PickerOptions {
  lat?: number | null;
  lon?: number | null;
  /** [lat, lon] pairs of the route, drawn as a line */
  track?: [number, number][];
  zoom?: number;
  onPick(lat: number, lon: number): void;
}

/** Returns the container to put in the page; the map initializes itself once it is visible. */
export function mapPicker(options: PickerOptions) {
  const node = el("div", { class: "ed-map" });
  void init(node, options);
  return node;
}

async function init(node: HTMLElement, options: PickerOptions) {
  const L = (await import("leaflet")).default;
  await import("leaflet/dist/leaflet.css");

  const start: [number, number] = [options.lat ?? options.track?.[0]?.[0] ?? -33.45, options.lon ?? options.track?.[0]?.[1] ?? -70.65];
  const map: LeafletMap = L.map(node, { scrollWheelZoom: true, attributionControl: false }).setView(start, options.zoom ?? (options.lat != null ? 12 : 8));
  L.tileLayer("https://tile.opentopomap.org/{z}/{x}/{y}.png", { maxZoom: 17, attribution: "© OpenTopoMap" }).addTo(map);

  let line: Polyline | null = null;
  if (options.track?.length) {
    line = L.polyline(options.track, { color: "#f97316", weight: 3 }).addTo(map);
    map.fitBounds(line.getBounds(), { padding: [20, 20] });
  }

  const icon = L.divIcon({ className: "ed-map__pin", html: "<span></span>", iconSize: [18, 18], iconAnchor: [9, 9] });
  let marker: Marker | null = options.lat != null && options.lon != null ? L.marker([options.lat, options.lon], { icon, draggable: true }).addTo(map) : null;

  const report = (lat: number, lon: number) => options.onPick(round(lat), round(lon));
  marker?.on("dragend", () => {
    const { lat, lng } = marker!.getLatLng();
    report(lat, lng);
  });

  map.on("click", (event) => {
    const { lat, lng } = (event as { latlng: { lat: number; lng: number } }).latlng;
    if (marker) marker.setLatLng([lat, lng]);
    else {
      marker = L.marker([lat, lng], { icon, draggable: true }).addTo(map);
      marker.on("dragend", () => {
        const { lat: a, lng: b } = marker!.getLatLng();
        report(a, b);
      });
    }
    report(lat, lng);
  });

  // The map is often built inside a tab that was hidden a moment ago.
  new ResizeObserver(() => map.invalidateSize()).observe(node);
}

const round = (value: number) => Math.round(value * 1e5) / 1e5;
