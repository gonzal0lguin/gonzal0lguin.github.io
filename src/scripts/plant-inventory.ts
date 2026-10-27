// Brings the digital inventory to life: the Leaflet map of every plant whose position we
// took, and the filters over the full list. The markup and every number come from
// src/components/PlantInventory.astro, rendered at build time.

import L from "leaflet";
import "leaflet/dist/leaflet.css";

interface MapPlant {
  id: string;
  /** Tag prefix, shown inside the marker: QY, LT, HG… */
  code: string;
  name: string;
  scientific: string;
  habit: "tree" | "shrub";
  status: string;
  statusLabel: string;
  statusGlyph: string;
  planted: string;
  notes: string | null;
  lat: number;
  lon: number;
}

let map: L.Map | null = null;
document.addEventListener("astro:before-swap", () => {
  map?.remove();
  map = null;
});

const esc = (value: unknown) =>
  String(value).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

export function initPlantInventory() {
  initMap();
  initFilters();
}

function initMap() {
  const container = document.getElementById("inventory-map");
  const data = document.getElementById("inventory-map-data");
  if (!container || !data || map) return;
  const plants: MapPlant[] = JSON.parse(data.textContent!);
  if (!plants.length) return;

  map = L.map(container, { scrollWheelZoom: false });
  // The plants sit in a cluster some 60 m across, so we have to zoom past what the imagery
  // actually holds: free aerial coverage of this corner of Calle Larga stops at zoom 18
  // (≈0.6 m per pixel) and the topo layer at 17. Both layers declare that honestly through
  // maxNativeZoom, so Leaflet upscales the last real tile instead of asking the server for
  // one that does not exist and painting a "map data not yet available" placeholder.
  map.on("click", () => map?.scrollWheelZoom.enable());
  container.addEventListener("mouseleave", () => map?.scrollWheelZoom.disable());

  const satellite = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
    maxZoom: 20,
    maxNativeZoom: 18,
    attribution: "Imagery © Esri, Maxar, Earthstar Geographics",
  }).addTo(map);
  const topo = L.tileLayer("https://{s}.tile.opentopomap.org/{z}/{x}/{y}.png", {
    maxZoom: 20,
    maxNativeZoom: 17,
    attribution:
      'Map data © <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>, SRTM · Style © <a href="https://opentopomap.org">OpenTopoMap</a>',
  });
  L.control.layers({ Satellite: satellite, Topo: topo }).addTo(map);
  L.control.scale({ imperial: false }).addTo(map);

  for (const plant of plants) {
    const icon = L.divIcon({
      className: "",
      html: `<div class="iv-pin iv-pin--${esc(plant.habit)} iv-pin--${esc(plant.status)}">${esc(plant.code)}</div>`,
      iconSize: [26, 26],
      iconAnchor: [13, 13],
    });
    L.marker([plant.lat, plant.lon], { icon, title: `${plant.id} · ${plant.name}` })
      .addTo(map)
      .bindPopup(
        `<div class="iv-popup">
          <h4>${esc(plant.id)} · ${esc(plant.name)}</h4>
          <em>${esc(plant.scientific)}</em>
          <dl>
            <dt>${esc(plant.statusGlyph)}</dt><dd>${esc(plant.statusLabel)}</dd>
            <dt>·</dt><dd>${esc(plant.planted)}</dd>
            ${plant.notes ? `<dt>·</dt><dd>${esc(plant.notes)}</dd>` : ""}
          </dl>
        </div>`,
      );
  }

  map.fitBounds(L.latLngBounds(plants.map((plant) => L.latLng(plant.lat, plant.lon))), { padding: [40, 40], maxZoom: 20 });
}

function initFilters() {
  const filters = document.querySelector<HTMLElement>("[data-inventory-filters]");
  const body = document.querySelector<HTMLElement>("[data-inventory-rows]");
  if (!filters || !body) return;

  const search = filters.querySelector<HTMLInputElement>("[data-filter-search]")!;
  const species = filters.querySelector<HTMLSelectElement>("[data-filter-species]")!;
  const status = filters.querySelector<HTMLSelectElement>("[data-filter-status]")!;
  const count = filters.querySelector<HTMLElement>("[data-filter-count]")!;
  const empty = document.querySelector<HTMLElement>("[data-inventory-empty]")!;
  const rows = [...body.querySelectorAll<HTMLTableRowElement>("tr")];
  // "{shown} de 44 plantas": the total is already filled in, the shown count is not.
  const template = count.dataset.template ?? "{shown}";

  const apply = () => {
    const needle = search.value.trim().toLowerCase();
    let shown = 0;
    for (const row of rows) {
      const matches =
        (!needle || (row.dataset.text ?? "").includes(needle)) &&
        (!species.value || row.dataset.species === species.value) &&
        (!status.value || row.dataset.status === status.value);
      row.hidden = !matches;
      if (matches) shown++;
    }
    count.textContent = template.replace("{shown}", String(shown));
    empty.hidden = shown > 0;
  };

  search.addEventListener("input", apply);
  species.addEventListener("change", apply);
  status.addEventListener("change", apply);
}
