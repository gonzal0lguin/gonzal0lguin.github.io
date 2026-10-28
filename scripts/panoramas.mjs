// The view from every summit of the log: the same computation as the hill's panorama
// (scripts/lib/viewshed.mjs) run from the top of each summit in src/content/summits/ that has
// `panorama: true`, plus what is worth telling about each view — the glaciers in sight, the
// towns and lakes, the sea if it shows, and which of my other summits can be seen from there.
//
// For each summit it writes:
//
//   src/data/panoramas/<id>.json            everything the summit page draws
//   public/assets/panoramas/<id>/terrain.png the polar terrain mesh for the 3D view
//   public/assets/panoramas/<id>/viewshed.png what can be seen, with the glacier ice, as a map overlay
//
// Named summits, glaciers, towns and lakes come from OpenStreetMap, fetched once per region
// through Overpass and cached in scripts/data/osm/ (delete a region's files to refresh them).
//
// Run with:
//   npm run panoramas               summits that don't have a panorama yet
//   npm run panoramas -- provincia  just these (recomputed)
//   npm run panoramas -- --all      every summit with `panorama: true`

import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { loadDem } from "./lib/dem.mjs";
import { overpass } from "./lib/overpass.mjs";
import { linkRegistry, readAscents, readSummits, readTracks, registryMatcher } from "./lib/ascents.mjs";
import {
  BANDS, METHOD, RADIUS, RAD, RAY_STEP, angleOf, bearingDistance, castRays, meshInfo, peaksInView, polyline,
  ridgeLines, round, routesInView, snapToTop, terrainPng, viewBox, viewStats, viewshedPng,
} from "./lib/viewshed.mjs";

const OUT_DATA = new URL("../src/data/panoramas/", import.meta.url);
const OUT_ASSETS = new URL("../public/assets/panoramas/", import.meta.url);
const OSM = new URL("./data/osm/", import.meta.url);

/** Hidden summits this high (with a Wikidata entry, so they are famous) are drawn as ghosts. */
const FAMOUS = 5000;
/** The top is somewhere you walk around: its own ground, this close, hides nothing (see castRays). */
const OCCLUDE_FROM = 300;
/** A glacier counts as in view when at least this much of it shows, in km². */
const GLACIER_SEEN = 0.02;

// ------------------------------------------------------------------------------ regions

/**
 * OpenStreetMap data for a box: one Overpass query per kind of thing, for a whole region at
 * once, so neighbouring summits share it. scripts/data/osm/regions.json lists the regions.
 */
async function regionData(box) {
  const index = new URL("regions.json", OSM);
  const regions = existsSync(index) ? JSON.parse(readFileSync(index, "utf8")) : {};
  let name = Object.keys(regions).find((key) => {
    const [s, w, n, e] = regions[key];
    return s <= box.south && w <= box.west && n >= box.north && e >= box.east;
  });
  if (!name) {
    const bbox = [Math.floor(box.south * 2) / 2, Math.floor(box.west * 2) / 2, Math.ceil(box.north * 2) / 2, Math.ceil(box.east * 2) / 2];
    name = `region_${bbox.join("_")}`;
    regions[name] = bbox;
    writeFileSync(index, JSON.stringify(regions, null, 2) + "\n");
  }
  const b = `(${regions[name].join(",")})`;
  const water = `[natural=water][name][water~"^(lake|reservoir|lagoon)$"]`;
  const queries = {
    peaks: `node[natural~"^(peak|volcano|hill)$"][name]${b};out body;`,
    places: `node[place~"^(city|town)$"][name]${b};out body;`,
    lakes: `(way${water}${b};relation${water}${b};);out tags bb;`,
    glaciers: `(way[natural=glacier]${b};relation[natural=glacier]${b};);out geom;`,
  };
  const out = {};
  for (const [kind, query] of Object.entries(queries)) out[kind] = (await overpass(query, new URL(`${name}-${kind}.json`, OSM))).elements;
  return out;
}

// ------------------------------------------------------------------------------ glaciers

/** Closed rings from the member ways of a multipolygon, joining ways that share an end. */
function assembleRings(ways) {
  const same = (a, b) => a.lat === b.lat && a.lon === b.lon;
  const open = ways.map((way) => [...way]);
  const rings = [];
  while (open.length) {
    let ring = open.shift();
    let grown = true;
    while (!same(ring[0], ring[ring.length - 1]) && grown) {
      grown = false;
      for (let i = 0; i < open.length; i++) {
        const way = open[i];
        const end = ring[ring.length - 1];
        if (same(end, way[0])) ring = ring.concat(way.slice(1));
        else if (same(end, way[way.length - 1])) ring = ring.concat([...way].reverse().slice(1));
        else continue;
        open.splice(i, 1);
        grown = true;
        break;
      }
    }
    rings.push(ring);
  }
  return rings;
}

/**
 * Every glacier polygon burnt into a grid of 3″ cells (about 90 m), so the rays can ask "is this
 * ice, and whose?" for every sample in view. Rings are filled even-odd, which leaves the holes.
 */
function glacierGrid(elements, box) {
  const PER = 1200;
  const rows = Math.ceil((box.north - box.south) * PER);
  const cols = Math.ceil((box.east - box.west) * PER);
  const grid = new Uint16Array(rows * cols);
  const glaciers = [];
  for (const element of elements) {
    if (glaciers.length >= 65_534) break;
    const rings = (element.type === "way" ? [element.geometry] : assembleRings(element.members.filter((m) => m.type === "way" && m.geometry).map((m) => m.geometry))).filter(
      (ring) => ring && ring.length >= 4,
    );
    if (!rings.length) continue;
    const id = glaciers.length + 1;
    const lats = rings.flat().map((p) => p.lat);
    const r0 = Math.max(0, Math.floor((box.north - Math.max(...lats)) * PER));
    const r1 = Math.min(rows - 1, Math.ceil((box.north - Math.min(...lats)) * PER));
    let cells = 0, sumLat = 0, sumLon = 0;
    for (let r = r0; r <= r1; r++) {
      const lat = box.north - (r + 0.5) / PER;
      const crossings = [];
      for (const ring of rings) {
        for (let i = 1; i < ring.length; i++) {
          const p = ring[i - 1], q = ring[i];
          if (p.lat > lat !== q.lat > lat) crossings.push(p.lon + ((lat - p.lat) * (q.lon - p.lon)) / (q.lat - p.lat));
        }
      }
      crossings.sort((a, b) => a - b);
      for (let i = 0; i + 1 < crossings.length; i += 2) {
        const c0 = Math.max(0, Math.ceil((crossings[i] - box.west) * PER - 0.5));
        const c1 = Math.min(cols - 1, Math.floor((crossings[i + 1] - box.west) * PER - 0.5));
        for (let c = c0; c <= c1; c++) {
          grid[r * cols + c] = id;
          cells++;
          sumLat += lat;
          sumLon += box.west + (c + 0.5) / PER;
        }
      }
    }
    if (!cells) continue;
    const lat = sumLat / cells;
    const cell = ((6371008.8 * RAD) / PER) ** 2 * Math.cos(lat * RAD);
    glaciers.push({
      id,
      name: element.tags?.name ?? null,
      type: element.tags?.["glacier:type"] ?? null,
      lat: round(lat, 4),
      lon: round(sumLon / cells, 4),
      area: (cells * cell) / 1e6,
      visibleArea: 0,
      best: null,
    });
  }
  const at = (lat, lon) => {
    const r = Math.floor((box.north - lat) * PER);
    const c = Math.floor((lon - box.west) * PER);
    return r < 0 || c < 0 || r >= rows || c >= cols ? 0 : grid[r * cols + c];
  };
  return { glaciers, at };
}

// ------------------------------------------------------------------------------ one summit

async function panorama(entry, registry, ascents, tracks) {
  console.log(`\n=== ${entry.name}`);
  const box = viewBox(entry);
  console.time("dem");
  const dem = await loadDem(box);
  console.timeEnd("dem");
  // The registry's point is a map's; the top of the model can be a cell or two away.
  const summit = snapToTop(dem, entry, 120);
  console.log(`summit ${summit.lat.toFixed(5)}, ${summit.lon.toFixed(5)} at ${summit.elevation} m (registry ${entry.elevation} m)`);

  const osm = await regionData(box);
  const ice = glacierGrid(osm.glaciers, box);
  const eye = summit.elevation + 1.7;
  const view = castRays(dem, summit, {
    occludeFrom: OCCLUDE_FROM,
    onVisible(lat, lon, h, d, area, k) {
      const id = ice.at(lat, lon);
      if (!id) return;
      const glacier = ice.glaciers[id - 1];
      glacier.visibleArea += area / 1e6;
      const angle = angleOf(h, d, eye);
      if (!glacier.best || angle > glacier.best.angle) glacier.best = { azimuth: k * RAY_STEP, angle, distance: d };
    },
  });
  const ridges = ridgeLines(view);

  // Summits: mine always, and the famous high ones even when hidden ("can you see Aconcagua?").
  const mine = registryMatcher(registry, bearingDistance);
  const peaks = peaksInView(
    view,
    osm.peaks.map((node) => ({ id: node.id, name: node.tags.name, lat: node.lat, lon: node.lon, ele: node.tags.ele, kind: node.tags.natural, wikidata: node.tags.wikidata })),
    { isNamed: (peak, nearestOfName) => mine(peak) || (peak.wikidata && (peak.ele ?? peak.model) >= FAMOUS && nearestOfName.get(peak.name) === peak) },
  );
  linkRegistry(peaks, registry, ascents, bearingDistance);
  const routes = routesInView(view, tracks, peaks);

  // Glaciers: within reach, and how much of each shows.
  const glaciers = ice.glaciers.filter((g) => bearingDistance(summit, g).distance < RADIUS);
  const seenGlaciers = glaciers.filter((g) => g.visibleArea >= GLACIER_SEEN).sort((a, b) => b.visibleArea - a.visibleArea);
  const glacierSummary = {
    count: glaciers.length,
    area: round(glaciers.reduce((sum, g) => sum + g.area, 0), 1),
    visibleCount: seenGlaciers.length,
    visibleArea: round(seenGlaciers.reduce((sum, g) => sum + g.visibleArea, 0), 1),
    list: seenGlaciers.slice(0, 40).map((g) => ({
      name: g.name,
      type: g.type,
      lat: g.lat,
      lon: g.lon,
      area: round(g.area, 2),
      visibleArea: round(g.visibleArea, 2),
      distance: round(g.best.distance / 1000, 1),
      azimuth: round(g.best.azimuth, 2),
      angle: round(g.best.angle, 3),
    })),
  };
  console.log(`glaciers: ${glacierSummary.visibleCount} of ${glacierSummary.count} in view, ${glacierSummary.visibleArea} of ${glacierSummary.area} km²`);

  // Towns: in view if a point 10 m above the ground clears what stands in front.
  const places = osm.places
    .map((node) => ({ node, ...bearingDistance(summit, node) }))
    .filter(({ distance }) => distance < RADIUS && distance > 1000)
    .map(({ node, azimuth, distance }) => {
      const seen = view.sight(node, dem.elevation(node.lat, node.lon) + 10);
      return {
        name: node.tags.name,
        kind: node.tags.place,
        population: Number(String(node.tags.population ?? "").replace(/\D/g, "")) || null,
        distance: round(distance / 1000, 1),
        azimuth: round(azimuth, 1),
        visible: seen.margin > 0,
      };
    })
    .sort((a, b) => (b.population ?? 0) - (a.population ?? 0));

  // Lakes big enough to make out from a summit (about 400 m across), once per name.
  const across = ({ bounds: b }) => (b ? Math.min((b.maxlat - b.minlat) * 111_195, (b.maxlon - b.minlon) * 111_195 * Math.cos(b.minlat * RAD)) : 0);
  const named = new Set();
  const lakes = osm.lakes
    .filter((element) => element.bounds && across(element) >= 400 && !named.has(element.tags.name) && named.add(element.tags.name))
    .map((element) => ({ ...element, center: { lat: (element.bounds.minlat + element.bounds.maxlat) / 2, lon: (element.bounds.minlon + element.bounds.maxlon) / 2 } }))
    .map((element) => ({ element, ...bearingDistance(summit, element.center) }))
    .filter(({ distance }) => distance < RADIUS)
    .map(({ element, azimuth, distance }) => ({
      name: element.tags.name,
      kind: element.tags.water,
      distance: round(distance / 1000, 1),
      azimuth: round(azimuth, 1),
      visible: view.seen(element.center),
    }))
    .sort((a, b) => a.distance - b.distance);

  // My other summits: are they in view from here?
  const others = registry
    .filter((other) => other.id !== entry.id && bearingDistance(summit, other).distance < RADIUS - 2000)
    .map((other) => {
      const peak = peaks.find((p) => p.summit === other.id);
      const { azimuth, distance } = bearingDistance(summit, other);
      const status = peak?.status ?? (view.sight(other, dem.elevation(other.lat, other.lon)).margin > 0.06 ? "visible" : "hidden");
      return { id: other.id, name: other.name, distance: round(distance / 1000, 1), azimuth: round(azimuth, 1), status };
    })
    .sort((a, b) => a.distance - b.distance);

  // Assets.
  const assets = new URL(`${entry.id}/`, OUT_ASSETS);
  mkdirSync(assets, { recursive: true });
  writeFileSync(new URL("terrain.png", assets), terrainPng(view));
  const shed = viewshedPng(view, { ice: (lat, lon) => ice.at(lat, lon) > 0 });
  writeFileSync(new URL("viewshed.png", assets), shed.file);

  const stats = viewStats(view);
  const inView = peaks.filter((p) => p.status !== "hidden");
  const highest = [...inView].sort((a, b) => (b.ele ?? b.model) - (a.ele ?? a.model))[0];
  const farthestPeak = [...inView].sort((a, b) => b.distance - a.distance)[0];
  const brief = (p) => p && { name: p.name, ele: p.ele ?? p.model, distance: p.distance, azimuth: p.azimuth };
  Object.assign(stats, {
    peaksInView: inView.length,
    peaksAround: osm.peaks.filter((node) => bearingDistance(summit, node).distance < RADIUS).length,
    highest: brief(highest),
    farthestPeak: brief(farthestPeak),
    sea: view.stats.sea && { distance: round(view.stats.sea.distance / 1000, 1), azimuth: round(view.stats.sea.azimuth, 1) },
  });

  mkdirSync(OUT_DATA, { recursive: true });
  const base = `/assets/panoramas/${entry.id}`;
  writeFileSync(
    new URL(`${entry.id}.json`, OUT_DATA),
    JSON.stringify({
      source: "Copernicus GLO-30 DEM (© DLR e.V. 2010–2014 and © Airbus 2014–2018, provided under COPERNICUS by the European Union and ESA); summits, glaciers, towns and lakes © OpenStreetMap contributors",
      generated: new Date().toISOString().slice(0, 10),
      method: METHOD,
      summit: { id: entry.id, name: entry.name },
      viewpoint: { lat: round(summit.lat, 5), lon: round(summit.lon, 5), elevation: Math.round(summit.elevation), eye: round(view.eyeHeight, 1) },
      stats,
      bands: BANDS,
      skyline: polyline(view.skyline),
      bandSkylines: view.bandSkylines.map(polyline),
      ridges,
      peaks,
      routes,
      glaciers: glacierSummary,
      places,
      lakes,
      mine: others,
      mesh: meshInfo(`${base}/terrain.png`),
      viewshed: { ...shed.bounds, file: `${base}/viewshed.png` },
    }),
  );
  console.log(
    `${entry.id}.json: ${ridges.length} ridges, ${inView.length} summits in view, highest ${highest?.name}; ` +
      `towns in view: ${places.filter((p) => p.visible).map((p) => p.name).slice(0, 8).join(", ") || "—"}; ` +
      `mine in view: ${others.filter((o) => o.status !== "hidden").map((o) => o.name).join(", ") || "—"}`,
  );
}

// ------------------------------------------------------------------------------ run

const args = process.argv.slice(2);
const registry = readSummits();
const ascents = readAscents();
const tracks = readTracks();
const wanted = registry.filter((entry) => {
  if (args.length && !args.includes("--all")) return args.includes(entry.id);
  return entry.panorama && (args.includes("--all") || !existsSync(new URL(`${entry.id}.json`, OUT_DATA)));
});
if (!wanted.length) console.log("Every summit with `panorama: true` already has one (use --all or name them to recompute).");
for (const entry of wanted) {
  if (entry.lat > 0 || entry.lon > 0) {
    console.log(`\n=== ${entry.name}: skipped — the elevation reader only knows the southern and western hemispheres`);
    continue;
  }
  await panorama(entry, registry, ascents, tracks);
}
