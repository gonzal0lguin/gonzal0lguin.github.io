// The outline of every summit of the log, for the "my summits to scale" chart: a slice of the
// elevation model (scripts/lib/dem.mjs) through each top, drawn at true scale next to the others.
//
// It writes src/data/profiles.json: per summit, the bearing it is seen from and the outline as
// [metres across, metres high] pairs, centred on the top.
//
// Why a slice and not the view from somewhere. A distant silhouette — the highest ground along
// every line of sight — is what a mountain really looks like from afar, but in the Andes almost
// every summit has higher ground within a few kilometres, so that silhouette is the range behind
// it, not the mountain. A view from the trailhead is personal but in perspective: the foothills
// you start under come out bigger than the summit, and half the log has no track anyway. So the
// outline is the highest ground in a thin band (DEPTH either side) across the top, which keeps
// the mountain's own shoulders and drops whatever stands behind it.
//
// Which slice. Every bearing is tried and the one kept is where the top stands out most: the
// ground falls away on both sides (scored by the weaker side, so a one-sided shoulder loses to a
// real peak) and nothing in the slice rises above the top. A slice reads the same from either
// side, mirrored; it is shown from the side the GPS track starts on when there is one, so it is
// roughly the mountain I walked up to, and otherwise from the north, the sunny side down here.
//
// Run with: npm run profiles   (a second or two per summit once the tiles are cached)

import { writeFileSync } from "node:fs";
import { loadDem } from "./lib/dem.mjs";
import { readSummits, readTracks } from "./lib/ascents.mjs";
import { RAD, bearingDistance, round, simplify, snapToTop } from "./lib/viewshed.mjs";

const OUT = new URL("../src/data/profiles.json", import.meta.url);

/** Half the width of the outline, m. The chart may show less of it. */
const HALF_WIDTH = 4000;
/** Half the thickness of the slice, m: enough to smooth over a gully, not to reach the next peak. */
const DEPTH = 400;
/** Sampling step along and across the slice, m — the model's own resolution. */
const STEP = 30;
/** Bearings tried, degrees apart. A slice and its opposite are the same, so 0–180 covers all. */
const AZ_STEP = 5;
/** Ground in the slice above the top costs this many times its excess, so the top stays the top. */
const OVERTOP = 3;
/** Simplification of the outline, m: well under a pixel at any size the chart is drawn. */
const TOLERANCE = 4;

const M_PER_DEG = 111_320;

/** The outline across `top` along bearing `azimuth` + 90°, as heights every STEP metres. */
function slice(dem, top, azimuth) {
  const t = azimuth * RAD;
  const toLat = 1 / M_PER_DEG;
  const toLon = 1 / (M_PER_DEG * Math.cos(top.lat * RAD));
  const heights = [];
  for (let u = -HALF_WIDTH; u <= HALF_WIDTH; u += STEP) {
    let highest = 0;
    for (let d = -DEPTH; d <= DEPTH; d += STEP) {
      // u runs to the viewer's right, d away from them, for a viewer looking along `azimuth`.
      const east = u * Math.cos(t) + d * Math.sin(t);
      const north = -u * Math.sin(t) + d * Math.cos(t);
      highest = Math.max(highest, dem.elevation(top.lat + north * toLat, top.lon + east * toLon));
    }
    heights.push(highest);
  }
  return heights;
}

function score(heights, top) {
  const mid = (heights.length - 1) / 2;
  const fall = (from, to) => heights.slice(from, to).reduce((sum, h) => sum + top - h, 0) / (to - from);
  const over = Math.max(0, ...heights.map((h) => h - top));
  return Math.min(fall(0, mid), fall(mid + 1, heights.length)) - OVERTOP * over;
}

const angleBetween = (a, b) => Math.abs(((a - b + 540) % 360) - 180);

const summits = readSummits();
const tracks = readTracks();
const out = {};

for (const summit of summits) {
  // Where the first track up this summit starts, if any.
  const track = tracks.find((track) => track.summits.includes(summit.id));
  const start = track?.segments[0]?.[0];
  const reach = (Math.hypot(HALF_WIDTH, DEPTH) + 500) / M_PER_DEG;
  const dem = await loadDem({
    south: summit.lat - reach,
    north: summit.lat + reach,
    west: summit.lon - reach / Math.cos(summit.lat * RAD),
    east: summit.lon + reach / Math.cos(summit.lat * RAD),
  });
  if (!dem.tiles) {
    console.warn(`${summit.id}: no elevation model here, skipped`);
    continue;
  }
  const top = snapToTop(dem, summit, 200);

  let best = null;
  for (let azimuth = 0; azimuth < 180; azimuth += AZ_STEP) {
    const heights = slice(dem, top, azimuth);
    const value = score(heights, top.elevation);
    if (!best || value > best.value) best = { azimuth, heights, value };
  }

  // The viewer of a slice looking along `azimuth` stands on its far side, at azimuth + 180°.
  let from = (best.azimuth + 180) % 360;
  let heights = best.heights;
  const wanted = start ? bearingDistance(top, start).azimuth : 0;
  if (angleBetween(from, wanted) > 90) {
    from = best.azimuth;
    heights = [...heights].reverse();
  }

  const points = simplify(heights.map((h, i) => [-HALF_WIDTH + i * STEP, h]), TOLERANCE);
  out[summit.id] = {
    from: Math.round(from),
    side: start ? "track" : "north",
    top: round(top.elevation, 1),
    points: points.map(([u, h]) => [u, Math.round(h)]),
  };
  console.log(`${summit.id.padEnd(16)} seen from ${String(Math.round(from)).padStart(3)}° (${out[summit.id].side}), ${points.length} points`);
}

writeFileSync(OUT, JSON.stringify(out) + "\n");
console.log(`wrote ${Object.keys(out).length} outlines to src/data/profiles.json`);
