// The data behind the figures of the post "Mirar un cerro como lo miraría un robot": one real
// photo registered against the elevation model, and one real sight line walked sample by sample.
// Writes src/data/robot-figures.json and the post's header picture. Run it by hand, the result
// is committed: `npm run figures`.
//
// THE PHOTO. It has no GPS data and nobody recorded which way the phone was pointing, so the
// camera is found the way a robot's extrinsics are: by making the model and the picture agree.
// The ridge is read from the photo column by column (sky is warm, snow is not), the skyline is
// computed from a candidate position, and the mean vertical distance between the two is minimised
// over position, heading, tilt, roll and focal length. Nothing but that distance goes in, so the
// rest is a check on the fit: the answer lands a few tens of metres from the recorded track, with
// no roll and a focal length that is a phone's 26 mm lens.
//
// The search has to start near the answer. The ridge is only 2 km away, so 100 m to one side moves
// it by dozens of pixels, and over the whole route there are several false optima where some
// other stretch of ridge happens to fit a similar shape, some of them better than a slightly
// wrong position at the right place. HINT is where the outlines were first seen to agree.
//
// THE RAY. From La Cruz to Aconcagua, because it is the best example there is: 101 km, a mountain
// more than twice as high as the viewpoint, and it comes out within five thousandths of a degree
// of the ridge in front of it.

import { readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import sharp from "sharp";
import { loadDem } from "./lib/dem.mjs";
import { DEG, EFFECTIVE, RAD, bearingDistance, destination, simplify } from "./lib/viewshed.mjs";

const ROOT = new URL("../", import.meta.url);
const PHOTO = fileURLToPath(new URL("public/assets/img/headers/san-gabriel-hero.jpg", ROOT));
const TRACK = new URL("public/assets/lib/san-gabriel-26-06-2026.gpx", ROOT);
const HEADER = fileURLToPath(new URL("public/assets/img/headers/mirar-como-robot.jpg", ROOT));
const OUT = new URL("src/data/robot-figures.json", ROOT);

const SUMMIT = { name: "Cerro San Gabriel", lat: -33.75722, lon: -70.21472 };
const HINT = { lat: -33.77829, lon: -70.20869 };
/** Past this column the ridge in the picture is the spur the camera stands on: too near to trust. */
const LAST_COLUMN = 1880;
const EYE = 1.5;
/** A beam of the simulated scanner hits the ground this far apart, in bearing and in tilt, degrees. */
const SCAN_BEARING = 0.3;
const SCAN_TILT = 0.9;
/** Metres. Colour bands of the scan: one each below these limits and one for everything past the last. */
const RANGE_LIMITS = [700, 1100, 1500, 2000];

const RAY_FROM = { name: "Cerro La Cruz", lat: -33.47528, lon: -70.47139, elevation: 2570 };
const RAY_TO = { name: "Cerro Aconcagua", lat: -32.65333, lon: -70.01167 };
const RAY_EYE = 1.7;
const RAY_STEP = 30;
const RAY_PAST = 8000;
const RAY_BINS = 640;

const round = (value, digits = 1) => Number(value.toFixed(digits));

// ------------------------------------------------------------------------------ the photo

const { data, info } = await sharp(PHOTO).removeAlpha().raw().toBuffer({ resolveWithObject: true });
const W = info.width;
const H = info.height;

/** Snow and rock. The sky here is warm enough that its blue never reaches its green. */
const isLand = (x, y) => {
  const i = (y * W + x) * 3;
  return data[i] < 218 && data[i + 2] >= data[i + 1] - 1;
};

function readRidge() {
  const found = [];
  for (let x = 4; x < W; x += 6) {
    for (let y = 470; y < 900; y++) {
      let solid = true;
      for (let k = 0; k < 16 && solid; k++) solid = isLand(x, y + k);
      if (solid) {
        found.push([x, y]);
        break;
      }
    }
  }
  // Sunlit rock on the crest reads as sky for a few columns; a low percentile drops the notches.
  return found.map(([x], i) => {
    const near = found.slice(Math.max(0, i - 5), i + 6).map((p) => p[1]).sort((a, b) => a - b);
    return [x, near[Math.floor(near.length * 0.35)]];
  });
}

const ridge = readRidge();
const fitPoints = ridge.filter(([x], i) => i % 2 === 0 && x < LAST_COLUMN);

const xml = readFileSync(TRACK, "utf8");
const track = [...xml.matchAll(/<trkpt lat="([-\d.]+)" lon="([-\d.]+)">\s*<ele>([-\d.]+)/g)].map((m) => ({ lat: +m[1], lon: +m[2], ele: +m[3] }));
const topIndex = track.reduce((best, point, i) => (point.ele > track[best].ele ? i : best), 0);

const dem = await loadDem({ south: -34.3, north: -32.4, west: -70.7, east: -69.7 });

const toward = (from, to) => bearingDistance(from, to);
const angleTo = (h, distance, eye) => Math.atan((h - eye - (distance * distance) / (2 * EFFECTIVE)) / distance) * DEG;

function horizon(camera, center, half, step, reach) {
  const eye = dem.elevation(camera.lat, camera.lon) + EYE;
  const line = [];
  for (let azimuth = center - half; azimuth <= center + half; azimuth += step) {
    let best = -90;
    for (let d = 90; d < reach; d += d < 8000 ? 30 : 90) {
      const point = destination(camera, azimuth, d);
      best = Math.max(best, angleTo(dem.elevation(point.lat, point.lon), d, eye));
    }
    line.push([azimuth, best]);
  }
  return line;
}

/** Pinhole camera: heading, tilt and roll in degrees, focal length in pixels. Null behind it. */
function project(azimuth, elevation, camera) {
  const a = (azimuth - camera.yaw) * RAD;
  const e = elevation * RAD;
  const forward = Math.cos(a) * Math.cos(e);
  const right = Math.sin(a) * Math.cos(e);
  const up = Math.sin(e);
  const tilt = camera.pitch * RAD;
  const z = forward * Math.cos(tilt) + up * Math.sin(tilt);
  const y = -forward * Math.sin(tilt) + up * Math.cos(tilt);
  const roll = camera.roll * RAD;
  const across = right * Math.cos(roll) + y * Math.sin(roll);
  const along = -right * Math.sin(roll) + y * Math.cos(roll);
  return z > 0 ? [W / 2 + (camera.f * across) / z, H / 2 - (camera.f * along) / z] : null;
}

function residual(skyline, camera) {
  const drawn = skyline.map(([azimuth, angle]) => project(azimuth, angle, camera)).filter(Boolean);
  const gaps = [];
  let j = 0;
  for (const [u, v] of fitPoints) {
    while (j < drawn.length - 2 && drawn[j + 1][0] < u) j++;
    const a = drawn[j];
    const b = drawn[j + 1];
    if (!a || !b || u < a[0] || u > b[0]) continue;
    gaps.push(v - (a[1] + ((b[1] - a[1]) * (u - a[0])) / (b[0] - a[0])));
  }
  if (gaps.length < fitPoints.length * 0.9) return { error: Infinity, offset: 0 };
  const sorted = [...gaps].sort((x, y) => x - y);
  return { error: gaps.reduce((sum, gap) => sum + Math.min(Math.abs(gap), 60), 0) / gaps.length, offset: sorted[sorted.length >> 1] };
}

/** The tilt follows from everything else, so it is solved rather than searched. */
function solveTilt(skyline, camera) {
  let result;
  for (let i = 0; i < 4; i++) {
    result = residual(skyline, camera);
    if (!Number.isFinite(result.error)) return result;
    camera.pitch -= Math.atan(result.offset / camera.f) * DEG;
  }
  return residual(skyline, camera);
}

function fitCamera() {
  let best = { error: Infinity };
  for (let north = -240; north <= 240; north += 40) {
    for (let east = -240; east <= 240; east += 40) {
      const position = { lat: HINT.lat + north / 111200, lon: HINT.lon + east / 92500 };
      const { azimuth } = toward(position, SUMMIT);
      const skyline = horizon(position, azimuth, 62, 0.2, 16000);
      for (const lens of [0.72, 0.78, 0.84]) {
        for (const roll of [-2, 0, 2]) {
          for (let turn = -12; turn <= 2; turn++) {
            const f = lens * W;
            const camera = { yaw: azimuth + turn, pitch: 15, roll, f };
            const result = solveTilt(skyline, camera);
            if (result.error < best.error) best = { ...result, camera: { ...camera }, position };
          }
        }
      }
    }
  }

  const skylineAt = (position) => horizon(position, toward(position, SUMMIT).azimuth, 66, 0.1, 18000);
  let skyline = skylineAt(best.position);
  const attempt = (camera) => {
    const trial = { ...camera };
    const result = solveTilt(skyline, trial);
    if (result.error < best.error - 1e-3) best = { ...best, ...result, camera: trial };
  };
  for (let round_ = 0; round_ < 7; round_++) {
    const scale = 1 / (1 + round_ * 0.8);
    for (const step of [-1, -0.5, -0.2, 0.2, 0.5, 1]) attempt({ ...best.camera, yaw: best.camera.yaw + step * scale });
    for (const step of [-1, -0.5, -0.2, 0.2, 0.5, 1]) attempt({ ...best.camera, roll: best.camera.roll + step * scale });
    for (const step of [-60, -30, -12, 12, 30, 60]) attempt({ ...best.camera, f: best.camera.f + step * scale });
    for (const [north, east] of [[-30, 0], [30, 0], [0, -30], [0, 30], [-15, 0], [15, 0], [0, -15], [0, 15]]) {
      const position = { lat: best.position.lat + (north * scale) / 111200, lon: best.position.lon + (east * scale) / 92500 };
      const moved = skylineAt(position);
      const trial = { ...best.camera };
      const result = solveTilt(moved, trial);
      if (result.error < best.error - 1e-3) {
        best = { ...best, ...result, camera: trial, position };
        skyline = moved;
      }
    }
  }
  return best;
}

console.time("camera");
const fit = fitCamera();
console.timeEnd("camera");
const { camera, position } = fit;
const ground = dem.elevation(position.lat, position.lon);
const eyeHeight = ground + EYE;

const metresApart = (a, b) => Math.hypot((a.lat - b.lat) * 111200, (a.lon - b.lon) * 92500);
const startIndex = track.slice(0, topIndex).reduce((best, point, i) => (metresApart(point, position) < metresApart(track[best], position) ? i : best), 0);
const fromTrack = metresApart(track[startIndex], position);
console.log(`camera ${position.lat.toFixed(5)}, ${position.lon.toFixed(5)} at ${ground.toFixed(0)} m, ${fromTrack.toFixed(0)} m from the track`);
console.log(`heading ${(camera.yaw % 360).toFixed(1)}, tilt ${camera.pitch.toFixed(1)}, roll ${camera.roll.toFixed(1)}, f/W ${(camera.f / W).toFixed(3)}, error ${fit.error.toFixed(1)} px`);

const inFrame = ([u, v]) => u >= -40 && u <= W + 40 && v >= -40 && v <= H + 40;

function photoSkyline() {
  const line = horizon(position, toward(position, SUMMIT).azimuth, 70, 0.05, 30000).map(([a, e]) => project(a, e, camera)).filter((p) => p && inFrame(p));
  return simplify(line, 0.8).map(([u, v]) => [round(u), round(v)]);
}

/** Whether nothing stands between the camera and a point of the track. */
function inSight(point, distance, height, azimuth) {
  const target = (height - eyeHeight - (distance * distance) / (2 * EFFECTIVE)) / distance;
  for (let d = 40; d < distance - 30; d += 15) {
    const p = destination(position, azimuth, d);
    if ((dem.elevation(p.lat, p.lon) - eyeHeight - (d * d) / (2 * EFFECTIVE)) / d > target + 0.004) return false;
  }
  return true;
}

function photoTrack() {
  const runs = [];
  let previous = null;
  for (const point of track.slice(startIndex, topIndex + 1)) {
    if (previous && metresApart(previous, point) < 12) continue;
    previous = point;
    const { azimuth, distance } = toward(position, point);
    if (distance < 160) continue;
    const height = dem.elevation(point.lat, point.lon);
    const at = project(azimuth, angleTo(height, distance, eyeHeight), camera);
    if (!at || !inFrame(at)) continue;
    const seen = inSight(point, distance, height, azimuth);
    const last = runs.at(-1);
    const dot = [round(at[0]), round(at[1])];
    if (last?.seen === seen) last.points.push(dot);
    else runs.push({ seen, points: last ? [last.points.at(-1), dot] : [dot] });
  }
  return runs;
}

/** What a scanner standing there would return: level rings of beams, first thing each one hits. */
function photoScan() {
  const center = camera.yaw;
  const bands = Array.from({ length: RANGE_LIMITS.length + 1 }, () => []);
  for (let tilt = -9; tilt <= 28; tilt += SCAN_TILT) {
    const rise = Math.tan(tilt * RAD);
    for (let azimuth = center - 36; azimuth <= center + 36; azimuth += SCAN_BEARING) {
      for (let d = 80; d < 7000; d += d < 1500 ? 12 : 24) {
        const p = destination(position, azimuth, d);
        const ground_ = dem.elevation(p.lat, p.lon) - (d * d) / (2 * EFFECTIVE);
        if (ground_ < eyeHeight + rise * d) continue;
        const at = project(azimuth, tilt, camera);
        if (at && at[0] >= 0 && at[0] <= W && at[1] >= 0 && at[1] <= H) {
          const band = RANGE_LIMITS.findIndex((limit) => d < limit);
          bands[band === -1 ? RANGE_LIMITS.length : band].push([Math.round(at[0]), Math.round(at[1])]);
        }
        break;
      }
    }
  }
  return bands;
}

const skyline = photoSkyline();
const trackRuns = photoTrack();
const scan = photoScan();
// The top of the track, not the model's highest node: that one is 160 m further along the ridge.
const top = track[topIndex];
const summitInfo = toward(position, top);
const summitAt = project(summitInfo.azimuth, angleTo(dem.elevation(top.lat, top.lon), summitInfo.distance, eyeHeight), camera);

// ------------------------------------------------------------------------------ the ray

function walkRay() {
  const { azimuth, distance: toTop } = toward(RAY_FROM, RAY_TO);
  const eye = dem.elevation(RAY_FROM.lat, RAY_FROM.lon) + RAY_EYE;
  const reach = toTop + RAY_PAST;
  const samples = [];
  let highest = -Infinity;
  for (let d = 60; d <= reach; d += RAY_STEP) {
    const p = destination(RAY_FROM, azimuth, d);
    const h = dem.elevation(p.lat, p.lon);
    const angle = angleTo(h, d, eye);
    const seen = angle > highest;
    if (seen) highest = angle;
    samples.push({ d, h, angle, highest, seen });
  }

  // One column per bin, keeping the sample that stands highest so no crest gets averaged away.
  const size = samples.length / RAY_BINS;
  const bins = [];
  for (let i = 0; i < RAY_BINS; i++) {
    const slice = samples.slice(Math.floor(i * size), Math.floor((i + 1) * size));
    if (!slice.length) continue;
    const top = slice.reduce((a, b) => (b.angle > a.angle ? b : a));
    bins.push([round(top.d / 1000, 2), Math.round(top.h), round(top.angle, 3), round(slice.at(-1).highest, 3), slice.some((s) => s.seen) ? 1 : 0]);
  }

  // The ridge that decides it: where the running highest angle last rose before Aconcagua.
  const before = samples.filter((s) => s.d < toTop - 1500);
  const crest = before.reduce((a, b) => (b.angle > a.angle ? b : a));
  const peak = samples.filter((s) => Math.abs(s.d - toTop) < 1500).reduce((a, b) => (b.h > a.h ? b : a));
  return {
    azimuth: round(azimuth, 2),
    eye: Math.round(eye),
    from: RAY_FROM.name,
    to: RAY_TO.name,
    crest: { km: round(crest.d / 1000, 1), h: Math.round(crest.h), angle: round(crest.angle, 3) },
    peak: { km: round(peak.d / 1000, 1), h: Math.round(peak.h), angle: round(peak.angle, 3) },
    bins,
  };
}

const ray = walkRay();
console.log(`ray: crest at ${ray.crest.km} km, ${ray.crest.angle}°; peak at ${ray.peak.km} km, ${ray.peak.angle}°`);

writeFileSync(
  OUT,
  `${JSON.stringify({
    photo: {
      src: "/assets/img/headers/san-gabriel-hero.jpg",
      width: W,
      height: H,
      camera: {
        lat: round(position.lat, 5),
        lon: round(position.lon, 5),
        ground: Math.round(ground),
        heading: round(camera.yaw % 360, 1),
        tilt: round(camera.pitch, 1),
        roll: round(camera.roll, 1),
        lens: round(camera.f / W, 3),
        error: round(fit.error, 1),
        fromTrack: Math.round(fromTrack),
      },
      summit: { name: SUMMIT.name, distance: Math.round(summitInfo.distance), at: summitAt.map((v) => round(v)) },
      skyline,
      track: trackRuns,
      scan,
      rangeLimits: RANGE_LIMITS,
    },
    ray,
  })}\n`,
);
console.log(`scan points ${scan.reduce((sum, band) => sum + band.length, 0)}, track runs ${trackRuns.length}`);

// ------------------------------------------------------------------------------ the header

const path = (points) => `M${points.map(([u, v]) => `${u} ${v}`).join("L")}`;
const INK = ["#0f6b75", "#128a94", "#1fa8ae", "#49c2c3", "#86d9d5"];
const dots = scan.map((band, i) => `<path d="${band.map(([u, v]) => `M${u} ${v}h0`).join("")}" stroke="${INK[i]}" stroke-width="5" stroke-linecap="round" fill="none"/>`).join("");
const overlay = `<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}">${dots}<path d="${path(skyline)}" fill="none" stroke="#ffffff" stroke-width="5"/>${trackRuns
  .filter((run) => run.seen)
  .map((run) => `<path d="${path(run.points)}" fill="none" stroke="#b6f36b" stroke-width="7" stroke-linejoin="round"/>`)
  .join("")}</svg>`;
const composed = await sharp(PHOTO).composite([{ input: Buffer.from(overlay) }]).jpeg({ quality: 90 }).toBuffer();
await sharp(composed).resize(1600).jpeg({ quality: 84, mozjpeg: true }).toFile(HEADER);
