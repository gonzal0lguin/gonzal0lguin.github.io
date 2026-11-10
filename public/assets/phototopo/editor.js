// The photo-topo editor: one dependency-free module that draws a scene file over its photo and
// lets a person finish what the fit started.
//
// It works from numbers the scene already holds. Terrain and track are stored as bearing,
// elevation angle and distance from the camera, so turning, tilting, rolling or zooming the
// camera is a re-projection done here on every pointer move, with no server. Moving the
// viewpoint is the one thing that needs the elevation model; that is offered only when the page
// is served by `mtn topo edit` (opts.api), and is a hint to run it otherwise.
//
// Two pieces are deliberately duplicated from the Python package and must change together:
// `project` (phototopo/camera.py) and the drawing rules in `trackRuns`/`overlay`
// (phototopo/render.py), `peakMarks`/`peakWords` included. The scene file is the contract;
// these are its two readers.

const RAD = Math.PI / 180;
const DEG = 180 / Math.PI;
const TOO_NEAR = 60;
const NEAR_DRAWN = 4; // a point placed from the track itself (7th number set) is drawn down to here
const PIN_SPAN = 30; // track points on each side of a hand correction over which it fades out
const DASHES = { solid: null, dashed: [3.2, 2.4], dotted: [0.1, 2.2] };
const SKYLINE_COLUMNS = 400; // the photo's skyline has one point per column of this grid
const PEAK_STYLE = { show: true, line: true, elevation: true, upper: false, color: "#ffffff", size: 0.02, width: 0.0014 };
const ASPECTS = { "4:5": 4 / 5, "1:1": 1, "9:16": 9 / 16, "16:9": 16 / 9 };
const MARK_SIZE = 0.018; // of the picture's width: what one unit of a symbol's drawing measures
// The symbols a person can put on the picture, drawn around the place they mark in units of the
// mark's size, y down: filled outlines, strokes, a disc or a circle. The same table as MARKS in
// render.py. The flag stands on its place; the others are centred on it.
const MARKS = {
  tent: [["fill", [[-1, 0.75], [0, -0.85], [1, 0.75], [0.32, 0.75], [0, 0.05], [-0.32, 0.75]]]],
  hut: [["fill", [[-0.9, 0.85], [-0.9, -0.05], [0, -0.9], [0.9, -0.05], [0.9, 0.85], [0.28, 0.85], [0.28, 0.2], [-0.28, 0.2], [-0.28, 0.85]]]],
  flag: [["line", [[0, 0], [0, -2]]], ["fill", [[0, -2], [1.4, -1.5], [0, -1]]]],
  peak: [["fill", [[-1, 0.8], [0, -0.9], [1, 0.8]]]],
  star: [["fill", [[0, -1], [0.247, -0.34], [0.951, -0.309], [0.399, 0.13], [0.588, 0.809], [0, 0.42], [-0.588, 0.809], [-0.399, 0.13], [-0.951, -0.309], [-0.247, -0.34]]]],
  diamond: [["fill", [[0, -1], [0.8, 0], [0, 1], [-0.8, 0]]]],
  drop: [["fill", [[0, -1], [0.55, -0.15], [0.7, 0.3], [0.5, 0.75], [0, 0.95], [-0.5, 0.75], [-0.7, 0.3], [-0.55, -0.15]]]],
  cross: [["line", [[-0.75, -0.75], [0.75, 0.75]]], ["line", [[-0.75, 0.75], [0.75, -0.75]]]],
  dot: [["dot", 0.7]],
  ring: [["ring", 0.8]],
};
const MARK_NAMES = [["tent", "Camp"], ["hut", "Hut"], ["flag", "Flag"], ["peak", "Peak"], ["star", "Star"], ["diamond", "Diamond"], ["drop", "Water"], ["cross", "Cross"], ["dot", "Dot"], ["ring", "Ring"]];

/** Pixel of a world direction, or null when it is behind the camera. */
export function project(az, alt, c, W, H) {
  const a = (az - c.yaw) * RAD, e = alt * RAD;
  const forward = Math.cos(a) * Math.cos(e), right = Math.sin(a) * Math.cos(e), up = Math.sin(e);
  const t = c.pitch * RAD, r = c.roll * RAD;
  const z = forward * Math.cos(t) + up * Math.sin(t);
  if (z <= 1e-6) return null;
  const y = -forward * Math.sin(t) + up * Math.cos(t);
  const across = right * Math.cos(r) + y * Math.sin(r);
  const along = -right * Math.sin(r) + y * Math.cos(r);
  const f = c.lens * W;
  return [W / 2 + (f * across) / z, H / 2 - (f * along) / z];
}

function smooth(line, window) {
  if (window <= 0 || line.length <= 2) return line;
  const at = (i) => line[Math.min(line.length - 1, Math.max(0, i))];
  const out = line.map((_, i) => {
    let x = 0, y = 0;
    for (let k = -window; k <= window; k++) { x += at(i + k)[0]; y += at(i + k)[1]; }
    return [x / (2 * window + 1), y / (2 * window + 1)];
  });
  out[0] = line[0];
  out[out.length - 1] = line[line.length - 1];
  return out;
}

/**
 * Per track point, the [bearing, angle] correction made by hand. A pin says "this point of the
 * route is really over there"; its shift fades to nothing `span` points away on each side,
 * unless another pin is met first, and then it runs straight from one to the other.
 * The same rule as `pin_shift` in render.py.
 */
export function pinShift(track) {
  const shift = track.points.map(() => [0, 0]);
  const pins = [...(track.pins || [])].sort((a, b) => a.i - b.i);
  if (!pins.length) return shift;
  const knots = [];
  pins.forEach((pin, n) => {
    const span = pin.span ?? PIN_SPAN;
    if (n === 0 || pins[n - 1].i < pin.i - span) knots.push([pin.i - span, 0, 0]);
    knots.push([pin.i, pin.daz, pin.dalt]);
    if (n === pins.length - 1 || pins[n + 1].i > pin.i + span) knots.push([pin.i + span, 0, 0]);
  });
  for (let k = 0; k < knots.length - 1; k++) {
    const [a, b] = [knots[k], knots[k + 1]];
    for (let i = Math.max(0, Math.ceil(a[0])); i <= Math.min(shift.length - 1, b[0]); i++) {
      const t = b[0] === a[0] ? 0 : (i - a[0]) / (b[0] - a[0]);
      shift[i] = [a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
    }
  }
  return shift;
}

/** Pixel of track point i with hand corrections applied, or null when it cannot be drawn. */
function trackPixel(scene, i, shift) {
  const p = scene.track.points[i], { width: W, height: H } = scene.photo;
  if (p[2] < (p[6] ? NEAR_DRAWN : TOO_NEAR)) return null;
  return project(p[0] + shift[i][0], p[1] + shift[i][1], scene.camera, W, H);
}

/** Pixel of a symbol: of its track point when it was put on the route, of its direction otherwise. */
function markPixel(scene, mark, shift) {
  if (mark.i != null) return scene.track.points[mark.i] ? trackPixel(scene, mark.i, shift) : null;
  return project(mark.az, mark.alt, scene.camera, scene.photo.width, scene.photo.height);
}

/**
 * Per track point, which entry of style.sections gives it its look, or -1 for the route's own.
 * A section [a, b] owns the line from a to b, so its last point belongs to what follows; later
 * sections lie over earlier ones. The same rule as `section_of` in render.py.
 */
export function sectionOf(scene) {
  const owner = scene.track.points.map(() => -1);
  (scene.style.sections || []).forEach((section, n) => {
    for (let i = Math.max(0, section.range[0]); i < Math.min(owner.length, section.range[1]); i++) owner[i] = n;
  });
  return owner;
}

/** The bearing and angle that put a world direction under a picture point (a few Newton steps). */
function aim(scene, az, alt, target) {
  const { width: W, height: H } = scene.photo, c = scene.camera, d = 0.05;
  for (let round = 0; round < 5; round++) {
    const at = project(az, alt, c, W, H), east = project(az + d, alt, c, W, H), up = project(az, alt + d, c, W, H);
    if (!at || !east || !up) break;
    const a = (east[0] - at[0]) / d, b = (up[0] - at[0]) / d, e = (east[1] - at[1]) / d, f = (up[1] - at[1]) / d;
    const det = a * f - b * e;
    if (Math.abs(det) < 1e-9) break;
    const dx = target[0] - at[0], dy = target[1] - at[1];
    az += Math.max(-10, Math.min(10, (dx * f - b * dy) / det));
    alt += Math.max(-10, Math.min(10, (a * dy - e * dx) / det));
  }
  return [az, alt];
}

/** The track as pixel polylines, each either all in sight or all hidden, and of one look. */
export function trackRuns(scene) {
  const { width: W, height: H } = scene.photo;
  const style = scene.style, all = scene.track.points;
  const [first, last] = style.range || [0, all.length - 1];
  const runs = [], shift = pinShift(scene.track), owner = sectionOf(scene);
  let current = [], seen = null, look = -1;
  const close = () => { if (current.length >= 2) runs.push({ seen, look, line: smooth(current, style.smooth | 0) }); };
  for (let i = first; i <= last; i++) {
    const p = all[i];
    const at = trackPixel(scene, i, shift);
    if (!at || at[0] <= -W || at[0] >= 2 * W || at[1] <= -H || at[1] >= 2 * H) { close(); current = []; seen = null; continue; }
    const visible = !!p[3];
    if (seen === null) { seen = visible; look = owner[i]; }
    else if (visible !== seen || owner[i] !== look) { current.push(at); close(); current = []; seen = visible; look = owner[i]; }
    current.push(at);
  }
  close();
  return runs;
}

export function cropBox(scene) {
  const { width: W, height: H } = scene.photo;
  const crop = scene.style.crop || {};
  const ratio = ASPECTS[crop.aspect];
  if (!ratio) return [0, 0, W, H];
  const [w, h] = W / H < ratio ? [W, W / ratio] : [H * ratio, H];
  const x = Math.min(W - w, Math.max(0, (crop.cx ?? 0.5) * W - w / 2));
  const y = Math.min(H - h, Math.max(0, (crop.cy ?? 0.5) * H - h / 2));
  return [Math.round(x), Math.round(y), Math.round(w), Math.round(h)];
}

const path = (line) => "M" + line.map(([u, v]) => `${u.toFixed(1)} ${v.toFixed(1)}`).join("L");
const esc = (text) => String(text).replace(/[&<>"]/g, (ch) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[ch]);

function chains(scene, list) {
  const { width: W, height: H } = scene.photo;
  const out = [];
  for (const chain of list) {
    let line = [];
    for (const p of chain) {
      const at = project(p[0], p[1], scene.camera, W, H);
      if (at && at[0] > -W && at[0] < 2 * W) line.push(at);
      else { if (line.length > 1) out.push(line); line = []; }
    }
    if (line.length > 1) out.push(line);
  }
  return out;
}

/** Splits points sorted by u wherever columns are missing: the photo's skyline has gaps. */
function pieces(points, gap) {
  const out = [];
  let line = [];
  for (const p of points) {
    if (line.length && p[0] - line[line.length - 1][0] > gap) { out.push(line); line = []; }
    line.push(p);
  }
  out.push(line);
  return out.filter((piece) => piece.length > 1);
}

/**
 * Rewrites the photo's skyline between two picture points: drawn through them, or rubbed out.
 * This is the safeguard for what the skyline reader gets wrong (a cloud bank, a tree line):
 * a person can see the ridge, and "Snap" and the refits use whatever is here.
 */
export function editSkyline(scene, from, to, erase = false) {
  const step = scene.photo.width / SKYLINE_COLUMNS;
  const columns = new Map(scene.photo_skyline.map(([u, v]) => [Math.floor(u / step), v]));
  const [a, b] = from[0] <= to[0] ? [from, to] : [to, from];
  const reach = erase ? 2 : 0;
  for (let i = Math.floor(a[0] / step) - reach; i <= Math.floor(b[0] / step) + reach; i++) {
    if (i < 0 || i >= SKYLINE_COLUMNS) continue;
    if (erase) { columns.delete(i); continue; }
    const u = (i + 0.5) * step;
    const t = b[0] === a[0] ? 1 : Math.min(1, Math.max(0, (u - a[0]) / (b[0] - a[0])));
    columns.set(i, Math.round(a[1] + (b[1] - a[1]) * t));
  }
  scene.photo_skyline = [...columns.entries()].sort((x, y) => x[0] - y[0]).map(([i, v]) => [Math.round((i + 0.5) * step), v]);
}

/** Shown summits as { peak, i, at: summit pixel, to: pixel its name is anchored at }. */
function shownPeaks(scene) {
  const { width: W, height: H } = scene.photo;
  const look = scene.style.peaks;
  if (!look || look.show === false) return [];
  const out = [];
  (scene.peaks || []).forEach((peak, i) => {
    if (!peak.show) return;
    const at = project(peak.az, peak.alt, scene.camera, W, H);
    if (at) out.push({ peak, i, at, to: [at[0] + (peak.dx || 0) * W, at[1] + (peak.dy || 0) * W] });
  });
  return out;
}

/** Pointer lines from summits to their names. */
export function peakMarks(scene) {
  return scene.style.peaks?.line === false ? [] : shownPeaks(scene).map(({ at, to }) => [at, to]);
}

/**
 * Summit names and heights as { text, x, y, size, i }, centred on x, y. The name stands on the
 * end of its pointer line with the height in smaller type under it; dragged below its summit,
 * the pair hangs from the line instead.
 */
export function peakWords(scene) {
  const W = scene.photo.width, look = scene.style.peaks, out = [];
  const thousands = (n) => String(Math.round(n)).replace(/\B(?=(\d{3})+(?!\d))/g, ".");
  for (const { peak, i, at, to } of shownPeaks(scene)) {
    const size = look.size * W;
    const rows = [[look.upper ? peak.name.toUpperCase() : peak.name, size]];
    if (look.elevation !== false && peak.ele) rows.push([`${thousands(peak.ele)} m`, size * 0.72]);
    const tall = rows.reduce((sum, row) => sum + row[1] * 1.18, 0);
    let y = to[1] <= at[1] ? to[1] - tall - size * 0.25 : to[1] + size * 0.25;
    for (const [text, rowSize] of rows) {
      out.push({ text, x: to[0], y: y + rowSize * 0.59, size: rowSize, i });
      y += rowSize * 1.18;
    }
  }
  return out;
}

const skylineChain = (scene) => scene.skyline.alt.map((alt, i) => [scene.skyline.az0 + scene.skyline.step * i, alt]);

/** Median vertical gap, in photo pixels, between the photo's skyline and the model's. */
export function skylineGap(scene, camera = scene.camera, capped = false) {
  const { width: W, height: H } = scene.photo;
  const model = [];
  for (const [az, alt] of skylineChain(scene)) {
    const at = project(az, alt, camera, W, H);
    if (at) model.push(at);
  }
  const cap = 0.03 * W, gaps = [];
  let j = 0;
  for (const [u, v] of scene.photo_skyline) {
    if (v < 0.03 * H) continue;
    while (j < model.length - 2 && model[j + 1][0] < u) j++;
    const a = model[j], b = model[j + 1];
    if (!a || !b || u < a[0] || u > b[0] || b[0] === a[0]) { gaps.push(cap); continue; }
    gaps.push(Math.min(cap, Math.abs(v - (a[1] + ((b[1] - a[1]) * (u - a[0])) / (b[0] - a[0])))));
  }
  if (!gaps.length) return Infinity;
  if (capped) return gaps.reduce((s, g) => s + g, 0) / gaps.length;
  return gaps.sort((x, y) => x - y)[gaps.length >> 1];
}

/** Mean distance in photo pixels between where control points were clicked and where the model draws them. */
export function controlGap(scene, camera = scene.camera) {
  const { width: W, height: H } = scene.photo, controls = scene.controls || [];
  if (!controls.length) return 0;
  let sum = 0;
  for (const c of controls) {
    const at = project(c.az, c.alt, camera, W, H);
    sum += at ? Math.hypot(at[0] - c.u * W, at[1] - c.v * H) : W;
  }
  return sum / controls.length;
}

/**
 * Finishes an alignment by hand: from where the outline was dragged, nudge heading, tilt, roll
 * and zoom one at a time, keep what brings the skylines closer, and halve the nudge. It only
 * ever improves on the starting point, so a rough drag is enough and a good one is not undone.
 * Control points count for more than the skyline: the camera is first turned onto them, however
 * far that is, and the nudges then have to keep them.
 */
export function snap(scene) {
  const { width: W, height: H } = scene.photo, controls = scene.controls || [];
  const price = (camera) => {
    const sky = skylineGap(scene, camera, true);
    return (Number.isFinite(sky) ? sky : 0) + 2 * controlGap(scene, camera);
  };
  let best = { ...scene.camera };
  if (controls.length) {
    const turned = { ...best };
    for (let round = 0; round < 8; round++) {
      let dx = 0, dy = 0, seen = 0;
      for (const c of controls) {
        const at = project(c.az, c.alt, turned, W, H);
        if (at) { dx += c.u * W - at[0]; dy += c.v * H - at[1]; seen++; }
      }
      if (!seen) { turned.yaw = controls[0].az; turned.pitch = controls[0].alt; continue; }
      const f = turned.lens * W;
      turned.yaw -= (Math.atan(dx / seen / f) * DEG) / Math.max(0.2, Math.cos(turned.pitch * RAD));
      turned.pitch = Math.max(-50, Math.min(50, turned.pitch + Math.atan(dy / seen / f) * DEG));
    }
    if (price(turned) < price(best)) best = turned;
  }
  let cost = price(best);
  const steps = { yaw: 1, pitch: 1, roll: 0.5, lens: 0.02 * best.lens };
  for (let round = 0; round < 9; round++) {
    for (const key of Object.keys(steps)) {
      for (const sign of [-1, 1]) {
        const trial = { ...best, [key]: best[key] + sign * steps[key] };
        const c = price(trial);
        if (c < cost - 1e-6) { best = trial; cost = c; }
      }
    }
    for (const key of Object.keys(steps)) steps[key] /= 2;
  }
  return best;
}

/** SVG markup of everything drawn over the photo, in photo pixels. */
export function overlay(scene, { align = false, photoSkyline = true, pins = false, picked = null } = {}) {
  const { width: W, height: H } = scene.photo;
  const s = scene.style, out = [];
  const stroke = (line, color, width, opacity = 1, dash = null) =>
    `<path d="${path(line)}" fill="none" stroke="${color}" stroke-width="${width.toFixed(2)}" stroke-opacity="${opacity}" stroke-linecap="round" stroke-linejoin="round"${dash ? ` stroke-dasharray="${(dash[0] * width).toFixed(1)} ${(dash[1] * width).toFixed(1)}"` : ""}/>`;

  if (align || s.ridges.show) {
    const look = align ? { color: "#ffffff", width: 0.0012, opacity: 0.75 } : s.ridges;
    for (const line of chains(scene, scene.ridges)) out.push(stroke(line, look.color, look.width * W, look.opacity));
  }
  if (align || s.skyline.show) {
    const look = align ? { color: "#ffffff", width: 0.003, opacity: 1 } : s.skyline;
    for (const line of chains(scene, [skylineChain(scene)])) {
      if (align) out.push(stroke(line, "#000000", look.width * W * 2, 0.35));
      out.push(stroke(line, look.color, look.width * W, look.opacity));
    }
  }
  if (align && photoSkyline) for (const line of pieces(scene.photo_skyline, (2.5 * W) / SKYLINE_COLUMNS)) out.push(stroke(line, "#ff3df2", 0.0016 * W, 0.9));

  const runs = trackRuns(scene);
  for (const under of [true, false]) {
    for (const { seen, look, line } of runs) {
      if (!seen && s.hidden.mode === "hide") continue;
      const track = look < 0 ? s.track : { ...s.track, ...s.sections[look] };
      const width = track.width * W, cased = width + 2 * s.casing.extra * W;
      const dash = seen || s.hidden.mode === "solid" ? DASHES[track.dash] : DASHES.dashed;
      const fade = seen ? 1 : s.hidden.opacity;
      if (under && s.casing.show) out.push(stroke(line, s.casing.color, cased, s.casing.opacity * fade, dash && [(dash[0] * width) / cased, (dash[1] * width) / cased]));
      else if (!under) out.push(stroke(line, track.color, width, track.opacity * fade, dash));
    }
  }

  const points = scene.track.points, [first, last] = s.range || [0, points.length - 1];
  const radius = s.markers.size * W, shift = pinShift(scene.track);
  for (const [wanted, i] of [[s.markers.start, first], [s.markers.top, scene.track.top]]) {
    if (!wanted || i < first || i > last) continue;
    const at = trackPixel(scene, i, shift);
    if (at) out.push(`<circle cx="${at[0].toFixed(1)}" cy="${at[1].toFixed(1)}" r="${(radius * 0.78).toFixed(1)}" fill="${s.markers.color}" stroke="${s.track.color}" stroke-width="${(radius * 0.45).toFixed(1)}"/>`);
  }

  const text = (item, x, y, kind, i) => {
    const size = (item.size ?? s.text.size) * W;
    const shadow = s.text.shadow ? ` stroke="#000" stroke-opacity="0.65" stroke-width="${(size / 7).toFixed(1)}" stroke-linejoin="round" paint-order="stroke"` : "";
    return `<text x="${x.toFixed(1)}" y="${y.toFixed(1)}" font-size="${size.toFixed(1)}" font-family="system-ui, -apple-system, 'Segoe UI', Helvetica, Arial, sans-serif" font-weight="600" text-anchor="middle" dominant-baseline="central" fill="${item.color ?? s.text.color}"${shadow} data-kind="${kind}" data-i="${i}" style="cursor:move">${esc(item.text)}</text>`;
  };
  (scene.marks || []).forEach((mark, i) => {
    const at = markPixel(scene, mark, shift), shape = MARKS[mark.kind];
    if (!at || !shape) return;
    const unit = (mark.size ?? MARK_SIZE) * W, fill = mark.color ?? "#ffffff";
    const grab = ` data-kind="mark" data-i="${i}" style="cursor:move"`;
    const edge = `stroke="#000" stroke-opacity="0.55" stroke-width="${(unit * 0.22).toFixed(1)}" stroke-linejoin="round" paint-order="stroke"`;
    let foot = 0;
    for (const [how, what] of shape) {
      if (how === "dot" || how === "ring") {
        const circle = `<circle cx="${at[0].toFixed(1)}" cy="${at[1].toFixed(1)}" r="${(what * unit).toFixed(1)}"`;
        if (how === "dot") out.push(`${circle} fill="${fill}" ${edge}${grab}/>`);
        else out.push(`${circle} fill="none" stroke="#000" stroke-opacity="0.55" stroke-width="${(unit * 0.5).toFixed(1)}"${grab}/>`, `${circle} fill="none" stroke="${fill}" stroke-width="${(unit * 0.28).toFixed(1)}"${grab}/>`);
        foot = Math.max(foot, what);
        continue;
      }
      const d = path(what.map(([x, y]) => [at[0] + x * unit, at[1] + y * unit]));
      if (how === "fill") out.push(`<path d="${d}Z" fill="${fill}" ${edge}${grab}/>`);
      else out.push(`<path d="${d}" fill="none" stroke="#000" stroke-opacity="0.55" stroke-width="${(unit * 0.5).toFixed(1)}" stroke-linecap="round"${grab}/>`, `<path d="${d}" fill="none" stroke="${fill}" stroke-width="${(unit * 0.28).toFixed(1)}" stroke-linecap="round"${grab}/>`);
      foot = Math.max(foot, ...what.map((p) => p[1]));
    }
    if (mark.text) out.push(text({ text: mark.text, size: (unit * 1.2) / W, color: fill }, at[0], at[1] + (foot + 1.1) * unit, "mark", i));
  });
  (scene.labels || []).forEach((label, i) => {
    if (label.show === false) return;
    const at = project(label.az, label.alt, scene.camera, W, H);
    if (!at) return;
    const to = [at[0] + (label.dx || 0) * W, at[1] + (label.dy || 0) * W];
    if (!label.line) { out.push(text(label, to[0], to[1], "label", i)); return; }
    // With a line the text stands on its end (or hangs from it), as a summit's name does.
    const size = (label.size ?? s.text.size) * W, width = (s.peaks?.width ?? 0.0014) * W;
    if (s.text.shadow) out.push(stroke([at, to], "#000000", width * 2.2, 0.45));
    out.push(stroke([at, to], label.color ?? s.text.color, width, 0.9));
    out.push(text(label, to[0], to[1] + (to[1] <= at[1] ? -0.85 : 0.85) * size, "label", i));
  });
  if (s.peaks) {
    for (const line of peakMarks(scene)) {
      if (s.text.shadow) out.push(stroke(line, "#000000", s.peaks.width * W * 2.2, 0.45));
      out.push(stroke(line, s.peaks.color, s.peaks.width * W, 0.9));
    }
    for (const word of peakWords(scene)) out.push(text({ text: word.text, size: word.size / W, color: s.peaks.color }, word.x, word.y, "peak", word.i));
  }
  (s.texts || []).forEach((item, i) => out.push(text(item, item.x * W, item.y * H, "text", i)));
  // Control points, while aligning: a cross where the summit was clicked, tied to where the
  // model has it now. When the two meet, the camera agrees with the person.
  if (align) (scene.controls || []).forEach((control) => {
    const at = [control.u * W, control.v * H], model = project(control.az, control.alt, scene.camera, W, H), r = 0.011 * W;
    const both = (line, width) => out.push(stroke(line, "#000000", width * 2.2, 0.55), stroke(line, "#22d3ee", width));
    if (model) {
      both([model, at], 0.0016 * W);
      out.push(`<circle cx="${model[0].toFixed(1)}" cy="${model[1].toFixed(1)}" r="${(0.005 * W).toFixed(1)}" fill="#22d3ee" stroke="#000" stroke-opacity="0.55" stroke-width="${(0.0016 * W).toFixed(1)}"/>`);
    }
    both([[at[0] - r, at[1]], [at[0] + r, at[1]]], 0.002 * W);
    both([[at[0], at[1] - r], [at[0], at[1] + r]], 0.002 * W);
    out.push(text({ text: control.name, size: 0.013, color: "#22d3ee" }, at[0], at[1] + 2.4 * r, "control", 0).replace(' style="cursor:move"', ""));
  });
  // Handles of the hand corrections: editor furniture, never part of the exported picture.
  if (pins) (scene.track.pins || []).forEach((pin, n) => {
    const at = trackPixel(scene, pin.i, shift);
    if (at) out.push(`<circle cx="${at[0].toFixed(1)}" cy="${at[1].toFixed(1)}" r="${(0.014 * W).toFixed(1)}" fill="#ffffff" fill-opacity="0.9" stroke="#111111" stroke-width="${(0.003 * W).toFixed(1)}" data-kind="pin" data-i="${n}" style="cursor:move"/>`);
  });
  // Where a stretch being selected starts: furniture too.
  if (picked != null) {
    const at = trackPixel(scene, picked, shift);
    if (at) out.push(`<circle cx="${at[0].toFixed(1)}" cy="${at[1].toFixed(1)}" r="${(0.012 * W).toFixed(1)}" fill="#ffd43b" stroke="#111111" stroke-width="${(0.003 * W).toFixed(1)}"/>`);
  }
  return out.join("");
}

/** The finished picture as a canvas, at the photo's own resolution. */
export async function renderCanvas(scene, image) {
  const [x, y, w, h] = cropBox(scene);
  const k = image.naturalWidth / scene.photo.width;
  const canvas = document.createElement("canvas");
  canvas.width = Math.round(w * k);
  canvas.height = Math.round(h * k);
  const ctx = canvas.getContext("2d");
  ctx.drawImage(image, x * k, y * k, w * k, h * k, 0, 0, canvas.width, canvas.height);
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" width="${canvas.width}" height="${canvas.height}" viewBox="${x} ${y} ${w} ${h}">${overlay(scene)}</svg>`;
  const url = URL.createObjectURL(new Blob([svg], { type: "image/svg+xml" }));
  try {
    const layer = new Image();
    layer.src = url;
    await layer.decode();
    ctx.drawImage(layer, 0, 0);
  } finally {
    URL.revokeObjectURL(url);
  }
  return canvas;
}

function el(tag, props = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(props)) {
    if (key === "class") node.className = value;
    else if (key.startsWith("on")) node.addEventListener(key.slice(2), value);
    else if (value !== false && value != null) node.setAttribute(key, value === true ? "" : value);
  }
  node.append(...children.filter((child) => child != null));
  return node;
}

function download(name, blob) {
  const link = el("a", { href: URL.createObjectURL(blob), download: name });
  link.click();
  setTimeout(() => URL.revokeObjectURL(link.href), 5000);
}

/**
 * Mounts the editor in `root`.
 *   api      base URL of `mtn topo edit` ("" when served by it); enables moving the camera and saving
 *   demos    [{ name, scene, photo }] URLs offered in a menu
 *   strings  { "English text": "translation" } for the interface
 */
export function mountEditor(root, opts = {}) {
  const T = (text) => opts.strings?.[text] ?? text;
  const api = opts.api ?? null;
  let scene = null, fitted = null, image = null, tab = "align", showPhotoSkyline = true, busy = false, pen = null, detected = null, pinSpan = PIN_SPAN;
  // What a click on the picture does in the Style tab: "fix" grabs the route, "stretch" picks the
  // two ends of a section, "mark" puts a symbol down. One at a time.
  let tool = null, stretchFrom = null, markKind = "tent";
  // Aligning: the next click on the photo says where summit number `target` of `targets()` is.
  let pointing = false, target = 0;
  const targets = () => [...scene.peaks.map((p) => ({ name: p.name, az: p.az, alt: p.alt, dist: p.dist })), ...(scene.labels || []).map((l) => ({ name: l.text, az: l.az, alt: l.alt, dist: l.dist }))];

  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("class", "pt-svg");
  const stage = el("div", { class: "pt-stage" }, svg);
  const panel = el("div", { class: "pt-panel" });
  const status = el("p", { class: "pt-status" });
  const tabs = el("div", { class: "pt-tabs", role: "tablist" });
  const picker = el("input", { type: "file", multiple: true, accept: ".json,image/*", hidden: true, onchange: (e) => openFiles(e.target.files) });
  const bar = el("div", { class: "pt-bar" }, tabs);
  if (opts.demos?.length) {
    const select = el("select", { class: "pt-input", "aria-label": T("Example"), onchange: (e) => loadUrls(opts.demos[e.target.value]) }, ...opts.demos.map((demo, i) => el("option", { value: i }, demo.name)));
    bar.prepend(select);
  }
  bar.append(el("button", { class: "pt-button", type: "button", onclick: () => picker.click() }, T("Open scene + photo…")), picker);
  const empty = el("div", { class: "pt-empty" }, el("p", {}, T("Drop a scene file (.scene.json) and its photo here.")), el("p", { class: "pt-hint" }, T("Make one with:"), " ", el("code", {}, "mtn topo fit photo.jpg track.gpx")));
  root.classList.add("pt");
  root.replaceChildren(bar, el("div", { class: "pt-main" }, stage, panel), status);
  stage.append(empty);

  root.addEventListener("dragover", (e) => e.preventDefault());
  root.addEventListener("drop", (e) => { e.preventDefault(); openFiles(e.dataTransfer.files); });

  const say = (text) => { status.textContent = text; };

  async function loadImage(url) {
    const img = new Image();
    img.src = url;
    await img.decode();
    return img;
  }

  async function setScene(next, img, keepFitted = false) {
    scene = next;
    if (img) image = img;
    if (!keepFitted) { fitted = { ...scene.camera }; detected = JSON.stringify(scene.photo_skyline); pen = null; }
    scene.peaks ||= [];
    scene.track.pins ||= [];
    scene.marks ||= [];
    scene.controls ||= [];
    scene.style.sections ||= [];
    scene.style.peaks = { ...PEAK_STYLE, ...scene.style.peaks };
    scene.style.range ||= [0, scene.track.points.length - 1];
    empty.remove();
    buildTabs();
    buildPanel();
    draw();
  }

  async function loadUrls({ scene: sceneUrl, photo }) {
    try {
      say(T("Loading…"));
      const [data, img] = await Promise.all([fetch(sceneUrl).then((r) => { if (!r.ok) throw new Error(r.status); return r.json(); }), loadImage(photo)]);
      await setScene(data, img);
    } catch (error) {
      say(`${T("Could not load")}: ${error.message}`);
    }
  }

  async function openFiles(files) {
    let data = null, img = null;
    for (const file of files) {
      if (file.name.endsWith(".json")) data = JSON.parse(await file.text());
      else if (file.type.startsWith("image/")) img = await loadImage(URL.createObjectURL(file));
    }
    if (data && (img || image)) await setScene(data, img);
    else if (img && scene) { image = img; draw(); }
    else say(T("Both are needed: the scene file and its photo."));
  }

  function draw() {
    if (!scene || !image) return;
    const align = tab === "align";
    const [x, y, w, h] = align ? [0, 0, scene.photo.width, scene.photo.height] : cropBox(scene);
    svg.setAttribute("viewBox", `${x} ${y} ${w} ${h}`);
    svg.innerHTML = `<image href="${image.src}" x="0" y="0" width="${scene.photo.width}" height="${scene.photo.height}" preserveAspectRatio="none"/>` + overlay(scene, { align, photoSkyline: showPhotoSkyline, pins: tab === "style" && tool === "fix", picked: tab === "style" && tool === "stretch" ? stretchFrom : null });
    svg.style.cursor = (align && (pen || pointing)) || (tab === "style" && tool) ? "crosshair" : "";
    const c = scene.camera, gap = skylineGap(scene);
    say(`${T("heading")} ${(((c.yaw % 360) + 360) % 360).toFixed(1)}° · ${T("tilt")} ${c.pitch.toFixed(1)}° · ${T("roll")} ${c.roll.toFixed(1)}° · ${T("lens")} ${Math.round(c.lens * 43.27 / Math.hypot(1, scene.photo.height / scene.photo.width))} mm · ${T("skylines")} ${Number.isFinite(gap) ? gap.toFixed(1) : "–"} px ${T("apart")}`);
    for (const sync of syncs) sync();
  }

  // ---------------------------------------------------------------------------- pointer
  // Through the SVG's own matrix, not the element's box: the picture is letterboxed inside the
  // box whenever their shapes differ (a portrait photo, a narrow window), and a pointer mapped
  // by the box lands beside where it is.
  const toPhoto = (e) => {
    const matrix = svg.getScreenCTM();
    if (!matrix) return [0, 0];
    const at = new DOMPoint(e.clientX, e.clientY).matrixTransform(matrix.inverse());
    return [at.x, at.y];
  };
  let drag = null;
  /** The drawn track point nearest a picture point, or -1 when the route is farther than `reach` picture widths. */
  function onRoute(at, reach = 0.05) {
    const shift = pinShift(scene.track), [first, last] = scene.style.range;
    // Measured to the line between stored points, not to the points: near the camera two
    // neighbours can be a hand's width apart on screen, and a click between them is on the route.
    let best = -1, gap = reach * scene.photo.width, before = null;
    for (let i = first; i <= last; i++) {
      const p = trackPixel(scene, i, shift);
      if (p && before) {
        const [a, b] = [before.p, p], dx = b[0] - a[0], dy = b[1] - a[1], length = dx * dx + dy * dy;
        const t = length ? Math.min(1, Math.max(0, ((at[0] - a[0]) * dx + (at[1] - a[1]) * dy) / length)) : 0;
        const d = Math.hypot(at[0] - a[0] - t * dx, at[1] - a[1] - t * dy);
        if (d < gap) { gap = d; best = t < 0.5 ? before.i : i; }
      }
      before = p ? { p, i } : null;
    }
    return best;
  }
  /**
   * Puts a symbol at a picture point. On the route line it takes the track point, so it stays
   * on the route through pins and a moved camera; anywhere else it takes the direction under
   * the pointer, which survives re-aiming the camera but not moving it.
   */
  function placeMark(mark, at) {
    const i = onRoute(at, 0.012);
    if (i >= 0) { mark.i = i; delete mark.az; delete mark.alt; return; }
    const [az, alt] = aim(scene, mark.az ?? scene.camera.yaw, mark.alt ?? scene.camera.pitch, at);
    delete mark.i;
    mark.az = +az.toFixed(3);
    mark.alt = +alt.toFixed(3);
  }
  svg.addEventListener("pointerdown", (e) => {
    if (!scene) return;
    const kind = e.target.dataset?.kind;
    const at = toPhoto(e);
    if (tab === "style" && tool === "stretch" && !kind) {
      const i = onRoute(at);
      if (i < 0) { say(T("Click on the route line to grab it.")); return; }
      if (stretchFrom === null) { stretchFrom = i; draw(); say(T("Now click where the stretch ends.")); return; }
      if (i !== stretchFrom) scene.style.sections.push({ range: [Math.min(i, stretchFrom), Math.max(i, stretchFrom)], color: scene.style.track.color.toLowerCase() === "#ffd43b" ? "#ffffff" : "#ffd43b", dash: "dashed", width: scene.style.track.width });
      stretchFrom = null;
      tool = null;
      buildPanel();
      draw();
      return;
    }
    if (tab === "style" && tool === "mark" && !kind) {
      const mark = { kind: markKind, text: "", size: MARK_SIZE, color: "#ffffff" };
      placeMark(mark, at);
      scene.marks.push(mark);
      tool = null;
      drag = { kind: "mark", i: scene.marks.length - 1, at, to: at };
      buildPanel();
      draw();
    } else if (tab === "style" && tool === "fix" && !kind) {  // a name or a handle under the pointer is dragged as itself
      // Grab the route where it was touched: the nearest drawn point becomes a pin.
      const best = onRoute(at), shift = pinShift(scene.track);
      if (best < 0) { say(T("Click on the route line to grab it.")); return; }
      const same = scene.track.pins.findIndex((pin) => pin.i === best);
      if (same >= 0) scene.track.pins.splice(same, 1);
      scene.track.pins.push({ i: best, daz: shift[best][0], dalt: shift[best][1], span: pinSpan });
      drag = { kind: "pin", i: scene.track.pins.length - 1, at };
      draw();
    } else if (kind && tab !== "align") drag = { kind, i: +e.target.dataset.i, at };
    else if (tab === "align" && pointing) {
      const chosen = targets()[target];
      pointing = false;
      if (chosen) {
        scene.controls = scene.controls.filter((c) => c.name !== chosen.name);
        scene.controls.push({ ...chosen, u: +(at[0] / scene.photo.width).toFixed(4), v: +(at[1] / scene.photo.height).toFixed(4) });
        Object.assign(scene.camera, snap(scene));
      }
      buildPanel();
      draw();
      return;
    }
    else if (tab === "align" && pen) { drag = { kind: "pen", at }; editSkyline(scene, at, at, pen === "erase"); draw(); }
    else if (tab === "align") drag = { kind: "camera", at };
    else if (scene.style.crop.aspect !== "original") drag = { kind: "crop", at };
    else return;
    svg.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  svg.addEventListener("pointermove", (e) => {
    if (!drag) return;
    const at = toPhoto(e), dx = at[0] - drag.at[0], dy = at[1] - drag.at[1];
    const { width: W, height: H } = scene.photo;
    drag.at = at;
    if (drag.kind === "camera") {
      // The outline follows the pointer: moving it right is the camera turning left.
      const f = scene.camera.lens * W;
      scene.camera.yaw -= Math.atan(dx / f) * DEG / Math.max(0.2, Math.cos(scene.camera.pitch * RAD));
      scene.camera.pitch += Math.atan(dy / f) * DEG;
    } else if (drag.kind === "pen") {
      editSkyline(scene, [at[0] - dx, at[1] - dy], at, pen === "erase");
    } else if (drag.kind === "pin") {
      const pin = scene.track.pins[drag.i], p = scene.track.points[pin.i];
      const [az, alt] = aim(scene, p[0] + pin.daz, p[1] + pin.dalt, at);
      pin.daz = +(az - p[0]).toFixed(3);
      pin.dalt = +(alt - p[1]).toFixed(3);
    } else if (drag.kind === "mark") {
      // The pointer's own path is followed, not the symbol's: held by the route, the symbol
      // lags behind the pointer and has to let go again at the same distance.
      const mark = scene.marks[drag.i];
      drag.to ||= markPixel(scene, mark, pinShift(scene.track)) || at;
      drag.to = [drag.to[0] + dx, drag.to[1] + dy];
      placeMark(mark, drag.to);
    } else if (drag.kind === "peak") {
      const peak = scene.peaks[drag.i];
      peak.dx = (peak.dx || 0) + dx / W;
      peak.dy = (peak.dy || 0) + dy / W;
    } else if (drag.kind === "label") {
      const label = scene.labels[drag.i];
      label.dx = (label.dx || 0) + dx / W;
      label.dy = (label.dy || 0) + dy / W;
    } else if (drag.kind === "text") {
      const item = scene.style.texts[drag.i];
      item.x += dx / W;
      item.y += dy / H;
    } else if (drag.kind === "crop") {
      scene.style.crop.cx = Math.min(1, Math.max(0, (scene.style.crop.cx ?? 0.5) - dx / W));
      scene.style.crop.cy = Math.min(1, Math.max(0, (scene.style.crop.cy ?? 0.5) - dy / H));
      drag.at = toPhoto(e);
    }
    draw();
    if (drag.kind === "crop") drag.at = toPhoto(e);
  });
  svg.addEventListener("pointerup", () => { drag = null; });
  svg.addEventListener("pointercancel", () => { drag = null; });
  svg.addEventListener("dblclick", (e) => {
    if (e.target.dataset?.kind !== "pin") return;
    scene.track.pins.splice(+e.target.dataset.i, 1);
    draw();
  });
  svg.addEventListener("wheel", (e) => {
    if (!scene || tab !== "align") return;
    e.preventDefault();
    if (e.shiftKey) scene.camera.roll += Math.sign(e.deltaY || e.deltaX) * 0.1;
    else scene.camera.lens *= Math.exp(-e.deltaY * 0.0004);
    draw();
  }, { passive: false });

  // ---------------------------------------------------------------------------- panel
  let syncs = [];
  const get = (object, key) => object[key];

  function slider(label, object, key, min, max, step, show = (v) => v) {
    const input = el("input", { type: "range", min, max, step, class: "pt-range" });
    const value = el("span", { class: "pt-value" });
    const sync = () => { input.value = get(object, key); value.textContent = show(+get(object, key)); };
    input.addEventListener("input", () => { object[key] = +input.value; draw(); });
    syncs.push(sync);
    return el("label", { class: "pt-field" }, el("span", {}, T(label)), input, value);
  }
  function color(label, object, key) {
    const input = el("input", { type: "color", class: "pt-color", value: object[key] });
    input.addEventListener("input", () => { object[key] = input.value; draw(); });
    return el("label", { class: "pt-field" }, el("span", {}, T(label)), input);
  }
  function choice(label, object, key, options) {
    const input = el("select", { class: "pt-input" }, ...options.map(([value, name]) => el("option", { value, selected: object[key] === value }, T(name))));
    input.addEventListener("change", () => { object[key] = input.value; draw(); });
    return el("label", { class: "pt-field" }, el("span", {}, T(label)), input);
  }
  function toggle(label, object, key) {
    const input = el("input", { type: "checkbox", checked: object[key] !== false && !!object[key] });
    input.addEventListener("change", () => { object[key] = input.checked; draw(); });
    return el("label", { class: "pt-check" }, input, el("span", {}, T(label)));
  }
  const group = (title, ...children) => el("fieldset", { class: "pt-group" }, el("legend", {}, T(title)), ...children);
  const button = (label, onclick, primary = false) => el("button", { class: primary ? "pt-button pt-button--primary" : "pt-button", type: "button", onclick }, T(label));

  function buildTabs() {
    tabs.replaceChildren(...[["align", "Align"], ["style", "Style"], ["export", "Export"]].map(([id, label]) =>
      el("button", { class: "pt-tab", type: "button", role: "tab", "aria-selected": tab === id ? "true" : "false", onclick: () => { tab = id; tool = stretchFrom = null; pointing = false; buildTabs(); buildPanel(); draw(); } }, T(label))));
  }

  async function moveCamera(refit, north = 0, east = 0) {
    if (busy) return;
    busy = true;
    say(T(refit ? "Refitting against the terrain…" : "Scanning the terrain from the new position…"));
    pen = null;
    try {
      const camera = { ...scene.camera };
      camera.lat += (north / 6371008.8) * DEG;
      camera.lon += (east / (6371008.8 * Math.cos(camera.lat * RAD))) * DEG;
      const response = await fetch(`${api}/api/camera`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ scene, camera, refit }) });
      const next = await response.json();
      if (next.error) throw new Error(next.error);
      busy = false;
      await setScene(next, null, true);
    } catch (error) {
      busy = false;
      say(`${T("Failed")}: ${error.message}`);
    }
  }

  function buildPanel() {
    syncs = [];
    const s = scene.style, c = scene.camera, base = fitted;
    if (tab === "align") {
      const nudge = { north: 0, east: 0 };
      const number = (label, key) => {
        const input = el("input", { type: "number", class: "pt-input pt-input--short", value: 0, step: 5 });
        input.addEventListener("input", () => { nudge[key] = +input.value || 0; });
        return el("label", { class: "pt-field" }, el("span", {}, T(label)), input);
      };
      const check = el("input", { type: "checkbox", checked: showPhotoSkyline });
      check.addEventListener("change", () => { showPhotoSkyline = check.checked; draw(); });
      const places = targets(), choose = { get i() { return String(target); }, set i(v) { target = +v; } };
      if (target >= places.length) target = 0;
      const usePen = (mode) => { pointing = false; pen = pen === mode ? null : mode; showPhotoSkyline = true; svg.style.cursor = pen ? "crosshair" : ""; buildPanel(); draw(); };
      const penButton = (label, mode) => el("button", { class: pen === mode ? "pt-button pt-button--on" : "pt-button", type: "button", "aria-pressed": pen === mode ? "true" : "false", onclick: () => usePen(mode) }, T(label));
      panel.replaceChildren(
        el("p", { class: "pt-hint" }, T("Drag the picture to move the terrain outline (white) onto the mountains. Scroll to zoom the lens, Shift+scroll to roll. The pink line is the skyline read from the photo.")),
        group("Camera",
          slider("Heading", c, "yaw", base.yaw - 45, base.yaw + 45, 0.05, (v) => `${(((v % 360) + 360) % 360).toFixed(1)}°`),
          slider("Tilt", c, "pitch", -45, 45, 0.05, (v) => `${v.toFixed(1)}°`),
          slider("Roll", c, "roll", -12, 12, 0.05, (v) => `${v.toFixed(1)}°`),
          slider("Lens", c, "lens", base.lens * 0.6, base.lens * 1.6, 0.001, (v) => v.toFixed(3)),
          el("label", { class: "pt-check" }, check, el("span", {}, T("Show the photo's skyline"))),
          el("div", { class: "pt-row" },
            button("Snap to skyline", () => { Object.assign(scene.camera, snap(scene)); draw(); }, true),
            button("Reset", () => { Object.assign(scene.camera, fitted); draw(); }))),
        group("Photo skyline",
          el("p", { class: "pt-hint" }, T("If the pink line follows a cloud or a tree instead of the ridge, fix it: draw along the real ridge, or erase the wrong stretch. Then snap or refit.")),
          el("div", { class: "pt-row" }, penButton("Draw", "draw"), penButton("Erase", "erase"),
            button("Undo edits", () => { scene.photo_skyline = JSON.parse(detected); draw(); })),
          api !== null ? el("div", { class: "pt-row" }, button("Fit again from scratch", () => moveCamera("full"))) : null),
        group("Control points",
          el("p", { class: "pt-hint" }, T("If you can tell a summit in the photo, tie it down: choose it, press the button and click on it in the photo. The outline snaps to it, and snapping and refitting hold to it. Two or three settle a photo with little skyline.")),
          ...(places.length
            ? [choice("Summit", choose, "i", places.map((place, i) => [String(i), place.name])),
               el("div", { class: "pt-row" }, el("button", { class: pointing ? "pt-button pt-button--on" : "pt-button", type: "button", "aria-pressed": pointing ? "true" : "false", onclick: () => { pointing = !pointing; pen = null; buildPanel(); draw(); } }, T("Click it on the photo")))]
            : [el("p", { class: "pt-hint" }, T("No named summit is in sight in this scene."))]),
          ...scene.controls.map((control, n) => el("div", { class: "pt-head" }, el("span", {}, control.name),
            el("button", { class: "pt-button", type: "button", title: T("Delete"), "aria-label": `${T("Delete")} ${control.name}`, onclick: () => { scene.controls.splice(n, 1); buildPanel(); draw(); } }, "×")))),
        group("Position",
          el("p", { class: "pt-hint" }, `${c.lat.toFixed(5)}, ${c.lon.toFixed(5)} · ${Math.round(c.ground ?? 0)} m`),
          ...(api !== null
            ? [number("Move north (m)", "north"), number("Move east (m)", "east"),
               el("div", { class: "pt-row" }, button("Move camera", () => moveCamera(false, nudge.north, nudge.east)), button("Refit from here", () => moveCamera(true, nudge.north, nudge.east), true))]
            : [el("p", { class: "pt-hint" }, T("Moving the camera needs the elevation model. Run the editor locally for that:"), " ", el("code", {}, "mtn topo edit photo.scene.json"))])),
      );
    } else if (tab === "style") {
      const last = scene.track.points.length - 1;
      const range = { get start() { return s.range[0]; }, set start(v) { s.range[0] = Math.min(v, s.range[1] - 1); }, get end() { return s.range[1]; }, set end(v) { s.range[1] = Math.max(v, s.range[0] + 1); } };
      const km = (i) => `${scene.track.points[Math.round(i)][4].toFixed(1)} km`;
      // The sliders step by track point, which on a long track is coarse; a typed distance
      // goes to the point nearest to it.
      const typed = (label, key) => {
        const input = el("input", { type: "number", class: "pt-input pt-input--short", min: 0, max: scene.track.points[last][4], step: 0.01 });
        const sync = () => { if (document.activeElement !== input) input.value = scene.track.points[range[key]][4].toFixed(2); };
        input.addEventListener("change", () => {
          const want = +input.value;
          let best = 0;
          scene.track.points.forEach((p, i) => { if (Math.abs(p[4] - want) < Math.abs(scene.track.points[best][4] - want)) best = i; });
          range[key] = best;
          draw();
          input.value = scene.track.points[range[key]][4].toFixed(2);
        });
        syncs.push(sync);
        return el("label", { class: "pt-field pt-field--half" }, el("span", {}, T(label)), input);
      };
      const rows = (list, kind) => list.map((item, i) => {
        const input = el("input", { type: "text", class: "pt-input", value: item.text });
        input.addEventListener("input", () => { item.text = input.value; draw(); });
        const size = el("input", { type: "range", class: "pt-range", min: 0.01, max: 0.08, step: 0.001, value: item.size ?? s.text.size });
        size.addEventListener("input", () => { item.size = +size.value; draw(); });
        const shown = el("input", { type: "checkbox", checked: item.show !== false, title: T("Show") });
        shown.addEventListener("change", () => { item.show = shown.checked; draw(); });
        const line = el("input", { type: "checkbox", checked: !!item.line, title: T("Pointer line") });
        line.addEventListener("change", () => { item.line = line.checked; draw(); });
        return el("div", { class: "pt-label" }, kind === "label" ? shown : null, input, size, kind === "label" ? line : null,
          kind === "text" ? el("button", { class: "pt-button", type: "button", title: T("Delete"), onclick: () => { s.texts.splice(i, 1); buildPanel(); draw(); } }, "×") : null);
      });
      const useTool = (label, name) => el("button", { class: tool === name ? "pt-button pt-button--on" : "pt-button", type: "button", "aria-pressed": tool === name ? "true" : "false", onclick: () => { tool = tool === name ? null : name; stretchFrom = null; buildPanel(); draw(); } }, T(label));
      const remove = (list, i) => el("button", { class: "pt-button", type: "button", title: T("Delete"), "aria-label": T("Delete"), onclick: () => { list.splice(i, 1); buildPanel(); draw(); } }, "×");
      const lines = [["solid", "Solid"], ["dashed", "Dashed"], ["dotted", "Dotted"]];
      const stretches = s.sections.map((section, n) => {
        const ends = { get start() { return section.range[0]; }, set start(v) { section.range[0] = Math.min(v, section.range[1] - 1); }, get end() { return section.range[1]; }, set end(v) { section.range[1] = Math.max(v, section.range[0] + 1); } };
        return el("div", { class: "pt-item" },
          el("div", { class: "pt-head" }, el("strong", {}, `${T("Stretch")} ${n + 1}`), remove(s.sections, n)),
          color("Colour", section, "color"),
          choice("Line", section, "dash", lines),
          slider("Width", section, "width", 0.001, 0.015, 0.0005, (v) => `${(v * 1000).toFixed(1)}`),
          slider("From", ends, "start", 0, last, 1, km),
          slider("To", ends, "end", 0, last, 1, km));
      });
      const kinds = { get new() { return markKind; }, set new(v) { markKind = v; } };
      const symbols = scene.marks.map((mark, n) => {
        const words = el("input", { type: "text", class: "pt-input", value: mark.text || "", placeholder: T("Name (optional)"), "aria-label": T("Name (optional)") });
        words.addEventListener("input", () => { mark.text = words.value; draw(); });
        return el("div", { class: "pt-item" },
          el("div", { class: "pt-head" }, words, remove(scene.marks, n)),
          choice("Symbol", mark, "kind", MARK_NAMES),
          color("Colour", mark, "color"),
          slider("Size", mark, "size", 0.006, 0.06, 0.001, (v) => `${(v * 1000).toFixed(0)}`));
      });
      panel.replaceChildren(
        group("Route",
          color("Colour", s.track, "color"),
          slider("Width", s.track, "width", 0.001, 0.015, 0.0005, (v) => `${(v * 1000).toFixed(1)}`),
          choice("Line", s.track, "dash", lines),
          slider("Opacity", s.track, "opacity", 0.1, 1, 0.05, (v) => `${Math.round(v * 100)}%`),
          slider("Smoothing", s, "smooth", 0, 12, 1),
          choice("Behind the mountain", s.hidden, "mode", [["dashed", "Dashed"], ["solid", "Same line"], ["hide", "Hidden"]]),
          slider("From", range, "start", 0, last, 1, km),
          slider("To", range, "end", 0, last, 1, km),
          el("div", { class: "pt-row" }, typed("From (km)", "start"), typed("To (km)", "end")),
          el("div", { class: "pt-row" }, button("Way up only", () => { s.range = [0, scene.track.top]; draw(); }), button("Whole track", () => { s.range = [0, last]; draw(); }))),
        group("Stretches",
          el("p", { class: "pt-hint" }, T("Give a part of the route its own colour and line: press the button, then click on the route where the stretch starts and where it ends.")),
          el("div", { class: "pt-row" }, useTool("Select a stretch", "stretch")),
          ...stretches),
        group("Fix the route",
          el("p", { class: "pt-hint" }, T("Where the route is drawn off the ground, switch this on and drag the line to where it really goes. Each drag leaves a handle; double-click a handle to remove it.")),
          el("div", { class: "pt-row" },
            useTool("Adjust route", "fix"),
            button("Clear adjustments", () => { scene.track.pins = []; draw(); })),
          slider("Reach of a drag", { get v() { return pinSpan; }, set v(x) { pinSpan = x; } }, "v", 3, 150, 1, (v) => `${v} ${T("points")}`)),
        group("Outline", toggle("Dark edge", s.casing, "show"), color("Edge colour", s.casing, "color"), slider("Edge strength", s.casing, "opacity", 0, 1, 0.05, (v) => `${Math.round(v * 100)}%`)),
        group("Markers", toggle("Start", s.markers, "start"), toggle("Summit", s.markers, "top"), slider("Size", s.markers, "size", 0.003, 0.02, 0.0005, (v) => `${(v * 1000).toFixed(1)}`), color("Fill", s.markers, "color")),
        group("Symbols",
          el("p", { class: "pt-hint" }, T("Mark a camp, a hut or anything else: pick a symbol, press the button and click on the picture. Dropped on the route line it stays on the route. Drag a symbol to move it.")),
          choice("Symbol", kinds, "new", MARK_NAMES),
          el("div", { class: "pt-row" }, useTool("Place on the picture", "mark")),
          ...symbols),
        group("Terrain", toggle("Skyline", s.skyline, "show"), color("Skyline colour", s.skyline, "color"), toggle("Ridges", s.ridges, "show"), color("Ridge colour", s.ridges, "color"), slider("Ridge opacity", s.ridges, "opacity", 0.1, 1, 0.05, (v) => `${Math.round(v * 100)}%`)),
        group("Text",
          el("p", { class: "pt-hint" }, T("Drag any text on the picture to place it. The second tick gives a label a pointer line to its place on the mountain.")),
          ...rows(scene.labels || [], "label"), ...rows(s.texts, "text"),
          el("div", { class: "pt-row" }, button("Add text", () => { s.texts.push({ text: scene.track.name || T("Text"), x: 0.5, y: 0.12, size: 0.04 }); buildPanel(); draw(); })),
          color("Colour", s.text, "color"), toggle("Dark halo", s.text, "shadow")),
        group("Summits",
          ...(scene.peaks.length
            ? [el("p", { class: "pt-hint" }, T("Names from OpenStreetMap. Tick the ones to show and drag a name to place it: its line follows.")),
               toggle("Show summits", s.peaks, "show"), toggle("Pointer line", s.peaks, "line"), toggle("Height", s.peaks, "elevation"), toggle("Capitals", s.peaks, "upper"),
               color("Colour", s.peaks, "color"),
               slider("Size", s.peaks, "size", 0.008, 0.05, 0.001, (v) => `${(v * 1000).toFixed(0)}`),
               slider("Line width", s.peaks, "width", 0.0005, 0.005, 0.0001, (v) => `${(v * 1000).toFixed(1)}`),
               el("div", { class: "pt-peaks" }, ...scene.peaks.map((peak) => {
                 const shown = el("input", { type: "checkbox", checked: !!peak.show, title: T("Show") });
                 shown.addEventListener("change", () => { peak.show = shown.checked; draw(); });
                 const name = el("input", { type: "text", class: "pt-input", value: peak.name });
                 name.addEventListener("input", () => { peak.name = name.value; draw(); });
                 return el("div", { class: "pt-label" }, shown, name, el("span", { class: "pt-value" }, `${(peak.dist / 1000).toFixed(peak.dist < 10000 ? 1 : 0)} km`));
               }))]
            : [el("p", { class: "pt-hint" }, T("No named summit is in sight in this scene.")),
               api !== null ? el("div", { class: "pt-row" }, button("Look up summits", () => moveCamera(false))) : null])),
        group("Frame",
          choice("Shape", s.crop, "aspect", [["original", "Original"], ["4:5", "4:5 portrait"], ["1:1", "1:1 square"], ["9:16", "9:16 story"], ["16:9", "16:9 wide"]]),
          el("p", { class: "pt-hint" }, T("Drag the picture to move the frame."))),
      );
    } else {
      const name = (scene.photo.file || "topo").replace(/\.[^.]+$/, "");
      const save = async (type, ext) => {
        say(T("Rendering…"));
        const canvas = await renderCanvas(scene, image);
        canvas.toBlob((blob) => { download(`${name}.topo.${ext}`, blob); say(`${canvas.width} × ${canvas.height} px`); }, type, 0.92);
      };
      panel.replaceChildren(
        group("Picture", el("p", { class: "pt-hint" }, T("Saved at the photo's full resolution, in the chosen frame.")), el("div", { class: "pt-row" }, button("Download JPG", () => save("image/jpeg", "jpg"), true), button("Download PNG", () => save("image/png", "png")))),
        group("Scene",
          el("p", { class: "pt-hint" }, T("The scene file keeps the camera and the style, to carry on later or to render from the command line.")),
          el("div", { class: "pt-row" },
            button("Download scene", () => download(`${name}.scene.json`, new Blob([JSON.stringify(scene)], { type: "application/json" }))),
            api !== null ? button("Save", async () => {
              const result = await fetch(`${api}/api/save`, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(scene) }).then((r) => r.json());
              say(result.error ? `${T("Failed")}: ${result.error}` : `${T("Saved")} ${result.saved}`);
            }, true) : null)),
      );
    }
    for (const sync of syncs) sync();
  }

  if (api !== null) loadUrls({ scene: `${api}/api/scene`, photo: `${api}/api/photo` });
  else if (opts.sceneUrl) loadUrls({ scene: opts.sceneUrl, photo: opts.photoUrl });
  else if (opts.demos?.length) loadUrls(opts.demos[0]);
  return { load: loadUrls, get scene() { return scene; } };
}
