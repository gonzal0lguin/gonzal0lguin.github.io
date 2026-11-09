// The horizon that opens the home page, drawn from what scripts/panoramas.mjs computed for a
// summit: an arc of the view, one layer per depth band, far to near.
//
// One degree across is one degree up, as on the summit pages, and on a laptop the same number of
// pixels per degree, so a mountain has here exactly the outline it has there. That is why the arc
// is only as wide as a screen holds at that scale: it centres on the highest summit in view
// instead of trying to fit more of the horizon by squeezing it.

import type { Summit } from "./summits";

/** Summits offered on the home page, the first one shown when it opens. */
export const HOME_SUMMITS = ["punta-canaletas", "union", "la-cruz", "provincia", "osorno"];

export const STAGE_W = 1600;
export const STAGE_H = 540;
/** viewBox units per degree, on both axes: 32 screen pixels on a 1440 screen, like MountainPanorama. */
const PX = 44;
const SPAN = STAGE_W / PX;
/** Sky kept above the highest point of the arc, for the names. */
const HEADROOM = 150;
const MIN_LABEL_GAP = 8.5;
const LABELS = 8;

export interface StageLayer {
  depth: number;
  fill: string;
  line: string;
}
export interface StagePeak {
  name: string;
  ele: number;
  distance: number;
  azimuth: number;
  x: number;
  y: number;
}
export interface Stage {
  layers: StageLayer[];
  ridges: string[];
  skyline: string;
  labels: (StagePeak & { tier: number })[];
  peaks: StagePeak[];
  center: number;
  span: number;
  highest: StagePeak | null;
}

const relative = (azimuth: number, center: number) => ((azimuth - center + 540) % 360) - 180;

export function buildStage(summit: Summit): Stage {
  const panorama = summit.panorama!;
  // The highest one actually in view: stats.highest can be a summit that only shows a flank.
  const inView = panorama.peaks.filter((peak) => peak.status === "visible" && peak.name && peak.ele);
  const center = inView.reduce((best, peak) => (peak.ele! > best.ele! ? peak : best), inView[0])?.azimuth ?? 90;
  const inArc = (points: number[][]) =>
    points
      .map(([azimuth, angle]) => [relative(azimuth, center), angle])
      .filter(([offset]) => Math.abs(offset) <= SPAN / 2 + 0.5);

  const skyline = inArc(panorama.skyline).sort((a, b) => a[0] - b[0]);
  const top = Math.max(...skyline.map(([, angle]) => angle));
  const x = (offset: number) => (offset + SPAN / 2) * PX;
  const y = (angle: number) => HEADROOM + (top - angle) * PX;
  const px = (value: number) => value.toFixed(1);
  const trace = (points: number[][]) => points.map(([offset, angle]) => `${px(x(offset))} ${px(y(angle))}`).join("L");

  const layers = panorama.bandSkylines
    .map((band, depth) => {
      const points = inArc(band).sort((a, b) => a[0] - b[0]);
      return { depth, points };
    })
    // A band that is all below the bottom edge is one the frame doesn't show.
    .filter(({ points }) => points.length > 1 && points.some(([, angle]) => y(angle) < STAGE_H))
    .map(({ depth, points }) => {
      const line = trace(points);
      return { depth, line: `M${line}`, fill: `M${px(x(points[0][0]))} ${STAGE_H}L${line}L${px(x(points.at(-1)![0]))} ${STAGE_H}Z` };
    })
    .reverse();

  const ridges = panorama.ridges
    .map((ridge) => inArc(ridge.points))
    .filter((points) => points.length > 1 && points.some(([, angle]) => y(angle) < STAGE_H))
    .map((points) => `M${trace(points)}`);

  const peaks = inView
    .map((peak) => ({ peak, offset: relative(peak.azimuth, center) }))
    .filter(({ peak, offset }) => Math.abs(offset) <= SPAN / 2 && y(peak.angle) < STAGE_H)
    .map(({ peak, offset }) => ({
      name: peak.name,
      ele: peak.ele!,
      distance: peak.distance,
      azimuth: peak.azimuth,
      x: (x(offset) / STAGE_W) * 100,
      y: (Math.max(0, y(peak.angle)) / STAGE_H) * 100,
    }))
    .sort((a, b) => b.ele - a.ele);

  // The tallest ones that don't run into each other, away from the edges where a name would be cut.
  const chosen: StagePeak[] = [];
  for (const peak of peaks) {
    if (chosen.length === LABELS) break;
    if (peak.x < 6 || peak.x > 94) continue;
    if (chosen.every((other) => Math.abs(other.x - peak.x) > MIN_LABEL_GAP)) chosen.push(peak);
  }
  const labels = chosen.sort((a, b) => a.x - b.x).map((peak, i) => ({ ...peak, tier: i % 2 }));

  return { layers, ridges, skyline: `M${trace(skyline)}`, labels, peaks, center, span: SPAN, highest: peaks[0] ?? null };
}
