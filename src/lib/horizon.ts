// What all the summit views say when they're read together, rather than one at a time. Each
// panorama (src/data/panoramas/<id>.json) lists the named summits it can see and which of my
// other summits show; adding those up gives two things no single view has: the mountains I keep
// looking at and have never climbed, and which of my summits can see each other.

import type { Peak } from "../data/hill";
import type { Summit } from "./summits";

export interface WishPeak {
  osm: number;
  name: string;
  ele: number;
  /** How each of my summits sees it, by summit id; a summit that doesn't appear can't. */
  from: Record<string, Peak["status"]>;
  /** Number of my summits it's clearly in view from. */
  seen: number;
  /** The nearest I've stood to it, km. */
  nearest: number;
  nearestFrom: string;
}

const ROWS = 90;
const MIN_ELE = 1000;
const MIN_SEEN = 2;
const plain = (name: string) => name.normalize("NFD").replace(/[̀-ͯ]/g, "").toLowerCase().replace(/^(cerro|volcan|nevado|punta) /, "");

/** The named summits in sight from mine that I haven't reached, most looked-at first. */
export function seenNotClimbed(summits: Summit[]) {
  const viewpoints = summits.filter((summit) => summit.panorama);
  const climbed = summits.filter((summit) => summit.ascents.some((ascent) => !ascent.attempt));
  const climbedOsm = new Set(climbed.flatMap((summit) => summit.data.osm ?? []));
  const climbedNames = new Set(climbed.map((summit) => plain(summit.data.name)));

  const peaks = new Map<number, WishPeak>();
  for (const { id, panorama } of viewpoints) {
    for (const peak of panorama!.peaks) {
      if (!peak.name || !peak.ele || !peak.osm || peak.status === "hidden") continue;
      const row = peaks.get(peak.osm) ?? { osm: peak.osm, name: peak.name, ele: peak.ele, from: {}, seen: 0, nearest: Infinity, nearestFrom: id };
      row.from[id] = peak.status;
      if (peak.status === "visible") row.seen++;
      if (peak.distance < row.nearest) Object.assign(row, { nearest: peak.distance, nearestFrom: id });
      peaks.set(peak.osm, row);
    }
  }

  const all = [...peaks.values()];
  const missing = all.filter((peak) => !climbedOsm.has(peak.osm) && !climbedNames.has(plain(peak.name)));
  // The high ones are the interesting ones, so they get their places in the table first.
  const rank = (a: WishPeak, b: WishPeak) => b.seen - a.seen || b.ele - a.ele;
  const eligible = missing.filter((peak) => peak.seen >= MIN_SEEN && peak.ele >= MIN_ELE);
  const high = eligible.filter((peak) => peak.ele >= 3000).sort(rank);
  const low = eligible.filter((peak) => peak.ele < 3000).sort(rank);
  return {
    viewpoints,
    distinct: all.length,
    notClimbed: missing.length,
    rows: [...high, ...low].slice(0, ROWS),
  };
}

export interface SightEdge {
  a: string;
  b: string;
  /** "visible" when at least one of the two views clearly shows the other. */
  status: "visible" | "borderline";
  distance: number;
}

/** Which of my summits can see each other. A pair is one line, whichever view reported it. */
export function summitSightlines(summits: Summit[]) {
  const edges = new Map<string, SightEdge>();
  for (const { id, panorama } of summits) {
    for (const other of panorama?.mine ?? []) {
      if (other.status === "hidden") continue;
      const [a, b] = [id, other.id].sort();
      const key = `${a}|${b}`;
      const known = edges.get(key);
      if (!known) edges.set(key, { a, b, status: other.status === "visible" ? "visible" : "borderline", distance: other.distance });
      else if (other.status === "visible") known.status = "visible";
    }
  }
  const list = [...edges.values()];
  const degree = (id: string) => list.filter((edge) => edge.a === id || edge.b === id).length;
  const nodes = summits
    .filter((summit) => summit.ascents.length)
    .map((summit) => ({ summit, degree: degree(summit.id) }))
    .sort((a, b) => b.degree - a.degree || b.summit.data.elevation - a.summit.data.elevation);
  return { edges: list, connected: nodes.filter((node) => node.degree > 0), alone: nodes.filter((node) => node.degree === 0) };
}
