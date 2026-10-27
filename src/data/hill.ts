// What the terrain says about the hill and the view from it, computed rather than remembered.
//
// Everything here comes from src/data/panorama.json, which scripts/terrain.mjs writes from the
// Copernicus GLO-30 elevation model (30 m cells) and the named summits in OpenStreetMap. 30 m
// cells are coarse for a hill this small: the hill figures are the shape of the ground to the
// nearest cell, not a survey. The post says so where it matters.

import panorama from "./panorama.json";

export type PeakStatus =
  /** The terrain model puts the summit clearly above whatever stands in front of it. */
  | "visible"
  /** Within 0.06° of the ground in front — inside the model's own error, so the eye decides. */
  | "borderline"
  /** A nearer ridge is clearly in the way. */
  | "hidden";

export interface Peak {
  osm: number;
  name: string;
  /** Official height, from OpenStreetMap, when it has one. */
  ele: number | null;
  /** Height of the highest model cell within 150 m of the OSM point. */
  model: number;
  lat: number;
  lon: number;
  /** Kilometres from the hilltop. */
  distance: number;
  /** Bearing from the hilltop, degrees clockwise from north. */
  azimuth: number;
  /** Apparent elevation angle of the summit, in degrees above the horizontal. */
  angle: number;
  /** Highest angle of the ground between the hill and the summit. */
  horizon: number;
  /** angle − horizon. */
  margin: number;
  /** Metres of the mountain that show above whatever is in front of it. */
  showing: number;
  /** Seen against the sky rather than against a bigger mountain behind it. */
  skyline: boolean;
  status: PeakStatus;
  /** The post names it. */
  named: boolean;
  /** The higher summit it is a shoulder of, if any. */
  parent: string | null;
  /** The named summit whose flank hides it, if any. */
  hiddenBy?: string | null;
  /** For a hidden summit the post names: the highest visible ground of the same mountain. */
  flank?: { lat: number; lon: number; elevation: number; away: number; azimuth: number; angle: number } | null;
  /** My ascent post for this summit, if there is one. */
  ascent?: { slug: string; title: string };
  kind: string;
  wikidata: string | null;
}

export interface RouteRun {
  visible: boolean;
  /** [azimuth, elevation angle] pairs, as seen from the hilltop. */
  points: number[][];
}

export interface Route {
  slug: string;
  title: string;
  date: string;
  /** Nearest and farthest point of the track from the hilltop, km. */
  distance: number[];
  length: number;
  /** Share of the track's length in view from the hilltop. */
  visibleShare: number;
  summits: string[];
  runs: RouteRun[];
}

const { grid, ...hill } = panorama.hill;

export const HILL = hill;
/** The model around the hill, one value per arc-second node, row by row from the north-west. */
export const HILL_GRID = grid;
export const SUMMIT = panorama.hill.summit;
export const PLANTING_CENTRE = panorama.hill.planting;
export const VIEWPOINT = panorama.viewpoint;
export const VIEW_STATS = panorama.stats;
export const PEAKS = panorama.peaks as Peak[];
export const ROUTES = panorama.routes as Route[];
export const PANORAMA = panorama;
