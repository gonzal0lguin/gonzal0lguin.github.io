// The summits of the log, joined with everything about them: the registry entry
// (src/content/summits/), the ascent posts that reach each one, and the view from the top that
// scripts/panoramas.mjs computed (src/data/panoramas/<id>.json). A summit climbed three times is
// one summit here with three ascents, which is what the summit pages and the index are built on.

import { getCollection, type CollectionEntry } from "astro:content";
import type { Peak, Route } from "../data/hill";
import { getLocalizedCollection } from "../i18n/content";
import type { Lang } from "../i18n/ui";
import { loadRoute } from "./gpx";

export interface Glacier {
  name: string | null;
  type: string | null;
  lat: number;
  lon: number;
  /** km² */
  area: number;
  visibleArea: number;
  distance: number;
  azimuth: number;
  /** Elevation angle of the highest ice in view. */
  angle: number;
}

export interface Place {
  name: string;
  kind: string;
  population: number | null;
  distance: number;
  azimuth: number;
  visible: boolean;
}

/** The computed view from a summit; the hill's src/data/panorama.json has the same core. */
export interface PanoramaData {
  generated: string;
  summit?: { id: string; name: string };
  viewpoint: { lat: number; lon: number; elevation: number; eye: number };
  stats: {
    visibleArea: number;
    visibleShare: number;
    above3000: number;
    farthest: { distance: number; azimuth: number; lat: number; lon: number; elevation: number };
    highestSkyline: number;
    lowestSkyline: number;
    peaksInView?: number;
    peaksAround?: number;
    highest?: { name: string; ele: number; distance: number; azimuth: number };
    farthestPeak?: { name: string; ele: number; distance: number; azimuth: number };
    sea?: { distance: number; azimuth: number } | null;
  };
  bands: number[];
  skyline: number[][];
  bandSkylines: number[][][];
  ridges: { distance: number; points: number[][] }[];
  peaks: Peak[];
  routes: Route[];
  mesh: { cols: number; rings: number; azStep: number; r0: number; ratio: number; effectiveRadius: number; file: string };
  viewshed: { south: number; north: number; west: number; east: number; file: string };
  glaciers?: { count: number; area: number; visibleCount: number; visibleArea: number; list: Glacier[] };
  places?: Place[];
  lakes?: { name: string; kind: string; distance: number; azimuth: number; visible: boolean }[];
  mine?: { id: string; name: string; distance: number; azimuth: number; status: Peak["status"] }[];
}

const PANORAMAS = import.meta.glob<PanoramaData>("../data/panoramas/*.json", { eager: true, import: "default" });

export function getPanorama(id: string): PanoramaData | null {
  return PANORAMAS[`../data/panoramas/${id}.json`] ?? null;
}

export interface SummitAscent {
  key: string;
  title: string;
  date: Date;
  attempt: boolean;
  heroImage?: string;
  /** Distance (m) and gain of the GPS route, when the post has one. */
  route: { distance: number; gain: number | null } | null;
}

export interface Summit {
  id: string;
  data: CollectionEntry<"summits">["data"];
  ascents: SummitAscent[];
  /** Newest ascent first. */
  last: Date | null;
  panorama: PanoramaData | null;
}

/** Summit ids of each ascent post, by key. They're set in the Spanish post; translations share them. */
async function summitsByAscent() {
  const entries = await getCollection("ascents", (entry) => !entry.slug.startsWith("en/"));
  return new Map(entries.map((entry) => [entry.slug, { ids: (entry.data.summits ?? []).map((ref) => ref.id), attempt: !!entry.data.attempt }]));
}

export async function ascentSummitIds(key: string): Promise<string[]> {
  return (await summitsByAscent()).get(key)?.ids ?? [];
}

// Built once per language for a production build; in dev, every page asks again so edits show up.
const cache = new Map<Lang, Summit[]>();

/** Every summit of the registry with its ascents in `lang`, highest first. */
export async function getSummits(lang: Lang): Promise<Summit[]> {
  if (cache.has(lang)) return cache.get(lang)!;
  const [registry, ascents, links] = await Promise.all([getCollection("summits"), getLocalizedCollection("ascents", lang), summitsByAscent()]);
  const summits = registry.map((entry) => {
    const visits = ascents
      .filter(({ key }) => links.get(key)?.ids.includes(entry.id))
      .map(({ key, entry: post }) => {
        const route = post.data.route ? loadRoute(post.data.route).stats : null;
        return {
          key,
          title: post.data.title,
          date: post.data.pubDate,
          attempt: !!links.get(key)?.attempt,
          heroImage: post.data.heroImage,
          route: route && { distance: route.distance, gain: route.gain },
        };
      })
      .sort((a, b) => b.date.valueOf() - a.date.valueOf());
    return { id: entry.id, data: entry.data, ascents: visits, last: visits[0]?.date ?? null, panorama: entry.data.panorama ? getPanorama(entry.id) : null };
  });
  summits.sort((a, b) => b.data.elevation - a.data.elevation);
  if (import.meta.env.PROD) cache.set(lang, summits);
  return summits;
}

/** The skyline in `bins` steps of bearing (the highest angle in each), for small drawings. */
export function skylineProfile(panorama: PanoramaData, bins = 120): number[] {
  const out = new Array(bins).fill(-90);
  const line = panorama.skyline;
  for (let i = 1; i < line.length; i++) {
    const [a0, v0] = line[i - 1];
    const [a1, v1] = line[i];
    for (let az = a0; az <= a1; az += 360 / bins / 4) {
      const v = v0 + ((v1 - v0) * (az - a0)) / (a1 - a0 || 1);
      const bin = Math.min(bins - 1, Math.floor((az / 360) * bins));
      if (v > out[bin]) out[bin] = v;
    }
  }
  return out.map((v, i) => (v === -90 ? out[(i + bins - 1) % bins] : v));
}

/** Snowline by season for a latitude: the central Andes values, lowered toward the south. */
export function snowlines(lat: number) {
  const central = { summer: 4400, autumn: 3500, winter: 2300, spring: 2900 };
  const south = { summer: 1900, autumn: 1500, winter: 900, spring: 1300 };
  const t = Math.min(1, Math.max(0, (Math.abs(lat) - 34) / 7));
  return Object.fromEntries(Object.entries(central).map(([season, value]) => [season, Math.round(value + (south[season as keyof typeof south] - value) * t)]));
}
