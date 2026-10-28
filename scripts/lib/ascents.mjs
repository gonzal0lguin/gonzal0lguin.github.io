// The log as the build scripts see it: each ascent post's frontmatter and GPS track, and the
// summit registry in src/content/summits/ (one YAML file per summit, the same files Astro reads
// as the "summits" collection).

import { readFileSync, readdirSync } from "node:fs";
import { parse } from "yaml";

const ASCENTS = new URL("../../src/content/ascents/", import.meta.url);
const SUMMITS = new URL("../../src/content/summits/", import.meta.url);
const PUBLIC = new URL("../../public/", import.meta.url);

const frontmatter = (text) => {
  const head = text.match(/^---\r?\n([\s\S]*?)\r?\n---/);
  return head ? parse(head[1]) ?? {} : {};
};

/**
 * The track of one or several GPX files as separate segments (one per <trkseg>), in the order
 * they were recorded — like src/lib/gpx.ts, which doesn't trust the order of the files either.
 * Nothing joins the end of one segment to the start of the next: that gap wasn't walked.
 */
export function trackSegments(files) {
  const segments = [files].flat().flatMap((file) => {
    const gpx = readFileSync(new URL(`.${file}`, PUBLIC), "utf8");
    return gpx.split(/<trkseg\b[^>]*>/).slice(1).map((segment) =>
      [...segment.matchAll(/<trkpt\b([^>]*)>([\s\S]*?)<\/trkpt>|<trkpt\b([^>]*)\/>/g)].map((match) => {
        const attributes = match[1] ?? match[3];
        const time = match[2]?.match(/<time>([^<]+)<\/time>/)?.[1];
        return {
          lat: Number(attributes.match(/lat="([-\d.]+)"/)[1]),
          lon: Number(attributes.match(/lon="([-\d.]+)"/)[1]),
          time: time ? Date.parse(time) : null,
        };
      }),
    );
  }).filter((segment) => segment.length > 1);
  const start = (segment) => segment.find((point) => point.time != null)?.time ?? Infinity;
  return segments
    .map((segment, i) => ({ segment, i }))
    .sort((a, b) => start(a.segment) - start(b.segment) || a.i - b.i)
    .map(({ segment }) => segment);
}

/** The Spanish ascent posts (the English ones share their frontmatter), sorted by slug. */
export function readAscents() {
  return readdirSync(ASCENTS)
    .filter((file) => /\.mdx?$/.test(file))
    .sort()
    .map((file) => {
      const meta = frontmatter(readFileSync(new URL(file, ASCENTS), "utf8"));
      return {
        slug: file.replace(/\.mdx?$/, ""),
        title: meta.title,
        date: meta.pubDate instanceof Date ? meta.pubDate.toISOString().slice(0, 10) : String(meta.pubDate ?? ""),
        summits: meta.summits ?? [],
        route: meta.route ?? null,
      };
    });
}

/** Every ascent with a GPS track, as routesInView (scripts/lib/viewshed.mjs) takes them. */
export const readTracks = () =>
  readAscents()
    .filter((ascent) => ascent.route)
    .map((ascent) => ({ ...ascent, segments: trackSegments(ascent.route) }));

/** The summit registry: { id, name, lat, lon, elevation, osm?, panorama?, ... } per file. */
export function readSummits() {
  return readdirSync(SUMMITS)
    .filter((file) => /\.ya?ml$/.test(file))
    .sort()
    .map((file) => ({ id: file.replace(/\.ya?ml$/, ""), ...parse(readFileSync(new URL(file, SUMMITS), "utf8")) }));
}

/**
 * The registry summit a computed peak is, if any: the same OSM node, or within 300 m of the
 * registry's coordinates. Returns its id.
 */
export function registryMatcher(summits, bearingDistance) {
  return (peak) =>
    summits.find((summit) => (summit.osm && summit.osm === peak.osm) || bearingDistance(summit, peak).distance < 300)?.id ?? null;
}

/** Tags each peak that is a summit of the log with its registry id and (first) ascent post. */
export function linkRegistry(peaks, summits, ascents, bearingDistance) {
  const match = registryMatcher(summits, bearingDistance);
  for (const peak of peaks) {
    const id = match(peak);
    if (!id) continue;
    peak.summit = id;
    const ascent = ascents.find((a) => a.summits.includes(id));
    if (ascent) peak.ascent ??= { slug: ascent.slug, title: ascent.title };
  }
}
