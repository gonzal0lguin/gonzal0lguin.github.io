// The checks behind the editor's "Revisión" view: the mistakes that only show up at build
// time (or not at all), found by walking the content and the files it points at.
import path from "node:path";
import { listEntries } from "./content.mjs";
import { POSTS_DIR, PUBLIC_DIR, collectUsage } from "./media.mjs";
import { listSummits, summitPhotos } from "./summits.mjs";
import { exists, readTextFile, splitFile, walk } from "./util.mjs";

const IMAGE = /\.(jpe?g|png|webp|avif|gif|heic|heif|tiff?)$/i;
const REMOTE = /^[a-z][a-z0-9+.-]*:|^\/\//i;

export async function health(root, mediaBaseUrl) {
  const issues = [];
  const add = (level, kind, message, entry) => issues.push({ level, kind, message, ...entry });
  const remoteMedia = REMOTE.test(mediaBaseUrl ?? "");

  const collections = await listEntries(root);
  // Which file names the content mentions at all — in Markdown, in a photo block or in raw
  // HTML (<img src="...">), which some posts use.
  const mentioned = await collectUsage(root);

  for (const collection of collections) {
    for (const entry of collection.entries) {
      const where = { path: entry.path, collection: entry.collection, key: entry.key, title: entry.title };
      const text = await readTextFile(path.join(root, entry.path));

      // Standalone pages have neither a date nor tags in their schema.
      if (collection.id !== "pages") {
        if (!entry.description) add("warn", "description", "No description (used in listings and link previews)", where);
        if (!entry.pubDate) add("error", "pubDate", "No pubDate", where);
        if (entry.tags.length === 0) add("info", "tags", "No tags", where);
      }

      if (entry.heroImage && !REMOTE.test(entry.heroImage)) {
        const file = path.join(root, PUBLIC_DIR, entry.heroImage.replace(/^\/+/, ""));
        if (!(await exists(file))) add("error", "heroImage", `heroImage not found: ${entry.heroImage}`, where);
      }

      for (const route of entry.route) {
        const file = path.join(root, PUBLIC_DIR, route.replace(/^\/+/, ""));
        if (!(await exists(file))) add("error", "route", `Route file not found: ${route}`, where);
      }

      const { body } = splitFile(text);
      const sources = [
        ...[...body.matchAll(/!\[[^\]]*\]\(([^)\s]+)(?:\s+"[^"]*")?\)/g)].map((match) => match[1]),
        // Raw HTML: a few posts place an image with <img src="..."> for the width attribute.
        ...[...body.matchAll(/<img\b[^>]*?\ssrc=["']([^"']+)["']/gi)].map((match) => match[1]),
      ];
      for (const src of sources) {
        const [photo] = src.split("#");
        if (REMOTE.test(photo) || photo === "") continue;
        if (photo.startsWith("/")) {
          if (!(await exists(path.join(root, PUBLIC_DIR, photo.replace(/^\/+/, ""))))) add("error", "photo", `Photo not found: ${photo}`, where);
        } else if (photo.startsWith("./")) {
          if (!(await exists(path.join(root, path.dirname(entry.path), photo)))) add("error", "photo", `Photo not found: ${photo}`, where);
        } else if (!remoteMedia) {
          if (!(await exists(path.join(root, POSTS_DIR, photo)))) add("error", "photo", `Photo not found: ${POSTS_DIR}/${photo}`, where);
        }
      }

      if (!entry.translation) add("info", "translation", "No English version yet", where);
      else if (entry.translation.state === "stale") add("warn", "translation", "English version is out of date", where);
    }
  }

  await checkSummits(root, collections, add);

  // Photos nobody uses: safe to delete, and easy to lose track of. By file name, so a photo
  // placed with raw HTML or an unusual path still counts as used.
  const photos = await walk(root, path.join(root, POSTS_DIR), (file) => IMAGE.test(file));
  const unused = photos.filter((photo) => !mentioned.has(path.basename(photo)));

  const order = { error: 0, warn: 1, info: 2 };
  issues.sort((a, b) => order[a.level] - order[b.level] || a.path.localeCompare(b.path));
  return {
    issues,
    unusedPhotos: unused,
    counts: {
      entries: collections.reduce((total, c) => total + c.entries.length, 0),
      errors: issues.filter((i) => i.level === "error").length,
      warnings: issues.filter((i) => i.level === "warn").length,
    },
  };
}

/**
 * The summit registry and what the scripts computed from it. A missing panorama or outline is
 * not a broken build — the summit page simply has no view and the chart no silhouette — so
 * most of this is a warning with the command that fixes it.
 */
async function checkSummits(root, collections, add) {
  const summits = await listSummits(root, collections);
  const known = new Set(summits.map((summit) => summit.id));

  for (const entry of collections.find((collection) => collection.id === "ascents")?.entries ?? []) {
    const where = { path: entry.path, collection: entry.collection, key: entry.key, title: entry.title };
    for (const id of entry.summits) {
      // A reference to a file that is not there fails the build: Astro resolves it as a collection entry.
      if (!known.has(id)) add("error", "summit", `summits: points at ${id}, which has no file in src/content/summits`, where);
    }
  }

  for (const summit of summits) {
    const where = { path: summit.path, collection: "summits", key: summit.id, title: summit.name };
    if (summit.error) {
      add("error", "summit", `Not valid YAML: ${summit.error}`, where);
      continue;
    }
    for (const key of ["name", "elevation", "lat", "lon"]) {
      if (summit[key] == null) add("error", "summit", `Missing ${key} (the schema requires it)`, where);
    }
    if (!summit.region) add("info", "summit", "No region (it labels the summit on its page and in the list)", where);
    if (summit.ascents.length === 0) add("info", "summit", "No ascent points at this summit yet", where);

    if (summit.wants && !summit.panorama) {
      const north = (summit.lat ?? 0) > 0 || (summit.lon ?? 0) > 0;
      add(
        north ? "info" : "warn",
        "panorama",
        north
          ? "panorama: true, but panoramas north or east of 0° are skipped — it gets its outline and no view"
          : `panorama: true and nothing computed yet: npm run panoramas -- ${summit.id}`,
        where,
      );
    }
    if (!summit.wants && summit.panorama) add("info", "panorama", "panorama: false, so the computed view in src/data/panoramas is not shown", where);
    if (summit.panorama?.broken) add("error", "panorama", `${summit.panorama.path} is not valid JSON: npm run panoramas -- ${summit.id}`, where);
    else if (summit.panorama?.moved) add("warn", "panorama", `The panorama was computed from somewhere else: npm run panoramas -- ${summit.id}`, where);
    else if (summit.panorama?.stale) add("info", "panorama", `The file changed after the panorama was computed: npm run panoramas -- ${summit.id}`, where);
    for (const image of summit.panorama?.images ?? []) {
      if (image.missing) add("error", "panorama", `${image.path} is missing (the panorama's data points at it): npm run panoramas -- ${summit.id}`, where);
    }
    if (!summit.profile) add("warn", "profile", "No outline for the to-scale chart: npm run profiles", where);
  }

  for (const photo of await summitPhotos(root)) {
    const where = { path: photo.path, collection: "summits", key: photo.id, title: photo.name };
    if (!photo.src) add("error", "summit", "A summit photo has no src", where);
    else if (!REMOTE.test(photo.src) && !(await exists(path.join(root, PUBLIC_DIR, photo.src.replace(/^\/+/, ""))))) {
      add("error", "summit", `Summit photo not found: ${photo.src}`, where);
    }
    if (photo.src && photo.azimuth == null) add("info", "summit", `${photo.src} has no azimuth, so it is not pinned on the panorama`, where);
  }
}
