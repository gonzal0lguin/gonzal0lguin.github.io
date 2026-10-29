// The summit registry (src/content/summits/*.yaml) as the editor sees it, plus everything the
// scripts computed from each entry: the panorama of the view from the top, the two images it
// draws with, and the outline of the summit for the to-scale chart.
//
// The registry is the one part of the content that is not Markdown and has no translation: one
// data file per summit, read by Astro as the `summits` collection, by scripts/lib/ascents.mjs,
// and listed here with the ascent posts that point at it (`summits:` in their frontmatter) so
// the editor can show what a change would affect.
//
// Nothing here computes anything. A panorama takes half a minute of DEM and Overpass work, so
// the editor only reports whether one exists, whether it is older than the file it came from,
// and runs `npm run panoramas` / `npm run profiles` as a task (editor/server/tasks.mjs).
import path from "node:path";
import { parse } from "yaml";
import { listEntries } from "./content.mjs";
import { HttpError, bad, exists, fileInfo, readTextFile, relative, safePath, trash, walk, writeFileWithBackup } from "./util.mjs";

export const SUMMITS_DIR = "src/content/summits";
/** What `npm run panoramas` writes: the data of the view, and the images the page draws. */
const PANORAMA_DIR = "src/data/panoramas";
const PANORAMA_ASSETS = "public/assets/panoramas";
/** What `npm run profiles` writes: every summit's outline, in one file. */
const PROFILES = "src/data/profiles.json";
const PUBLIC_DIR = "public";

const YAML = [".yaml", ".yml"];
const isYaml = (file) => YAML.includes(path.extname(file).toLowerCase());

/**
 * How far the computed viewpoint may sit from the coordinates in the file before the panorama
 * counts as belonging to another place: the panorama snaps the viewpoint to the top of the
 * elevation model, a few hundred metres at most, so only a real move shows up.
 */
const MOVED = 0.01;

export const summitPath = (id) => `${SUMMITS_DIR}/${id}.yaml`;

/** The registry, with what each entry's panorama and outline look like on disk. */
export async function listSummits(root, collections) {
  const files = await walk(root, path.join(root, SUMMITS_DIR), isYaml);
  const links = ascentLinks(collections ?? (await listEntries(root)));
  const profiles = await readProfiles(root);

  const summits = [];
  for (const file of files) {
    const id = path.basename(file).replace(/\.ya?ml$/i, "");
    const text = await readTextFile(path.join(root, file));
    let data = {};
    let error = null;
    try {
      data = parse(text) ?? {};
    } catch (problem) {
      error = problem.message;
    }
    const info = await fileInfo(root, file);
    summits.push({
      id,
      path: file,
      mtime: info.mtime,
      error,
      name: data.name ?? id,
      elevation: typeof data.elevation === "number" ? data.elevation : null,
      lat: typeof data.lat === "number" ? data.lat : null,
      lon: typeof data.lon === "number" ? data.lon : null,
      region: data.region ?? null,
      osm: data.osm ?? null,
      wikidata: data.wikidata ?? null,
      wants: data.panorama === true,
      timezone: data.timezone ?? null,
      photos: Array.isArray(data.photos) ? data.photos.length : 0,
      ascents: links.get(id) ?? [],
      panorama: await panoramaState(root, id, data, info.mtime),
      profile: profiles[id] ? { from: profiles[id].from ?? null, side: profiles[id].side ?? null, points: profiles[id].points?.length ?? 0 } : null,
    });
  }
  summits.sort((a, b) => (b.elevation ?? 0) - (a.elevation ?? 0) || a.id.localeCompare(b.id));
  return summits;
}

/**
 * What `npm run panoramas` left for this summit. `stale` means the YAML file was edited after
 * the panorama was computed, `moved` that it now points somewhere else entirely — either way
 * the numbers on the page are the old ones until the panorama is run again.
 */
async function panoramaState(root, id, data, yamlMtime) {
  const dataPath = `${PANORAMA_DIR}/${id}.json`;
  const info = await fileInfo(root, dataPath);
  if (info.missing) return null;
  let viewpoint = null;
  let generated = null;
  try {
    const json = JSON.parse(await readTextFile(path.join(root, dataPath)));
    viewpoint = json.viewpoint ?? null;
    generated = json.generated ?? null;
  } catch {
    return { path: dataPath, broken: true, generated: null, mtime: info.mtime, stale: true, moved: false, images: [] };
  }
  const images = [];
  for (const kind of ["terrain", "viewshed"]) {
    const rel = `${PANORAMA_ASSETS}/${id}/${kind}.png`;
    images.push({ kind, path: rel, publicPath: `/${rel.slice(PUBLIC_DIR.length + 1)}`, ...(await fileInfo(root, rel)) });
  }
  return {
    path: dataPath,
    broken: false,
    generated,
    mtime: info.mtime,
    stale: info.mtime < yamlMtime,
    moved:
      viewpoint != null &&
      typeof data.lat === "number" &&
      typeof data.lon === "number" &&
      (Math.abs(viewpoint.lat - data.lat) > MOVED || Math.abs(viewpoint.lon - data.lon) > MOVED),
    images,
  };
}

/** The ascent posts of each summit id, newest first — the same link the site builds from. */
function ascentLinks(collections) {
  const links = new Map();
  for (const entry of collections.find((collection) => collection.id === "ascents")?.entries ?? []) {
    for (const id of entry.summits) {
      if (!links.has(id)) links.set(id, []);
      links.get(id).push({ key: entry.key, path: entry.path, title: entry.title, pubDate: entry.pubDate, attempt: entry.attempt });
    }
  }
  return links;
}

async function readProfiles(root) {
  try {
    return JSON.parse(await readTextFile(path.join(root, PROFILES)));
  } catch {
    return {};
  }
}

export async function readSummit(root, rel) {
  const full = safePath(root, rel ?? "", { extensions: YAML });
  const text = await readTextFile(full);
  const info = await fileInfo(root, relative(root, full));
  return { path: relative(root, full), text, mtime: info.mtime };
}

/** Overwrite a registry file, with the same stale-save guard content files get. */
export async function saveSummit(root, { path: rel, text, mtime, force }) {
  const full = safePath(root, rel, { extensions: YAML });
  if (typeof text !== "string") throw bad("Missing file content");
  if (!rel.startsWith(`${SUMMITS_DIR}/`)) throw bad(`Only files under ${SUMMITS_DIR} can be saved here`);
  if (await exists(full)) {
    const current = (await fileInfo(root, relative(root, full))).mtime;
    if (!force && mtime && current !== mtime) {
      throw new HttpError(409, `${rel} changed on disk since it was opened. Reload it, or save again to overwrite.`);
    }
  }
  const result = await writeFileWithBackup(root, full, text);
  return { ...result, mtime: (await fileInfo(root, relative(root, full))).mtime };
}

export async function createSummit(root, { id, text }) {
  if (!/^[a-z0-9][a-z0-9-]*$/.test(id ?? "")) throw bad("The summit's file name may only contain lowercase letters, numbers and dashes");
  const rel = summitPath(id);
  const full = safePath(root, rel, { extensions: YAML });
  if (await exists(full)) throw bad(`${rel} already exists`);
  const result = await writeFileWithBackup(root, full, text ?? "");
  return { ...result, id, mtime: (await fileInfo(root, rel)).mtime };
}

/**
 * Move a summit out of the registry, together with what was computed for it. The ascent posts
 * that point at it are left alone: the caller lists them first, because removing the reference
 * is an edit to the post, not to the registry.
 */
export async function deleteSummit(root, { id, withPanorama = true }) {
  if (!/^[a-z0-9][a-z0-9-]*$/.test(id ?? "")) throw bad("Unknown summit");
  const moved = [];
  const take = async (rel) => {
    const full = path.join(root, rel);
    if (await exists(full)) moved.push(await trash(root, full));
  };
  await take(summitPath(id));
  if (withPanorama) {
    await take(`${PANORAMA_DIR}/${id}.json`);
    for (const kind of ["terrain", "viewshed"]) await take(`${PANORAMA_ASSETS}/${id}/${kind}.png`);
  }
  if (moved.length === 0) throw bad(`${summitPath(id)} not found`);
  return { trashed: moved };
}

/** Summit ids and the photo paths they point at, for the checks in health.mjs. */
export async function summitPhotos(root) {
  const files = await walk(root, path.join(root, SUMMITS_DIR), isYaml);
  const out = [];
  for (const file of files) {
    let data = {};
    try {
      data = parse(await readTextFile(path.join(root, file))) ?? {};
    } catch {
      continue;
    }
    for (const photo of Array.isArray(data.photos) ? data.photos : []) {
      out.push({ id: path.basename(file).replace(/\.ya?ml$/i, ""), path: file, name: data.name ?? file, src: photo?.src ?? "", azimuth: photo?.azimuth });
    }
  }
  return out;
}
