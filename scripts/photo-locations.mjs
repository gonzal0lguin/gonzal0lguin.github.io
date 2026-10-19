// Saves where and when each post photo was taken (from its EXIF data) to
// src/data/photo-locations.json, so the route maps can show the photos even after the files are
// resized, stripped of metadata or moved to another host.
//
//   npm run photos                    reads public/assets/img/posts (the default photo folder)
//   npm run photos -- ~/Fotos/web     reads originals kept elsewhere, laid out like the posts
//                                     reference them ("la-cruz/summit.jpg")
//
// Entries are merged: photos that are no longer in the folder keep their saved location.
// Runs automatically before `npm run dev` and `npm run build`.

import { readdir, readFile, writeFile, mkdir } from "node:fs/promises";
import path from "node:path";
import exifr from "exifr";

const root = path.resolve(process.argv[2] ?? "public/assets/img/posts");
const output = path.resolve("src/data/photo-locations.json");
const PHOTO = /\.(jpe?g|heic|heif|tiff?|png|webp|avif)$/i;

async function* walk(dir) {
  let entries;
  try {
    entries = await readdir(dir, { withFileTypes: true });
  } catch {
    return;
  }
  for (const entry of entries) {
    const full = path.join(dir, entry.name);
    if (entry.isDirectory()) yield* walk(full);
    else if (PHOTO.test(entry.name)) yield full;
  }
}

const saved = JSON.parse(await readFile(output, "utf8").catch(() => "{}"));
let found = 0;
for await (const file of walk(root)) {
  const key = path.relative(root, file).split(path.sep).join("/");
  let tags;
  try {
    // Raw values: the date stays as written by the camera, without this machine's timezone.
    tags = await exifr.parse(file, { gps: true, reviveValues: false, pick: ["DateTimeOriginal", "OffsetTimeOriginal", "GPSLatitude", "GPSLatitudeRef", "GPSLongitude", "GPSLongitudeRef", "GPSAltitude", "GPSAltitudeRef", "latitude", "longitude"] });
  } catch {
    continue;
  }
  if (!tags) continue;
  const entry = {};
  if (Number.isFinite(tags.latitude) && Number.isFinite(tags.longitude) && !(tags.latitude === 0 && tags.longitude === 0)) {
    entry.lat = Math.round(tags.latitude * 1e6) / 1e6;
    entry.lon = Math.round(tags.longitude * 1e6) / 1e6;
    const altitude = Number(tags.GPSAltitude);
    if (Number.isFinite(altitude)) entry.ele = Math.round(altitude * (tags.GPSAltitudeRef === 1 ? -1 : 1));
  }
  // "2026:10:18 09:12:33" → "2026-10-18T09:12:33", plus the UTC offset when the camera saved one.
  const taken = typeof tags.DateTimeOriginal === "string" ? tags.DateTimeOriginal.trim() : null;
  if (taken && /^\d{4}:\d\d:\d\d \d\d:\d\d/.test(taken)) {
    entry.time = taken.replace(/^(\d{4}):(\d\d):(\d\d) /, "$1-$2-$3T") + (typeof tags.OffsetTimeOriginal === "string" ? tags.OffsetTimeOriginal.trim() : "");
  }
  if (Object.keys(entry).length) {
    saved[key] = entry;
    found++;
  }
}

const sorted = Object.fromEntries(Object.entries(saved).sort(([a], [b]) => a.localeCompare(b)));
await mkdir(path.dirname(output), { recursive: true });
await writeFile(output, JSON.stringify(sorted, null, 2) + "\n");
console.log(`photo-locations: ${found} photo(s) with location or time in ${path.relative(process.cwd(), root) || "."}, ${Object.keys(sorted).length} saved in total`);
