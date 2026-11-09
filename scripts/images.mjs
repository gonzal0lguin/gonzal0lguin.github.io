#!/usr/bin/env node
// Resized WebP copies of the site's photos, in the widths a page actually needs.
//
// Reads every JPEG, PNG and WebP under public/assets/img/ and writes up to four copies to
// public/assets/opt/, with a list of what exists in .cache/image-variants.json. Pages ask for
// a photo by its original path (src/lib/image-variants.mjs) and get a srcset back.
//
// Astro's <Image> can't do this job here: photos are referenced by public path, from
// frontmatter and from Markdown, and it hands those through untouched. That is how a 136 px
// avatar came to weigh 2.2 MB. Moving them all into src/ would break the photo library's
// PUBLIC_MEDIA_BASE_URL and the editor, so the copies are made next to the originals instead.
//
// Runs before `dev` and `build`. Both outputs are gitignored: they are derived, every version
// of a photo stays in git history forever, and CI remakes them from the originals in under a
// minute. Locally a photo is only redone when its file changes. A photo that isn't in the list
// (one on a CDN, say) is simply served as it is.

import { existsSync, mkdirSync, readFileSync, readdirSync, rmSync, statSync, writeFileSync } from "node:fs";
import path from "node:path";
import sharp from "sharp";

const SOURCE = "public/assets/img";
const OUTPUT = "public/assets/opt";
const MANIFEST = ".cache/image-variants.json";
const WIDTHS = [320, 640, 960, 1600];
const QUALITY = 78;
const WORKERS = 4;

const publicPath = (file) => `/${path.relative("public", file).split(path.sep).join("/")}`;
const variantFile = (file, width) => path.join(OUTPUT, `${path.relative(SOURCE, file)}.${width}.webp`);

function photos(dir) {
  if (!existsSync(dir)) return [];
  return readdirSync(dir, { withFileTypes: true }).flatMap((entry) => {
    const file = path.join(dir, entry.name);
    if (entry.isDirectory()) return photos(file);
    return /\.(jpe?g|png|webp)$/i.test(entry.name) ? [file] : [];
  });
}

let previous = {};
try {
  previous = JSON.parse(readFileSync(MANIFEST, "utf8"));
} catch {
  // First run: everything gets made.
}

const manifest = {};
let made = 0;
let originalBytes = 0;
let variantBytes = 0;

async function process(file) {
  const { mtimeMs, size } = statSync(file);
  const key = publicPath(file);
  const known = previous[key];
  if (known?.mtimeMs === mtimeMs && known?.size === size && known.widths.every((w) => existsSync(variantFile(file, w)))) {
    manifest[key] = known;
    return;
  }

  // Orientation lives in EXIF: width and height have to be the ones the eye sees.
  const meta = await sharp(file).metadata();
  const sideways = (meta.orientation ?? 1) >= 5;
  const width = sideways ? meta.height : meta.width;
  const height = sideways ? meta.width : meta.height;
  const widths = WIDTHS.filter((w) => w < width);
  if (width < WIDTHS.at(-1)) widths.push(width);
  else if (!widths.includes(WIDTHS.at(-1))) widths.push(WIDTHS.at(-1));

  for (const w of widths) {
    const out = variantFile(file, w);
    mkdirSync(path.dirname(out), { recursive: true });
    await sharp(file).rotate().resize({ width: w, withoutEnlargement: true }).webp({ quality: QUALITY, effort: 4 }).toFile(out);
    variantBytes += statSync(out).size;
  }
  originalBytes += size;
  made++;
  manifest[key] = { mtimeMs, size, width, height, widths };
}

const files = photos(SOURCE);
const queue = [...files];
await Promise.all(
  Array.from({ length: WORKERS }, async () => {
    while (queue.length) await process(queue.shift());
  }),
);

// Copies of photos that no longer exist would otherwise be deployed forever.
for (const [key, entry] of Object.entries(previous)) {
  if (manifest[key]) continue;
  const file = path.join("public", key);
  for (const w of entry.widths) rmSync(variantFile(file, w), { force: true });
}

mkdirSync(path.dirname(MANIFEST), { recursive: true });
writeFileSync(MANIFEST, JSON.stringify(manifest));
const mb = (bytes) => (bytes / 1e6).toFixed(1);
console.log(`images: ${files.length} photos, ${made} remade${made ? ` (${mb(originalBytes)} MB of originals → ${mb(variantBytes)} MB across all widths)` : ""}`);
