// The resized copies of a photo, as made by scripts/images.mjs. Plain .mjs on node builtins
// because both the pages and the Markdown plugin (src/plugins/remark-media.mjs, which runs
// outside Vite) need it. Server side only: it reads a file.

import { readFileSync } from "node:fs";

const MANIFEST = ".cache/image-variants.json";
let manifest = null;

function entries() {
  if (manifest) return manifest;
  try {
    manifest = JSON.parse(readFileSync(MANIFEST, "utf8"));
  } catch {
    // Not generated (a bare `astro build`): every photo falls back to its original.
    manifest = {};
  }
  return manifest;
}

const variantUrl = (src, width) => `${src.replace("/assets/img/", "/assets/opt/")}.${width}.webp`;

/**
 * What an <img> needs to show `src` at the right size, or null when there are no copies of it
 * (a CDN photo, an SVG). `src` becomes the largest copy, for browsers that ignore srcset.
 */
export function variantsOf(src) {
  const entry = typeof src === "string" ? entries()[src] : null;
  if (!entry) return null;
  return {
    src: variantUrl(src, entry.widths.at(-1)),
    srcset: entry.widths.map((width) => `${variantUrl(src, width)} ${width}w`).join(", "),
    width: entry.width,
    height: entry.height,
  };
}

/** The smallest copy at least `width` wide, for places with a single known size (map popups). */
export function thumbOf(src, width = 640) {
  const entry = typeof src === "string" ? entries()[src] : null;
  if (!entry) return src;
  return variantUrl(src, entry.widths.find((w) => w >= width) ?? entry.widths.at(-1));
}
