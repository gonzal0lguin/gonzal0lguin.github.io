// The photo library and the GPX files in public/, as the editor sees them: what is there,
// what each post uses, and where an upload lands.
import path from "node:path";
import { mkdir } from "node:fs/promises";
import { CONTENT_DIR } from "./content.mjs";
import { bad, exists, fileInfo, readTextFile, relative, safePath, trash, walk, writeFileWithBackup } from "./util.mjs";

export const PUBLIC_DIR = "public";
/** Mirrors DEFAULT_MEDIA_BASE_URL in src/lib/media.mjs: where a relative photo path points. */
export const POSTS_DIR = "public/assets/img/posts";
export const HEADERS_DIR = "public/assets/img/headers";
export const GPX_DIR = "public/assets/lib";

const IMAGE = /\.(jpe?g|png|webp|avif|gif|heic|heif|tiff?)$/i;
const IMAGE_EXTENSIONS = [".jpg", ".jpeg", ".png", ".webp", ".avif", ".gif", ".heic", ".heif", ".tif", ".tiff"];

/**
 * Everything in the photo library plus which entries reference each file. `relPath` is how a
 * post writes the photo ("la-cruz/ridge.jpg"); `path` is its place in the repository.
 */
export async function listMedia(root, mediaBaseUrl) {
  const usage = await collectUsage(root);
  const describe = async (rel, base) => ({
    ...(await fileInfo(root, rel)),
    name: path.basename(rel),
    relPath: base ? rel.slice(base.length + 1) : null,
    folder: base ? path.dirname(rel.slice(base.length + 1)).replace(/^\.$/, "") : null,
    usedBy: usage.get(path.basename(rel)) ?? [],
  });

  const posts = await Promise.all((await walk(root, path.join(root, POSTS_DIR), (f) => IMAGE.test(f))).map((rel) => describe(rel, POSTS_DIR)));
  const headers = await Promise.all((await walk(root, path.join(root, HEADERS_DIR), (f) => IMAGE.test(f))).map((rel) => describe(rel, HEADERS_DIR)));
  const folders = [...new Set(posts.map((photo) => photo.folder).filter(Boolean))].sort();
  return { mediaBaseUrl, postsDir: POSTS_DIR, headersDir: HEADERS_DIR, folders, posts, headers };
}

/** File name → the content files that mention it, to warn before deleting a photo in use. */
export async function collectUsage(root) {
  const usage = new Map();
  const files = await walk(root, path.join(root, CONTENT_DIR), (f) => /\.mdx?$/i.test(f));
  for (const file of files) {
    const text = await readTextFile(path.join(root, file));
    for (const [, name] of text.matchAll(/([\w.()\-% ]+\.(?:jpe?g|png|webp|avif|gif|heic|heif|tiff?|gpx))/gi)) {
      const list = usage.get(name) ?? [];
      if (!list.includes(file)) list.push(file);
      usage.set(name, list);
    }
  }
  return usage;
}

/**
 * Save an uploaded file. `kind` picks the destination: a post photo (optionally in a
 * subfolder), a hero image or a GPX track. An existing name is never overwritten — the new
 * file gets a "-2" suffix — so a drag & drop can't clobber a photo already in a post.
 */
export async function upload(root, { kind = "photo", folder = "", name }, body) {
  if (!body?.length) throw bad("Empty upload");
  const base = kind === "header" ? HEADERS_DIR : kind === "gpx" ? GPX_DIR : POSTS_DIR;
  const extensions = kind === "gpx" ? [".gpx"] : IMAGE_EXTENSIONS;
  const clean = safeName(name);
  if (!extensions.includes(path.extname(clean).toLowerCase())) {
    throw bad(`${clean}: expected ${extensions.join(", ")}`);
  }
  const dir = kind === "photo" && folder ? path.join(base, safeFolder(folder)) : base;
  await mkdir(safePath(root, dir), { recursive: true });

  let rel = `${dir}/${clean}`;
  const ext = path.extname(clean);
  const stem = clean.slice(0, -ext.length);
  for (let n = 2; await exists(safePath(root, rel, { extensions })); n++) rel = `${dir}/${stem}-${n}${ext}`;

  const full = safePath(root, rel, { extensions });
  await writeFileWithBackup(root, full, body);
  return {
    path: rel,
    relPath: rel.startsWith(`${POSTS_DIR}/`) ? rel.slice(POSTS_DIR.length + 1) : null,
    publicPath: `/${rel.slice(PUBLIC_DIR.length + 1)}`,
    ...(await fileInfo(root, rel)),
  };
}

/**
 * Overwrite one existing file in public/ with an upload, keeping its path. This is how the
 * site's own files are updated — the CV PDF, the profile photo, the favicon — because the
 * templates point at those paths, so the name has to stay the same.
 */
export async function replaceFile(root, { path: rel }, body) {
  if (!body?.length) throw bad("Empty upload");
  const full = safePath(root, rel, { extensions: REPLACEABLE_EXTENSIONS });
  if (!rel.startsWith(`${PUBLIC_DIR}/`)) throw bad("Only files under public/ can be replaced");
  if (!(await exists(full))) throw bad(`${rel} does not exist — use an upload instead of a replacement`);
  const result = await writeFileWithBackup(root, full, body);
  return { ...result, publicPath: `/${rel.slice(PUBLIC_DIR.length + 1)}`, ...(await fileInfo(root, rel)) };
}

const REPLACEABLE_EXTENSIONS = [...IMAGE_EXTENSIONS, ".pdf", ".svg", ".ico", ".gpx"];

/**
 * The files the site's own templates point at: the profile photo, the CV PDF, the favicon,
 * the flags. They live outside the content collections, so this is the only place the editor
 * can offer to change them — by replacing the file, never by touching the code.
 */
export async function siteAssets(root) {
  // The editor's own sources are skipped: a path named in a hint is not a use by the site.
  const code = (await walk(root, path.join(root, "src"), (file) => /\.(astro|ts|tsx|mjs|js|css)$/i.test(file))).filter(
    (file) => !file.startsWith("src/editor/") && file !== "src/styles/editor.css",
  );
  const references = new Map();
  for (const file of code) {
    const text = await readTextFile(path.join(root, file));
    for (const [, ref] of text.matchAll(/(\/(?:assets|flags)\/[A-Za-z0-9._\/-]+|\/favicon[A-Za-z0-9._-]*)/g)) {
      if (!REPLACEABLE_EXTENSIONS.includes(path.extname(ref).toLowerCase())) continue;
      // Photos of posts have their own view; this is about the site's furniture.
      if (ref.startsWith("/assets/img/posts/") || ref.endsWith(".gpx")) continue;
      references.set(ref, [...new Set([...(references.get(ref) ?? []), file])]);
    }
  }
  // Every PDF in public/, so a new CV can be put next to the old one.
  for (const pdf of await walk(root, path.join(root, PUBLIC_DIR, "assets", "pdf"), (file) => /\.pdf$/i.test(file))) {
    const ref = `/${pdf.slice(PUBLIC_DIR.length + 1)}`;
    if (!references.has(ref)) references.set(ref, []);
  }

  return Promise.all(
    [...references.entries()].sort().map(async ([publicPath, usedIn]) => {
      const rel = `${PUBLIC_DIR}${publicPath}`;
      const info = await fileInfo(root, rel);
      return {
        publicPath,
        path: rel,
        name: path.basename(publicPath),
        kind: IMAGE.test(publicPath) || /\.svg$/i.test(publicPath) ? "image" : path.extname(publicPath).slice(1).toLowerCase(),
        usedIn,
        ...info,
      };
    }),
  );
}

export async function deleteMedia(root, { paths }) {
  const list = Array.isArray(paths) ? paths : [paths];
  const trashed = [];
  for (const rel of list) {
    const full = safePath(root, rel, { extensions: [...IMAGE_EXTENSIONS, ".gpx"] });
    if (!rel.startsWith(`${PUBLIC_DIR}/`)) throw bad("Only files under public/ can be removed here");
    if (await exists(full)) trashed.push(await trash(root, full));
  }
  return { trashed };
}

// --- GPX files

export async function listGpx(root) {
  const files = await walk(root, path.join(root, PUBLIC_DIR), (f) => /\.gpx$/i.test(f));
  const usage = await collectUsage(root);
  return Promise.all(
    files.map(async (rel) => ({
      ...(await fileInfo(root, rel)),
      name: path.basename(rel),
      publicPath: `/${rel.slice(PUBLIC_DIR.length + 1)}`,
      usedBy: usage.get(path.basename(rel)) ?? [],
    })),
  );
}

/** A GPX file's XML. `publicPath` is the form posts use: "/assets/lib/union.gpx". */
export async function readGpx(root, publicPath) {
  const rel = `${PUBLIC_DIR}/${String(publicPath).replace(/^\/+/, "")}`;
  const full = safePath(root, rel, { extensions: [".gpx"] });
  return { path: rel, publicPath: `/${rel.slice(PUBLIC_DIR.length + 1)}`, xml: await readTextFile(full) };
}

export async function saveGpx(root, { publicPath, xml }) {
  if (typeof xml !== "string" || !xml.includes("<gpx")) throw bad("That does not look like a GPX file");
  const rel = `${PUBLIC_DIR}/${String(publicPath).replace(/^\/+/, "")}`;
  const full = safePath(root, rel, { extensions: [".gpx"] });
  const result = await writeFileWithBackup(root, full, xml);
  return { ...result, publicPath: `/${rel.slice(PUBLIC_DIR.length + 1)}`, ...(await fileInfo(root, rel)) };
}

const safeName = (name) =>
  path
    .basename(String(name ?? "file"))
    .replace(/[^\w.\- ]+/g, "-")
    .replace(/\s+/g, "-")
    .replace(/^-+/, "") || "file";

const safeFolder = (folder) =>
  String(folder)
    .split("/")
    .map((part) => part.replace(/[^\w.-]+/g, "-"))
    .filter((part) => part && part !== "." && part !== "..")
    .slice(0, 2)
    .join("/");

