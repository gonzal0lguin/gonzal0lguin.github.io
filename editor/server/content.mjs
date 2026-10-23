// Reading and writing the Markdown files of src/content. Everything the editor saves goes
// through here, so every write is backed up first and a stale save is refused.
import { mkdir, rename, stat } from "node:fs/promises";
import path from "node:path";
import { HttpError, bad, exists, hashOf, peekFrontmatter, readTextFile, relative, safePath, trash, walk, writeFileWithBackup } from "./util.mjs";

export const CONTENT_DIR = "src/content";
const EN = "en";
const MARKDOWN = [".md", ".mdx"];

/** The collections of src/content/config.ts, in the order the editor lists them. */
export const COLLECTIONS = [
  { id: "ascents", label: "Ascensos", urlBase: "/ascents", mediaFolder: true },
  { id: "blog", label: "Noticias", urlBase: "/blog", mediaFolder: true },
  { id: "projects", label: "Proyectos", urlBase: "/projects", mediaFolder: true },
  { id: "pages", label: "Páginas", urlBase: null, mediaFolder: false },
];

const isMarkdown = (file) => MARKDOWN.includes(path.extname(file).toLowerCase());

/** Every entry of every collection, with its Spanish and English files paired up. */
export async function listEntries(root) {
  const collections = [];
  for (const collection of COLLECTIONS) {
    const files = await walk(root, path.join(root, CONTENT_DIR, collection.id), isMarkdown);
    const sources = new Map();
    const translations = new Map();
    for (const file of files) {
      const name = file.slice(`${CONTENT_DIR}/${collection.id}/`.length);
      const key = name.replace(/^en\//, "").replace(/\.mdx?$/, "");
      (name.startsWith(`${EN}/`) ? translations : sources).set(key, file);
    }

    const entries = [];
    for (const [key, file] of [...sources].sort()) {
      const text = await readTextFile(path.join(root, file));
      const data = peekFrontmatter(text);
      const info = await stat(path.join(root, file));
      const enFile = translations.get(key) ?? null;
      let translation = null;
      if (enFile) {
        const enText = await readTextFile(path.join(root, enFile));
        const enHash = peekFrontmatter(enText).translationHash ?? null;
        translation = {
          path: enFile,
          hash: enHash,
          // No hash means the translation was written by hand; `npm run translate` leaves those alone.
          state: !enHash ? "manual" : enHash === hashOf(text) ? "fresh" : "stale",
        };
      }
      entries.push({
        collection: collection.id,
        key,
        path: file,
        mtime: Math.round(info.mtimeMs),
        title: data.title ?? key,
        description: data.description ?? "",
        pubDate: data.pubDate ?? null,
        badge: data.badge ?? null,
        category: data.category ?? null,
        tags: Array.isArray(data.tags) ? data.tags : [],
        heroImage: data.heroImage ?? null,
        route: data.route ? (Array.isArray(data.route) ? data.route : [data.route]) : [],
        translation,
      });
    }
    // Newest first, like the site's own listings.
    entries.sort((a, b) => String(b.pubDate ?? "").localeCompare(String(a.pubDate ?? "")) || a.key.localeCompare(b.key));
    collections.push({ ...collection, entries });
  }
  return collections;
}

export const entryPath = (collection, key, lang, ext = ".md") =>
  `${CONTENT_DIR}/${collection}/${lang === "en" ? `${EN}/` : ""}${key}${ext}`;

export async function readEntry(root, rel) {
  const full = safePath(root, rel, { extensions: MARKDOWN });
  const text = await readTextFile(full);
  const info = await stat(full);
  return { path: relative(root, full), text, mtime: Math.round(info.mtimeMs) };
}

/**
 * Overwrite a content file. `mtime` is the one the browser loaded: when the file changed
 * underneath (another editor, `npm run translate`, a git checkout) the save is refused
 * instead of throwing away those changes.
 */
export async function saveEntry(root, { path: rel, text, mtime, force }) {
  const full = safePath(root, rel, { extensions: MARKDOWN });
  if (typeof text !== "string") throw bad("Missing file content");
  if (!rel.startsWith(`${CONTENT_DIR}/`)) throw bad("Only files under src/content can be saved here");
  if (await exists(full)) {
    const info = await stat(full);
    const current = Math.round(info.mtimeMs);
    if (!force && mtime && current !== mtime) {
      throw new HttpError(409, `${rel} changed on disk since it was opened. Reload it, or save again to overwrite.`);
    }
  }
  const result = await writeFileWithBackup(root, full, text);
  const info = await stat(full);
  return { ...result, mtime: Math.round(info.mtimeMs) };
}

export async function createEntry(root, { collection, key, lang = "es", ext = ".md", text }) {
  if (!COLLECTIONS.some((c) => c.id === collection)) throw bad(`Unknown collection: ${collection}`);
  if (!/^[a-z0-9][a-z0-9-]*$/.test(key ?? "")) throw bad("The slug may only contain lowercase letters, numbers and dashes");
  const rel = entryPath(collection, key, lang, MARKDOWN.includes(ext) ? ext : ".md");
  const full = safePath(root, rel, { extensions: MARKDOWN });
  if (await exists(full)) throw bad(`${rel} already exists`);
  await mkdir(path.dirname(full), { recursive: true });
  const result = await writeFileWithBackup(root, full, text ?? "");
  const info = await stat(full);
  return { ...result, mtime: Math.round(info.mtimeMs) };
}

/** Move an entry (and its translation) to .editor-local/trash. */
export async function deleteEntry(root, { path: rel, withTranslation = true }) {
  const full = safePath(root, rel, { extensions: MARKDOWN });
  const moved = [await trash(root, full)];
  if (withTranslation) {
    const name = path.basename(rel);
    const dir = path.dirname(rel);
    const partner = dir.endsWith(`/${EN}`) ? `${dir.slice(0, -EN.length - 1)}/${name}` : `${dir}/${EN}/${name}`;
    const partnerFull = path.join(root, partner);
    if (await exists(partnerFull)) moved.push(await trash(root, partnerFull));
  }
  return { trashed: moved };
}

/** Rename an entry's slug — the URL changes, so both language files move together. */
export async function renameEntry(root, { path: rel, key }) {
  const full = safePath(root, rel, { extensions: MARKDOWN });
  if (!/^[a-z0-9][a-z0-9-]*$/.test(key ?? "")) throw bad("The slug may only contain lowercase letters, numbers and dashes");
  const ext = path.extname(full);
  const name = path.basename(rel);
  const dir = path.dirname(rel);
  const isEnglish = dir.endsWith(`/${EN}`);
  const baseDir = isEnglish ? dir.slice(0, -EN.length - 1) : dir;
  const pairs = [
    [`${baseDir}/${name}`, `${baseDir}/${key}${ext}`],
    [`${baseDir}/${EN}/${name}`, `${baseDir}/${EN}/${key}${ext}`],
  ];
  const moved = [];
  for (const [from, to] of pairs) {
    const fromFull = path.join(root, from);
    const toFull = safePath(root, to, { extensions: MARKDOWN });
    if (!(await exists(fromFull))) continue;
    if (await exists(toFull)) throw bad(`${to} already exists`);
    await rename(fromFull, toFull);
    moved.push({ from, to });
  }
  if (moved.length === 0) throw bad(`${rel} not found`);
  return { moved, path: `${baseDir}/${isEnglish ? `${EN}/` : ""}${key}${ext}` };
}

/** The hash `npm run translate` stores, so the editor can tell a stale translation. */
export const sourceHash = (text) => hashOf(text);
