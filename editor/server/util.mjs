// Shared helpers for the editor API (development only — see editor/vite-plugin.mjs).
import { createHash } from "node:crypto";
import { constants } from "node:fs";
import { access, copyFile, mkdir, readdir, readFile, rename, stat, writeFile } from "node:fs/promises";
import path from "node:path";

/** Where backups and deleted files go. Gitignored; nothing is ever removed for good. */
export const LOCAL_DIR = ".editor-local";

export class HttpError extends Error {
  constructor(status, message) {
    super(message);
    this.status = status;
  }
}

export const bad = (message) => new HttpError(400, message);

export function sendJson(res, status, data) {
  const body = JSON.stringify(data);
  res.statusCode = status;
  res.setHeader("content-type", "application/json; charset=utf-8");
  res.setHeader("cache-control", "no-store");
  res.end(body);
}

/** Request body as a Buffer, with a size limit (uploads are posted raw). */
export function readBody(req, limit = 64 * 1024 * 1024) {
  return new Promise((resolve, reject) => {
    const chunks = [];
    let size = 0;
    req.on("data", (chunk) => {
      size += chunk.length;
      if (size > limit) {
        reject(new HttpError(413, "Request body too large"));
        req.destroy();
        return;
      }
      chunks.push(chunk);
    });
    req.on("end", () => resolve(Buffer.concat(chunks)));
    req.on("error", reject);
  });
}

export async function readJson(req) {
  const body = await readBody(req, 4 * 1024 * 1024);
  if (body.length === 0) return {};
  try {
    return JSON.parse(body.toString("utf8"));
  } catch {
    throw bad("Request body is not valid JSON");
  }
}

/**
 * Resolve `relative` inside `root` and refuse anything that escapes it, so a bad request
 * can never touch a file outside the repository.
 */
export function safePath(root, relative, { extensions } = {}) {
  if (typeof relative !== "string" || relative.trim() === "") throw bad("Missing path");
  if (relative.includes("\0")) throw bad("Invalid path");
  const full = path.resolve(root, relative.replace(/^[/\\]+/, ""));
  if (full !== root && !full.startsWith(root + path.sep)) throw bad(`Path outside the project: ${relative}`);
  if (extensions && !extensions.includes(path.extname(full).toLowerCase())) {
    throw bad(`Unsupported file type: ${path.basename(full)} (expected ${extensions.join(", ")})`);
  }
  return full;
}

/** Repository-relative path with forward slashes, the form the API speaks in. */
export const relative = (root, full) => path.relative(root, full).split(path.sep).join("/");

export const exists = (file) => access(file, constants.F_OK).then(() => true, () => false);

export const hashOf = (text) => createHash("sha256").update(text).digest("hex").slice(0, 16);

export const stamp = () => new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19);

/** Copy a file's current content into .editor-local/backups before overwriting it. */
export async function backup(root, full) {
  if (!(await exists(full))) return null;
  const target = path.join(root, LOCAL_DIR, "backups", `${relative(root, full)}.${stamp()}.bak`);
  await mkdir(path.dirname(target), { recursive: true });
  await copyFile(full, target);
  return relative(root, target);
}

/** Move a file out of the project into .editor-local/trash instead of deleting it. */
export async function trash(root, full) {
  const target = path.join(root, LOCAL_DIR, "trash", stamp(), relative(root, full));
  await mkdir(path.dirname(target), { recursive: true });
  await rename(full, target);
  return relative(root, target);
}

/** Write a file, keeping a copy of what was there. `data` is text, or bytes for an upload. */
export async function writeFileWithBackup(root, full, data) {
  const backupPath = await backup(root, full);
  await mkdir(path.dirname(full), { recursive: true });
  await writeFile(full, data, typeof data === "string" ? "utf8" : undefined);
  return { path: relative(root, full), backup: backupPath };
}

export async function readTextFile(full) {
  try {
    return await readFile(full, "utf8");
  } catch {
    throw new HttpError(404, `File not found: ${path.basename(full)}`);
  }
}

/** Every file under `dir`, repository-relative, sorted. Missing directories yield nothing. */
export async function walk(root, dir, test = () => true) {
  const out = [];
  async function visit(current) {
    let entries;
    try {
      entries = await readdir(current, { withFileTypes: true });
    } catch {
      return;
    }
    for (const entry of entries) {
      const full = path.join(current, entry.name);
      if (entry.isDirectory()) await visit(full);
      else if (entry.isFile() && test(full)) out.push(full);
    }
  }
  await visit(dir);
  out.sort();
  return out.map((full) => relative(root, full));
}

export async function fileInfo(root, rel) {
  try {
    const info = await stat(path.join(root, rel));
    return { path: rel, size: info.size, mtime: Math.round(info.mtimeMs) };
  } catch {
    return { path: rel, size: 0, mtime: 0, missing: true };
  }
}

/** Split a Markdown file into its frontmatter lines and its body, the way scripts/translate.mjs does. */
export function splitFile(text) {
  const match = text.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n?/);
  if (!match) return { frontmatter: "", body: text };
  return { frontmatter: match[1], body: text.slice(match[0].length) };
}

/**
 * Values of the frontmatter keys the dashboard shows. Deliberately shallow: the editor itself
 * parses the frontmatter in the browser (src/editor/frontmatter.ts) and keeps every key it
 * does not understand untouched.
 */
export function peekFrontmatter(text) {
  const { frontmatter } = splitFile(text);
  const data = {};
  const lines = frontmatter.split(/\r?\n/);
  lines.forEach((line, i) => {
    const match = line.match(/^([A-Za-z_][\w-]*):\s*(.*)$/);
    if (!match) return;
    const [, key, raw] = match;
    const value = raw.trim();
    if (value === "" || value === "|" || value === ">") {
      // A block value: collect the indented lines that follow (a list of files, mostly).
      const items = [];
      for (let j = i + 1; j < lines.length && /^\s+/.test(lines[j]); j++) {
        const item = lines[j].trim().replace(/^-\s*/, "");
        if (item) items.push(unquote(item));
      }
      data[key] = items;
      return;
    }
    if (value.startsWith("[")) {
      data[key] = value
        .slice(1, value.lastIndexOf("]") === -1 ? undefined : value.lastIndexOf("]"))
        .split(",")
        .map((item) => unquote(item.trim()))
        .filter(Boolean);
      return;
    }
    data[key] = unquote(value);
  });
  return data;
}

export function unquote(value) {
  const text = String(value).trim();
  if ((text.startsWith('"') && text.endsWith('"')) || (text.startsWith("'") && text.endsWith("'"))) {
    return text.slice(1, -1).replace(/\\"/g, '"');
  }
  return text;
}
