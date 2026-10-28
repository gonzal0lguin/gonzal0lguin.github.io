// Reading and writing a content file's YAML frontmatter without disturbing it.
//
// The file is kept as the list of top-level key blocks it already is (the same split
// scripts/translate.mjs does). A key whose value the editor did not change is written back
// byte for byte; only edited keys are re-serialized, in the style the existing files use
// (`title: "..."`, `tags: ['a', 'b']`, block lists for `route` and `waypoints`). Opening a
// file and saving it without touching anything therefore leaves it unchanged.

export type Scalar = string | number | boolean;
export type FmValue = Scalar | Scalar[] | Record<string, Scalar>[] | null;

interface KeyBlock {
  key: string;
  lines: string[];
}

/** Where a key the file does not have yet is inserted, so new files read like the old ones. */
const KEY_ORDER = [
  "translationHash",
  "title",
  "description",
  "pubDate",
  "updatedDate",
  "mountain",
  "elevation",
  "summits",
  "attempt",
  "latitude",
  "longitude",
  "greeting",
  "subtitle",
  "heroImage",
  "badge",
  "category",
  "route",
  "waypoints",
  "timezone",
  "tags",
];

export class Frontmatter {
  private blocks: KeyBlock[];
  private edited = new Map<string, FmValue>();
  /** Lines before the first key (a leading comment), kept as they are. */
  private preamble: string[];

  constructor(yaml: string) {
    const { blocks, preamble } = split(yaml);
    this.blocks = blocks;
    this.preamble = preamble;
  }

  keys() {
    return [...new Set([...this.blocks.map((b) => b.key), ...this.edited.keys()])];
  }

  has(key: string) {
    return this.edited.has(key) ? this.edited.get(key) != null : this.blocks.some((b) => b.key === key);
  }

  /** The key's value: what the editor set, or what the file says. */
  get(key: string): FmValue {
    if (this.edited.has(key)) return this.edited.get(key) ?? null;
    const block = this.blocks.find((b) => b.key === key);
    return block ? parseBlock(block.lines) : null;
  }

  string(key: string, fallback = "") {
    const value = this.get(key);
    return value == null || Array.isArray(value) ? fallback : String(value);
  }

  number(key: string): number | null {
    const value = this.get(key);
    if (value == null || value === "") return null;
    const n = Number(value);
    return Number.isFinite(n) ? n : null;
  }

  list(key: string): string[] {
    const value = this.get(key);
    if (value == null) return [];
    return (Array.isArray(value) ? value : [value]).map((item) => String(item));
  }

  maps(key: string): Record<string, Scalar>[] {
    const value = this.get(key);
    return Array.isArray(value) ? (value.filter((item) => item && typeof item === "object") as Record<string, Scalar>[]) : [];
  }

  /** Set a value, or remove the key when it is null, undefined, "" or an empty list. */
  set(key: string, value: FmValue) {
    const empty = value == null || value === "" || (Array.isArray(value) && value.length === 0);
    this.edited.set(key, empty ? null : value);
  }

  remove(key: string) {
    this.edited.set(key, null);
  }

  toYaml() {
    const out: KeyBlock[] = [];
    for (const block of this.blocks) {
      if (!this.edited.has(block.key)) {
        out.push(block);
        continue;
      }
      const value = this.edited.get(block.key) ?? null;
      if (value != null) out.push({ key: block.key, lines: emit(block.key, value) });
    }
    for (const [key, value] of this.edited) {
      if (value == null || out.some((block) => block.key === key)) continue;
      const rank = orderOf(key);
      const at = out.findIndex((block) => orderOf(block.key) > rank);
      out.splice(at === -1 ? out.length : at, 0, { key, lines: emit(key, value) });
    }
    return [...this.preamble, ...out.flatMap((block) => block.lines)].join("\n");
  }
}

const orderOf = (key: string) => {
  const index = KEY_ORDER.indexOf(key);
  return index === -1 ? KEY_ORDER.length : index;
};

/** Split the YAML into one block per top-level key: its own line plus the indented lines under it. */
function split(yaml: string) {
  const blocks: KeyBlock[] = [];
  const preamble: string[] = [];
  for (const line of yaml.split(/\r?\n/)) {
    if (/^[A-Za-z_][\w-]*\s*:/.test(line)) blocks.push({ key: line.slice(0, line.indexOf(":")).trim(), lines: [line] });
    else if (blocks.length) blocks[blocks.length - 1].lines.push(line);
    else preamble.push(line);
  }
  return { blocks, preamble };
}

function parseBlock(lines: string[]): FmValue {
  const first = lines[0] ?? "";
  const inline = first.slice(first.indexOf(":") + 1).trim();
  const rest = lines.slice(1).filter((line) => line.trim() !== "" && !line.trim().startsWith("#"));

  if (inline && !inline.startsWith("#")) return parseInline(inline);
  if (rest.length === 0) return null;

  if (rest.every((line) => line.trim().startsWith("-"))) {
    return rest.map((line) => parseInline(line.trim().slice(1).trim())) as Scalar[] | Record<string, Scalar>[];
  }
  // A nested map (nothing in this site uses one, but it round-trips as an object).
  const map: Record<string, Scalar> = {};
  for (const line of rest) {
    const at = line.indexOf(":");
    if (at > 0) map[line.slice(0, at).trim()] = parseInline(line.slice(at + 1).trim()) as Scalar;
  }
  return [map];
}

function parseInline(text: string): FmValue {
  const value = stripComment(text);
  if (value === "" || value === "~" || value === "null") return null;
  if (value.startsWith("[")) {
    return splitItems(value.slice(1, value.lastIndexOf("]") === -1 ? undefined : value.lastIndexOf("]"))).map((item) => parseInline(item) as Scalar);
  }
  if (value.startsWith("{")) {
    const map: Record<string, Scalar> = {};
    for (const pair of splitItems(value.slice(1, value.lastIndexOf("}") === -1 ? undefined : value.lastIndexOf("}")))) {
      const at = pair.indexOf(":");
      if (at > 0) map[unquote(pair.slice(0, at).trim())] = parseInline(pair.slice(at + 1).trim()) as Scalar;
    }
    return map as unknown as FmValue;
  }
  if (/^(['"]).*\1$/s.test(value)) return unquote(value);
  if (value === "true" || value === "false") return value === "true";
  if (/^-?\d+(\.\d+)?$/.test(value)) return Number(value);
  return value;
}

/** Comma-separated items of a flow collection, ignoring commas inside quotes or nested braces. */
function splitItems(text: string) {
  const items: string[] = [];
  let depth = 0;
  let quote = "";
  let current = "";
  for (const char of text) {
    if (quote) {
      current += char;
      if (char === quote) quote = "";
      continue;
    }
    if (char === '"' || char === "'") quote = char;
    if (char === "[" || char === "{") depth++;
    if (char === "]" || char === "}") depth--;
    if (char === "," && depth === 0) {
      items.push(current.trim());
      current = "";
      continue;
    }
    current += char;
  }
  if (current.trim()) items.push(current.trim());
  return items.filter(Boolean);
}

function stripComment(text: string) {
  let quote = "";
  for (let i = 0; i < text.length; i++) {
    const char = text[i];
    if (quote) {
      if (char === quote) quote = "";
    } else if (char === '"' || char === "'") quote = char;
    else if (char === "#" && (i === 0 || /\s/.test(text[i - 1]))) return text.slice(0, i).trim();
  }
  return text.trim();
}

export function unquote(value: string) {
  const text = value.trim();
  if (text.startsWith('"') && text.endsWith('"') && text.length > 1) return text.slice(1, -1).replace(/\\(["\\])/g, "$1");
  if (text.startsWith("'") && text.endsWith("'") && text.length > 1) return text.slice(1, -1).replace(/''/g, "'");
  return text;
}

const quote = (value: string) => `"${String(value).replace(/\\/g, "\\\\").replace(/"/g, '\\"')}"`;
const quoteSingle = (value: string) => `'${String(value).replace(/'/g, "''")}'`;

/** Keys written as a single-quoted flow list, the way every post writes its tags. */
const FLOW_LIST_KEYS = new Set(["tags", "summits"]);
/** Keys whose values are never quoted. */
const BARE_KEYS = new Set(["elevation", "latitude", "longitude"]);
/** Order of a waypoint's fields, matching the example in src/content/config.ts. */
const WAYPOINT_FIELDS = ["name", "kind", "lat", "lon", "ele", "note"];

function scalar(key: string, value: Scalar) {
  if (typeof value === "number" || typeof value === "boolean") return String(value);
  const text = String(value);
  if (BARE_KEYS.has(key) && /^-?\d+(\.\d+)?$/.test(text)) return text;
  return quote(text);
}

function emit(key: string, value: FmValue): string[] {
  if (Array.isArray(value)) {
    if (value.length && typeof value[0] === "object") {
      return [`${key}:`, ...(value as Record<string, Scalar>[]).map((item) => `  - ${flowMap(item)}`)];
    }
    const items = value as Scalar[];
    if (FLOW_LIST_KEYS.has(key)) return [`${key}: [${items.map((item) => (typeof item === "number" ? item : quoteSingle(String(item)))).join(", ")}]`];
    if (items.length === 1) return [`${key}: ${scalar(key, items[0])}`];
    return [`${key}:`, ...items.map((item) => `  - ${scalar(key, item)}`)];
  }
  return [`${key}: ${scalar(key, value as Scalar)}`];
}

function flowMap(item: Record<string, Scalar>) {
  const keys = [...WAYPOINT_FIELDS.filter((field) => field in item), ...Object.keys(item).filter((field) => !WAYPOINT_FIELDS.includes(field))];
  const pairs = keys
    .filter((field) => item[field] != null && item[field] !== "")
    .map((field) => {
      const value = item[field];
      // `kind` is an enum, written bare like in the schema's own example.
      if (field === "kind" || typeof value === "number" || typeof value === "boolean") return `${field}: ${value}`;
      return `${field}: ${quote(String(value))}`;
    });
  return `{ ${pairs.join(", ")} }`;
}

// --- Whole file

export interface ParsedFile {
  /** The frontmatter, editable without losing anything */
  fm: Frontmatter;
  /** Everything after the closing `---` */
  body: string;
  /** True when the file had no frontmatter block at all */
  bare: boolean;
}

export function parseFile(text: string): ParsedFile {
  const match = text.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n?/);
  if (!match) return { fm: new Frontmatter(""), body: text, bare: true };
  return { fm: new Frontmatter(match[1]), body: text.slice(match[0].length), bare: false };
}

export function serializeFile({ fm, body }: ParsedFile) {
  return `---\n${fm.toYaml()}\n---\n${body}`;
}
