// Turns the planting spreadsheet into the JSON the digital inventory is built from:
//
//   public/assets/INVENTARIO.xlsx  ->  src/data/inventory.json
//
// Run it after editing the spreadsheet:  npm run inventory
//
// The spreadsheet is the working document (it lives in a shared drive and gets edited in the
// field), so this script only reads it: every normalization happens here, and anything it
// can't make sense of is reported as a data-quality issue instead of being guessed. Those
// issues end up in the JSON too, so the page can say out loud what it doesn't know.
//
// An .xlsx file is a zip of XML parts. Reading it with node:zlib keeps the site free of a
// spreadsheet dependency, the same way the GPX and KML code is hand-written.
import { inflateRawSync } from "node:zlib";
import { readFileSync, writeFileSync } from "node:fs";
import path from "node:path";

const SOURCE = "public/assets/INVENTARIO.xlsx";
const OUTPUT = "src/data/inventory.json";

// Column headers of the spreadsheet, in order. Kept here so a renamed or reordered column
// fails loudly instead of silently shifting every value one place to the left.
const COLUMNS = [
  "id",
  "name",
  "scientific",
  "habit",
  "planted",
  "monthsPlanted",
  "height",
  "coordinates",
  "origin",
  "status",
  "notes",
];

const HABITS = { árbol: "tree", arbol: "tree", arbusto: "shrub" };

// The spreadsheet's `Estado` column. An empty cell means nobody has checked that plant since
// it went in, which is not the same as it being alive.
const STATUSES = {
  vivo: "alive",
  viva: "alive",
  muerto: "dead",
  muerta: "dead",
  "robado/vandalizado": "lost",
  robado: "lost",
  vandalizado: "lost",
};

/* ------------------------------------------------------------------ zip + xlsx reading */

/** The contents of one file inside a zip, by name. */
function readZip(buffer) {
  const files = new Map();
  // Walk the central directory backwards from the end-of-central-directory record.
  let eocd = buffer.length - 22;
  while (eocd >= 0 && buffer.readUInt32LE(eocd) !== 0x06054b50) eocd--;
  if (eocd < 0) throw new Error("Not a zip file: no end-of-central-directory record");
  let entry = buffer.readUInt32LE(eocd + 16);
  const count = buffer.readUInt16LE(eocd + 10);

  for (let i = 0; i < count; i++) {
    if (buffer.readUInt32LE(entry) !== 0x02014b50) throw new Error("Corrupt zip central directory");
    const method = buffer.readUInt16LE(entry + 10);
    const compressedSize = buffer.readUInt32LE(entry + 20);
    const nameLength = buffer.readUInt16LE(entry + 28);
    const extraLength = buffer.readUInt16LE(entry + 30);
    const commentLength = buffer.readUInt16LE(entry + 32);
    const offset = buffer.readUInt32LE(entry + 42);
    const name = buffer.toString("utf8", entry + 46, entry + 46 + nameLength);

    // The local header repeats the name and extra field, whose lengths can differ from the
    // central directory's, so the data offset has to be read from the local header itself.
    const localExtra = buffer.readUInt16LE(offset + 28);
    const localName = buffer.readUInt16LE(offset + 26);
    const start = offset + 30 + localName + localExtra;
    const raw = buffer.subarray(start, start + compressedSize);
    files.set(name, method === 0 ? raw : inflateRawSync(raw));
    entry += 46 + nameLength + extraLength + commentLength;
  }
  return files;
}

const unescapeXml = (text) =>
  text
    .replace(/&lt;/g, "<")
    .replace(/&gt;/g, ">")
    .replace(/&quot;/g, '"')
    .replace(/&apos;/g, "'")
    .replace(/&#(\d+);/g, (_, code) => String.fromCodePoint(Number(code)))
    .replace(/&#x([0-9a-f]+);/gi, (_, code) => String.fromCodePoint(parseInt(code, 16)))
    .replace(/&amp;/g, "&");

/** Text of every <t> element of an XML fragment, concatenated (a string can be split in runs). */
const textOf = (xml) => [...xml.matchAll(/<t(?:\s[^>]*)?>([\s\S]*?)<\/t>/g)].map((m) => unescapeXml(m[1])).join("");

/** The <si> entries of xl/sharedStrings.xml: the string table cells point into. */
function sharedStrings(xml) {
  if (!xml) return [];
  return [...xml.matchAll(/<si(?:\s[^>]*)?>([\s\S]*?)<\/si>/g)].map((m) => textOf(m[1]));
}

/** Rows of the first worksheet as maps of column letter -> text. */
function sheetRows(xml, strings) {
  return [...xml.matchAll(/<row[^>]*\br="(\d+)"[^>]*>([\s\S]*?)<\/row>/g)].map(([, number, body]) => {
    const cells = new Map();
    for (const [, ref, attrs, content] of body.matchAll(/<c\s+r="([A-Z]+)\d+"([^>]*?)(?:\/>|>([\s\S]*?)<\/c>)/g)) {
      const type = /\bt="([^"]+)"/.exec(attrs)?.[1];
      const value = /<v>([\s\S]*?)<\/v>/.exec(content ?? "")?.[1];
      let text = "";
      if (type === "s") text = strings[Number(value)] ?? "";
      else if (type === "inlineStr") text = textOf(content ?? "");
      else if (value != null) text = unescapeXml(value);
      cells.set(ref, text.trim());
    }
    return { number: Number(number), cells };
  });
}

/* ---------------------------------------------------------------------- normalization */

/** An Excel serial date (days since 1899-12-30) as an ISO calendar day. */
function excelDate(serial) {
  const days = Number(serial);
  if (!Number.isFinite(days) || days < 20000) return null;
  return new Date(Date.UTC(1899, 11, 30) + Math.round(days) * 86_400_000).toISOString().slice(0, 10);
}

/**
 * The `Coordenadas` column, which is filled by hand in the field and so comes in several
 * shapes: "-32.85790, -70.65715", the same without the comma, or a note such as "(mapa G)"
 * for a plant whose position is only on a phone map and hasn't been transcribed yet.
 */
function coordinates(text) {
  const numbers = [...text.matchAll(/-?\d+\.\d+/g)].map((m) => Number(m[0]));
  if (numbers.length < 2) return { lat: null, lon: null, note: text || null };
  const [lat, lon] = numbers;
  // Everything here is in the Aconcagua valley; anything else is a typo, not a plant.
  if (lat < -34 || lat > -32 || lon < -71.5 || lon > -70) return { lat: null, lon: null, note: text };
  return { lat, lon, note: null };
}

/** Sentence case for a scientific name typed in a hurry: "baccharis linearis" -> "Baccharis linearis". */
const capitalize = (text) => (text ? text[0].toUpperCase() + text.slice(1) : text);

/* ----------------------------------------------------------------------------- build */

const root = process.cwd();
const buffer = readFileSync(path.join(root, SOURCE));
const zip = readZip(buffer);
const strings = sharedStrings(zip.get("xl/sharedStrings.xml")?.toString("utf8"));
const sheetName = [...zip.keys()].find((name) => /^xl\/worksheets\/sheet1\.xml$/.test(name));
if (!sheetName) throw new Error(`${SOURCE} has no first worksheet`);
const rows = sheetRows(zip.get(sheetName).toString("utf8"), strings);

// The header row is the first one that has the ID column filled; everything above it is
// spacing the spreadsheet uses for its title.
const headerIndex = rows.findIndex((row) => /^id$/i.test(row.cells.get("A") ?? ""));
if (headerIndex < 0) throw new Error(`${SOURCE}: no header row with an "ID" column`);
const header = rows[headerIndex];
const letters = [...header.cells.keys()].sort((a, b) => a.localeCompare(b, "en"));
if (letters.length !== COLUMNS.length) {
  throw new Error(`${SOURCE}: expected ${COLUMNS.length} columns, found ${letters.length} (${letters.join(", ")})`);
}
const columnOf = Object.fromEntries(COLUMNS.map((name, i) => [name, letters[i]]));

const issues = [];
const report = (kind, id, detail) => issues.push({ kind, id, detail });

const plants = [];
for (const row of rows.slice(headerIndex + 1)) {
  const get = (name) => row.cells.get(columnOf[name]) ?? "";
  const id = get("id");
  const name = get("name");
  if (!id && !name) continue; // trailing empty row

  const planted = excelDate(get("planted"));
  if (!planted) report("date", id, `row ${row.number} has no planting date`);

  const habit = HABITS[get("habit").toLowerCase()] ?? null;
  if (!habit) report("habit", id, `unknown growth habit "${get("habit")}"`);

  const rawStatus = get("status");
  const status = rawStatus ? (STATUSES[rawStatus.toLowerCase()] ?? null) : "unchecked";
  if (status === null) report("status", id, `unknown status "${rawStatus}"`);

  const { lat, lon, note } = coordinates(get("coordinates"));
  if (lat == null) report("coordinates", id, note ? `position recorded as "${note}"` : "no position recorded");

  const height = Number(get("height"));

  plants.push({
    id,
    name,
    scientific: capitalize(get("scientific")),
    habit,
    planted,
    origin: get("origin") || null,
    status: status ?? "unknown",
    lat,
    lon,
    locationNote: note,
    height: Number.isFinite(height) && height > 0 ? height : null,
    notes: get("notes") || null,
  });
}

// IDs are written on the aluminium tag tied to each plant, so a repeat means two plants in
// the field carry the same label — worth saying rather than silently de-duplicating.
const seen = new Map();
for (const plant of plants) {
  const count = (seen.get(plant.id) ?? 0) + 1;
  seen.set(plant.id, count);
  if (count > 1) report("id", plant.id, `label "${plant.id}" is used by ${count} plants`);
}

// Tags are numbered per species from 000 upwards, so a gap means a plant that was labelled
// and then never written down — the kind of thing only a machine notices.
const sequences = new Map();
for (const plant of plants) {
  const match = /^([A-Z]+)(\d+)$/.exec(plant.id);
  if (match) sequences.set(match[1], (sequences.get(match[1]) ?? new Set()).add(Number(match[2])));
}
for (const [prefix, numbers] of sequences) {
  for (let n = 0; n < Math.max(...numbers); n++) {
    if (!numbers.has(n)) {
      report("sequence", `${prefix}${String(n).padStart(3, "0")}`, `no row for tag ${prefix}${String(n).padStart(3, "0")}`);
    }
  }
}

const inventory = {
  // Regenerate with `npm run inventory` — edits made here are lost on the next run.
  source: SOURCE,
  generated: new Date().toISOString().slice(0, 10),
  plants,
  issues,
};

writeFileSync(path.join(root, OUTPUT), `${JSON.stringify(inventory, null, 2)}\n`);

const counted = (kind) => issues.filter((issue) => issue.kind === kind).length;
console.log(`${OUTPUT}: ${plants.length} plants, ${new Set(plants.map((p) => p.scientific)).size} species`);
console.log(
  `  ${plants.filter((p) => p.lat != null).length} with coordinates, ` +
    `${issues.length} data issues (${["coordinates", "status", "id", "date", "habit"]
      .map((kind) => `${counted(kind)} ${kind}`)
      .join(", ")})`,
);
