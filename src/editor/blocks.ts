// The body of a post as a list of blocks, so photos can be edited as photos instead of as text.
//
// Two kinds of block: text (any Markdown) and photos — a `:::gallery` / `:::carousel`
// container, or a paragraph that holds nothing but images, which is exactly what
// src/plugins/remark-media.mjs turns into a figure or a grid.
//
// Every block keeps the source it came from, separating blank lines included, and gives it
// back untouched unless it was edited. Round-tripping a file is therefore a no-op.

export interface Photo {
  /** The path as written in the Markdown, without the location fragment */
  path: string;
  alt: string;
  /** The image title: the caption shown under the photo */
  caption: string;
  /** What follows "#" in the path: "off", "km=5.8", "time=12:41" or "-33.75,-70.10" */
  geo: string;
}

export type PhotoLayout = "auto" | "gallery" | "carousel";

interface Common {
  id: string;
  /** Exact source of the block, including the blank lines that followed it */
  raw: string;
  /** Blank lines that came before it and belong to nothing else (the gap after the frontmatter) */
  before?: string;
  /** Set once the block is edited: `raw` is no longer used */
  dirty?: boolean;
}

export interface TextBlock extends Common {
  type: "text";
  text: string;
}

export interface PhotosBlock extends Common {
  type: "photos";
  layout: PhotoLayout;
  cols: number | null;
  ratio: string | null;
  caption: string;
  photos: Photo[];
}

export type Block = TextBlock | PhotosBlock;

let counter = 0;
const nextId = () => `b${++counter}`;

const IMAGE_LINE = /^!\[([^\]]*)\]\(\s*([^)\s]+?)(?:\s+"([^"]*)")?\s*\)\s*$/;
const DIRECTIVE_OPEN = /^:::(gallery|carousel)\s*(\{[^}]*\})?\s*$/;
// A line that starts an HTML or MDX element: <TimeLineGroup>, <img ...>, <Columns />
const ELEMENT_OPEN = /^\s*<([A-Za-z][\w.-]*)[\s>/]/;
// An MDX expression or comment: {/* ... */}
const EXPRESSION_OPEN = /^\s*\{/;

/** Split a post body into blocks. Concatenating every `raw` gives the body back exactly. */
export function parseBlocks(body: string): Block[] {
  const lines = body.split("\n");
  const blocks: Block[] = [];
  let i = 0;
  let fence: string | null = null;

  // Lines of the paragraph being collected.
  let buffer: string[] = [];
  // Blank lines with no block to belong to yet (the gap between the frontmatter and the text).
  let prefix = "";
  /** `more` tells whether lines follow, i.e. whether the block ends with a newline. */
  const flushText = (more: boolean) => {
    if (buffer.length === 0) return;
    const raw = buffer.join("\n") + (more ? "\n" : "");
    buffer = [];
    // Blank lines alone are not a block: they are the separator of the one before them.
    if (raw.trim() === "") {
      if (blocks.length) blocks[blocks.length - 1].raw += raw;
      else prefix += raw;
      return;
    }
    blocks.push({ id: nextId(), type: "text", raw, text: raw.replace(/\n+$/, "") });
  };

  while (i < lines.length) {
    const line = lines[i];
    const fenceMatch = line.match(/^\s*(```+|~~~+)/);
    if (fence) {
      buffer.push(line);
      if (fenceMatch && line.trim().startsWith(fence)) fence = null;
      i++;
      continue;
    }
    if (fenceMatch) {
      fence = fenceMatch[1];
      buffer.push(line);
      i++;
      continue;
    }

    const directive = line.match(DIRECTIVE_OPEN);
    if (directive) {
      const end = findClose(lines, i + 1);
      if (end !== -1) {
        flushText(true);
        const inner = lines.slice(i + 1, end);
        const trailing = blankRun(lines, end + 1);
        const raw = [...lines.slice(i, end + 1), ...lines.slice(end + 1, end + 1 + trailing)].join("\n");
        blocks.push({
          id: nextId(),
          type: "photos",
          raw: raw + (end + 1 + trailing < lines.length ? "\n" : ""),
          layout: directive[1] as PhotoLayout,
          ...parseAttributes(directive[2] ?? ""),
          photos: inner.map(parseImageLine).filter((photo): photo is Photo => photo != null),
        });
        i = end + 1 + trailing;
        continue;
      }
    }

    // An element or an MDX expression that spans several lines (the CV is built out of
    // <TimeLineGroup> and <TimeLine>): keep the whole thing in one block, so it can be
    // edited as a unit instead of being cut at every blank line inside it.
    if (ELEMENT_OPEN.test(line) || EXPRESSION_OPEN.test(line)) {
      const span = ELEMENT_OPEN.test(line)
        ? elementLines(lines, i, line.match(ELEMENT_OPEN)![1])
        : expressionLines(lines, i);
      if (span > 1) {
        flushText(true);
        const trailing = blankRun(lines, i + span);
        const end = i + span + trailing;
        const raw = lines.slice(i, end).join("\n") + (end < lines.length ? "\n" : "");
        blocks.push({ id: nextId(), type: "text", raw, text: raw.replace(/\n+$/, "") });
        i = end;
        continue;
      }
    }

    // A paragraph of nothing but images: the loose layout remark-media picks up.
    if (IMAGE_LINE.test(line.trim())) {
      let end = i;
      while (end < lines.length && IMAGE_LINE.test(lines[end].trim())) end++;
      const photos = lines.slice(i, end).map(parseImageLine).filter((photo): photo is Photo => photo != null);
      if (photos.length) {
        flushText(true);
        const trailing = blankRun(lines, end);
        const raw = lines.slice(i, end + trailing).join("\n");
        blocks.push({
          id: nextId(),
          type: "photos",
          raw: raw + (end + trailing < lines.length ? "\n" : ""),
          layout: "auto",
          cols: null,
          ratio: null,
          caption: "",
          photos,
        });
        i = end + trailing;
        continue;
      }
    }

    // A blank line ends the paragraph: one block per paragraph, list, table or code block.
    if (line.trim() === "") {
      buffer.push(line);
      i++;
      while (i < lines.length && lines[i].trim() === "") buffer.push(lines[i++]);
      flushText(i < lines.length);
      continue;
    }

    buffer.push(line);
    i++;
  }
  flushText(false);
  if (prefix && blocks.length) blocks[0].before = prefix;
  return blocks;
}

/** How many blank lines start at `from`. */
function blankRun(lines: string[], from: number) {
  let n = 0;
  while (from + n < lines.length && lines[from + n].trim() === "") n++;
  return n;
}

/**
 * How many lines the element opened at `start` takes. Tags are matched on the joined text, so
 * attributes spread over several lines and `/>` on a line of its own both work. Returns 1 when
 * the element opens and closes on the same line (then it is ordinary text).
 */
function elementLines(lines: string[], start: number, tag: string) {
  const rest = lines.slice(start).join("\n");
  const tags = /<(\/?)([A-Za-z][\w.-]*)\b[^>]*?(\/?)>/g;
  let depth = 0;
  let match: RegExpExecArray | null;
  while ((match = tags.exec(rest))) {
    const [text, closing, name, selfClosing] = match;
    if (name !== tag) continue;
    if (closing) depth--;
    else if (!selfClosing) depth++;
    if (depth <= 0) return rest.slice(0, match.index + text.length).split("\n").length;
  }
  return 1;
}

/** How many lines the `{...}` expression at `start` takes, by balancing the braces. */
function expressionLines(lines: string[], start: number) {
  let depth = 0;
  for (let i = start; i < lines.length; i++) {
    for (const char of lines[i]) {
      if (char === "{") depth++;
      else if (char === "}") depth--;
    }
    if (depth <= 0) return i - start + 1;
  }
  return 1;
}

function findClose(lines: string[], from: number) {
  for (let i = from; i < lines.length; i++) if (/^:::\s*$/.test(lines[i])) return i;
  return -1;
}

function parseAttributes(text: string) {
  const attrs: Record<string, string> = {};
  for (const [, key, quoted, bare] of text.matchAll(/([\w-]+)\s*=\s*(?:"([^"]*)"|([^\s}]+))/g)) {
    attrs[key] = quoted ?? bare ?? "";
  }
  const cols = Number(attrs.cols);
  return { cols: Number.isFinite(cols) && cols > 0 ? cols : null, ratio: attrs.ratio ?? null, caption: attrs.caption ?? "" };
}

function parseImageLine(line: string): Photo | null {
  const match = line.trim().match(IMAGE_LINE);
  if (!match) return null;
  const [, alt, url, caption] = match;
  const hash = url.indexOf("#");
  return {
    path: hash === -1 ? url : url.slice(0, hash),
    geo: hash === -1 ? "" : url.slice(hash + 1),
    alt: alt ?? "",
    caption: caption ?? "",
  };
}

// --- Writing back

export const imageMarkdown = (photo: Photo) =>
  `![${photo.alt}](${photo.path}${photo.geo ? `#${photo.geo}` : ""}${photo.caption ? ` "${photo.caption.replace(/"/g, "'")}"` : ""})`;

export function photosMarkdown(block: PhotosBlock) {
  const images = block.photos.map(imageMarkdown);
  if (block.layout === "auto") return images.join("\n");
  const attrs: string[] = [];
  if (block.layout === "gallery" && block.cols) attrs.push(`cols=${block.cols}`);
  if (block.ratio) attrs.push(`ratio=${block.ratio}`);
  if (block.caption) attrs.push(`caption="${block.caption.replace(/"/g, "'")}"`);
  return [`:::${block.layout}${attrs.length ? `{${attrs.join(" ")}}` : ""}`, ...images, ":::"].join("\n");
}

const blockMarkdown = (block: Block) => (block.type === "text" ? block.text : photosMarkdown(block));

/** The body again: untouched blocks exactly as they were, edited ones re-written. */
export function serializeBlocks(blocks: Block[]) {
  const pieces = blocks.map((block, index) => {
    const last = index === blocks.length - 1;
    let piece = block.raw;
    const before = index === 0 ? block.before ?? "" : "";
    if (block.dirty) {
      // Keep whatever separated this block from the next; a new one gets a blank line.
      const after = block.raw.match(/\n*$/)?.[0] ?? "";
      piece = blockMarkdown(block) + (after || (last ? "\n" : "\n\n"));
    }
    // A block that used to be the last one needs a separator once something follows it, and a
    // freshly inserted block needs a blank line before it so Markdown still sees two blocks.
    if (!last && !piece.endsWith("\n")) piece += "\n\n";
    else if (!last && blocks[index + 1].raw === "" && !piece.endsWith("\n\n")) piece += "\n";
    return before + piece;
  });
  return pieces.join("");
}

export const emptyText = (text = ""): TextBlock => ({ id: nextId(), type: "text", raw: "", text, dirty: true });

export const emptyPhotos = (layout: PhotoLayout = "gallery", photos: Photo[] = []): PhotosBlock => ({
  id: nextId(),
  type: "photos",
  raw: "",
  layout,
  cols: layout === "gallery" ? 3 : null,
  ratio: null,
  caption: "",
  photos,
  dirty: true,
});

/** A short, human label for a block, used in the outline. */
export function blockLabel(block: Block) {
  if (block.type === "photos") {
    const kind = block.layout === "carousel" ? "Carrusel" : block.photos.length === 1 && block.layout === "auto" ? "Foto" : "Galería";
    return `${kind} · ${block.photos.length}`;
  }
  const heading = block.text.split("\n").find((line) => /^#{1,6}\s/.test(line));
  if (heading) return heading.replace(/^#+\s*/, "");
  const element = block.text.match(/^\s*<([A-Za-z][\w.-]*)/);
  if (element) return `<${element[1]}>`;
  if (/^\s*\{\/\*/.test(block.text)) return "Comentario";
  const first = block.text.split("\n").find((line) => line.trim() !== "") ?? "";
  return first.slice(0, 48) || "(vacío)";
}

/** Heading level of a text block, for the outline's indentation. */
export function headingLevel(block: Block) {
  if (block.type !== "text") return 0;
  const match = block.text.match(/^(#{1,6})\s/m);
  return match ? match[1].length : 0;
}
