// The picture a link shows when it's pasted somewhere: 1200×630 and around 100 KB, drawn at
// build time from what the site already knows (src/pages/[...lang]/og/[kind]/[key].jpg.ts).
//
// The text goes through sharp's text input and a font in the repo (scripts/fonts/, Noto Sans,
// OFL) rather than an SVG <text>: librsvg uses whatever fonts the machine has, and a CI runner
// without them turns "Volcán" into boxes without any error to go on.

import { existsSync } from "node:fs";
import path from "node:path";
import sharp from "sharp";
import { thumbOf } from "./image-variants.mjs";
import { OG_HEIGHT, OG_WIDTH } from "./og-size";

const MARGIN = 72;
const INK = "#f2f5f9";
const MUTED = "#b6c1cf";
const ACCENT = "#a4e58c";
const BACKGROUND = "#1d222b";
const BOLD = path.resolve("scripts/fonts/NotoSans-Bold.ttf");
const REGULAR = path.resolve("scripts/fonts/NotoSans-Regular.ttf");

export interface OgCard {
  eyebrow: string;
  title: string;
  detail?: string;
  /** Public path of a photo for the background. */
  photo?: string;
  /** A summit's horizon as elevation angles, north to north (lib/summits.ts, skylineProfile). */
  skyline?: number[] | null;
}

const escape = (text: string) => text.replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;");

async function text(content: string, font: string, fontfile: string, color: string, width: number) {
  const buffer = await sharp({
    text: { text: `<span foreground="${color}">${escape(content)}</span>`, font, fontfile, width, rgba: true, dpi: 72 },
  })
    .png()
    .toBuffer();
  const { height } = await sharp(buffer).metadata();
  return { buffer, height: height! };
}

function skylineSvg(skyline: number[]) {
  // The same shared scale as the small drawings on the summit cards, so a flat view stays flat.
  const LOW = -3;
  const HIGH = 9;
  const REACH = 210;
  const y = (angle: number) => OG_HEIGHT - ((Math.min(HIGH, Math.max(LOW, angle)) - LOW) / (HIGH - LOW)) * REACH;
  const line = skyline.map((angle, i) => `${((i / (skyline.length - 1)) * OG_WIDTH).toFixed(1)} ${y(angle).toFixed(1)}`).join("L");
  return Buffer.from(`<svg xmlns="http://www.w3.org/2000/svg" width="${OG_WIDTH}" height="${OG_HEIGHT}">
    <defs><linearGradient id="g" x1="0" x2="0" y1="0" y2="1"><stop offset="0.55" stop-color="${ACCENT}" stop-opacity="0.34"/><stop offset="1" stop-color="${ACCENT}" stop-opacity="0.04"/></linearGradient></defs>
    <path d="M0 ${OG_HEIGHT}L${line}L${OG_WIDTH} ${OG_HEIGHT}Z" fill="url(#g)"/>
    <path d="M${line}" fill="none" stroke="${ACCENT}" stroke-width="2.5"/>
  </svg>`);
}

/** Dark from the left where the text sits, so any photo stays readable behind it. */
const SCRIM = Buffer.from(`<svg xmlns="http://www.w3.org/2000/svg" width="${OG_WIDTH}" height="${OG_HEIGHT}">
  <defs><linearGradient id="s"><stop offset="0" stop-color="${BACKGROUND}" stop-opacity="0.96"/><stop offset="0.6" stop-color="${BACKGROUND}" stop-opacity="0.74"/><stop offset="1" stop-color="${BACKGROUND}" stop-opacity="0.32"/></linearGradient></defs>
  <rect width="${OG_WIDTH}" height="${OG_HEIGHT}" fill="url(#s)"/></svg>`);

export async function renderOgCard({ eyebrow, title, detail, photo, skyline }: OgCard): Promise<Buffer> {
  const layers: sharp.OverlayOptions[] = [];
  const photoUrl = photo ? thumbOf(photo, 1600) : null;
  const photoFile = photoUrl ? path.join("public", decodeURI(photoUrl)) : null;
  if (photoFile && existsSync(photoFile)) {
    layers.push({ input: await sharp(photoFile).rotate().resize(OG_WIDTH, OG_HEIGHT, { fit: "cover" }).toBuffer() }, { input: SCRIM });
  }
  if (skyline?.length) layers.push({ input: skylineSvg(skyline) });

  const width = OG_WIDTH - 2 * MARGIN;
  const titleSize = title.length <= 26 ? 76 : title.length <= 52 ? 60 : 46;
  const blocks = [
    { ...(await text(eyebrow.toUpperCase(), "Noto Sans Bold 24", BOLD, ACCENT, width)), gap: 20 },
    { ...(await text(title, `Noto Sans Bold ${titleSize}`, BOLD, INK, width)), gap: 22 },
    ...(detail ? [{ ...(await text(detail, "Noto Sans 32", REGULAR, MUTED, width)), gap: 0 }] : []),
  ];
  let top = MARGIN - 8;
  for (const block of blocks) {
    layers.push({ input: block.buffer, left: MARGIN, top });
    top += block.height + block.gap;
  }

  const signature = await text("Gonzalo Olguín · gonzal0lguin.github.io", "Noto Sans Bold 22", BOLD, INK, width);
  layers.push({ input: signature.buffer, left: MARGIN, top: OG_HEIGHT - MARGIN + 24 - signature.height });

  return sharp({ create: { width: OG_WIDTH, height: OG_HEIGHT, channels: 3, background: BACKGROUND } })
    .composite(layers)
    .jpeg({ quality: 80, mozjpeg: true })
    .toBuffer();
}
