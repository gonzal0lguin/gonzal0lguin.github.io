// A Copernicus GLO-30 elevation mosaic, read straight from the agency's cloud-optimised
// GeoTIFFs. No GDAL, no npm packages: a GeoTIFF is a TIFF with a few extra tags, and these
// ones are always 1° × 1° float32 tiles, 1024-px internal tiles, DEFLATE with the
// floating-point predictor. That is all this reader understands, and it says so if a file
// is anything else.
//
// Tiles are downloaded once into .cache/dem (gitignored) — about 40 MB each.

import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { inflateSync } from "node:zlib";

const CACHE = new URL("../../.cache/dem/", import.meta.url);
const SOURCE = "https://copernicus-dem-30m.s3.amazonaws.com";

const tileName = (south, west) => `S${String(south).padStart(2, "0")}_W${String(west).padStart(3, "0")}`;

/** Downloads a 1° tile if it isn't cached yet. Returns null over open sea, where there is no tile. */
async function fetchTile(south, west) {
  const name = tileName(south, west);
  const path = new URL(`${name}.tif`, CACHE);
  const missing = new URL(`${name}.missing`, CACHE);
  if (existsSync(path)) return readFileSync(path);
  if (existsSync(missing)) return null;
  mkdirSync(CACHE, { recursive: true });
  const id = `Copernicus_DSM_COG_10_S${String(south).padStart(2, "0")}_00_W${String(west).padStart(3, "0")}_00_DEM`;
  process.stderr.write(`downloading ${id}…\n`);
  const response = await fetch(`${SOURCE}/${id}/${id}.tif`);
  if (response.status === 404 || response.status === 403) {
    writeFileSync(missing, "");
    return null;
  }
  if (!response.ok) throw new Error(`${id}: HTTP ${response.status}`);
  const buffer = Buffer.from(await response.arrayBuffer());
  writeFileSync(path, buffer);
  return buffer;
}

/** Decodes one Copernicus tile into quarter metres (Int16 holds ±8,191 m at that step). */
function decodeTiff(buffer) {
  const little = buffer.toString("latin1", 0, 2) === "II";
  const u16 = (o) => (little ? buffer.readUInt16LE(o) : buffer.readUInt16BE(o));
  const u32 = (o) => (little ? buffer.readUInt32LE(o) : buffer.readUInt32BE(o));
  const f64 = (o) => (little ? buffer.readDoubleLE(o) : buffer.readDoubleBE(o));
  if (u16(2) !== 42) throw new Error("not a classic TIFF (BigTIFF is not supported)");

  const SIZES = { 1: 1, 2: 1, 3: 2, 4: 4, 11: 4, 12: 8, 16: 8 };
  const ifd = u32(4);
  const tags = {};
  for (let i = 0, n = u16(ifd); i < n; i++) {
    const at = ifd + 2 + i * 12;
    const tag = u16(at);
    const type = u16(at + 2);
    const count = u32(at + 4);
    const size = SIZES[type] * count;
    const offset = size <= 4 ? at + 8 : u32(at + 8);
    const read = { 3: u16, 4: u32, 12: f64 }[type];
    tags[tag] = read ? Array.from({ length: count }, (_, k) => read(offset + k * SIZES[type])) : null;
  }

  const [width] = tags[256];
  const [height] = tags[257];
  const [tileWidth] = tags[322];
  const [tileHeight] = tags[323];
  const compression = tags[259][0];
  const predictor = tags[317]?.[0] ?? 1;
  if (tags[258][0] !== 32 || tags[339]?.[0] !== 3) throw new Error("expected float32 samples");
  if (compression !== 8 && compression !== 32946) throw new Error(`unsupported compression ${compression}`);
  if (predictor !== 3 && predictor !== 1) throw new Error(`unsupported predictor ${predictor}`);

  // Georeferencing: pixel size and the coordinates of the raster's top-left corner.
  const [scaleX, scaleY] = tags[33550];
  const [, , , west, north] = tags[33922];
  // GeoKey 1025 says whether the tie point is a pixel's corner (1) or its centre (2).
  const keys = tags[34735];
  let pixelIsPoint = false;
  for (let k = 4; k < keys.length; k += 4) if (keys[k] === 1025) pixelIsPoint = keys[k + 3] === 2;

  const out = new Int16Array(width * height);
  const across = Math.ceil(width / tileWidth);
  const offsets = tags[324];
  const counts = tags[325];
  const row = new Uint8Array(tileWidth * 4);
  const view = new DataView(row.buffer);
  for (let t = 0; t < offsets.length; t++) {
    const raw = inflateSync(buffer.subarray(offsets[t], offsets[t] + counts[t]));
    const x0 = (t % across) * tileWidth;
    const y0 = Math.floor(t / across) * tileHeight;
    for (let y = 0; y < tileHeight && y0 + y < height; y++) {
      const base = y * tileWidth * 4;
      if (predictor === 3) {
        // Undo the byte-wise horizontal differencing, then re-interleave the four byte
        // planes (most significant first) back into big-endian floats.
        let previous = 0;
        const bytes = new Uint8Array(tileWidth * 4);
        for (let i = 0; i < tileWidth * 4; i++) bytes[i] = previous = (previous + raw[base + i]) & 0xff;
        for (let i = 0; i < tileWidth; i++) {
          row[i * 4] = bytes[i];
          row[i * 4 + 1] = bytes[tileWidth + i];
          row[i * 4 + 2] = bytes[2 * tileWidth + i];
          row[i * 4 + 3] = bytes[3 * tileWidth + i];
        }
      } else {
        for (let i = 0; i < tileWidth; i++) for (let b = 0; b < 4; b++) row[i * 4 + (little ? 3 - b : b)] = raw[base + i * 4 + b];
      }
      for (let x = 0; x < tileWidth && x0 + x < width; x++) {
        const value = view.getFloat32(x * 4, false);
        out[(y0 + y) * width + x0 + x] = Number.isFinite(value) && value > -1000 ? Math.round(value * 4) : 0;
      }
    }
  }
  // Latitude/longitude of the centre of pixel (0, 0).
  const half = pixelIsPoint ? 0 : 0.5;
  return { width, height, data: out, west: west + half * scaleX, north: north - half * scaleY, dx: scaleX, dy: scaleY };
}

/**
 * Loads every 1° tile needed to cover the box and returns a bilinear elevation sampler.
 * Points outside the loaded tiles, or over the sea, read as 0 m.
 */
export async function loadDem({ south, north, west, east }) {
  // GLO-30 is one global grid: nodes every arc-second, with node (0, 0) of every tile sitting
  // exactly on its north-west whole-degree corner (PixelIsPoint). So any node has a global
  // integer address — rows counted south from the equator, columns east from 180°W — and a
  // lookup is arithmetic, with no ambiguity at tile edges.
  const N = 3600;
  const north0 = Math.floor(-north); // first tile row (its north edge, in degrees south)
  const west0 = Math.floor(west) + 180; // first tile column
  const rows = Math.ceil(-south) - north0;
  const cols = Math.ceil(east) + 180 - west0;
  const grid = new Array(rows * cols).fill(null);
  let loaded = 0;
  for (let r = 0; r < rows; r++) {
    for (let c = 0; c < cols; c++) {
      // Copernicus names a tile by its south-west corner: S33_W071 spans 33°S–32°S, 71°W–70°W.
      const buffer = await fetchTile(north0 + r + 1, 180 - (west0 + c));
      if (!buffer) continue;
      const tile = decodeTiff(buffer);
      if (tile.width !== N || tile.height !== N || Math.abs(tile.dx * N - 1) > 1e-9) {
        throw new Error("expected a 3600 × 3600 arc-second tile");
      }
      grid[r * cols + c] = tile.data;
      loaded++;
    }
  }

  /** Elevation of the node at global row `gy` (south from the equator) and column `gx`. */
  function at(gy, gx) {
    const r = Math.floor(gy / N) - north0;
    const c = Math.floor(gx / N) - west0;
    if (r < 0 || c < 0 || r >= rows || c >= cols) return 0;
    const data = grid[r * cols + c];
    return data ? data[(gy - (r + north0) * N) * N + (gx - (c + west0) * N)] * 0.25 : 0;
  }

  /** Elevation of the grid node nearest to the point. */
  function node(lat, lon) {
    return at(Math.round(-lat * N), Math.round((lon + 180) * N));
  }

  /** Bilinear elevation in metres; 0 over the sea or outside the loaded tiles. */
  function elevation(lat, lon) {
    const py = -lat * N;
    const px = (lon + 180) * N;
    const gy = Math.floor(py);
    const gx = Math.floor(px);
    const fy = py - gy;
    const fx = px - gx;
    const ly = gy % N;
    const lx = gx % N;
    let a, b, c, d;
    if (ly < N - 1 && lx < N - 1) {
      // Fast path: all four nodes in the same tile.
      const r = Math.floor(gy / N) - north0;
      const k = Math.floor(gx / N) - west0;
      const data = r >= 0 && k >= 0 && r < rows && k < cols ? grid[r * cols + k] : null;
      if (!data) return 0;
      const i = ly * N + lx;
      a = data[i] * 0.25;
      b = data[i + 1] * 0.25;
      c = data[i + N] * 0.25;
      d = data[i + N + 1] * 0.25;
    } else {
      a = at(gy, gx);
      b = at(gy, gx + 1);
      c = at(gy + 1, gx);
      d = at(gy + 1, gx + 1);
    }
    return a * (1 - fx) * (1 - fy) + b * fx * (1 - fy) + c * (1 - fx) * fy + d * fx * fy;
  }

  /**
   * Bicubic (Catmull–Rom) elevation: slower, but smooth in slope as well as height, which is
   * what the 3D mesh near the viewer needs — bilinear surfaces crease along every grid line.
   */
  function smoothElevation(lat, lon) {
    const py = -lat * N;
    const px = (lon + 180) * N;
    const gy = Math.floor(py);
    const gx = Math.floor(px);
    const fy = py - gy;
    const fx = px - gx;
    const weights = (t) => [
      ((-t + 2) * t - 1) * t * 0.5,
      ((3 * t - 5) * t * t + 2) * 0.5,
      ((-3 * t + 4) * t + 1) * t * 0.5,
      (t - 1) * t * t * 0.5,
    ];
    const wx = weights(fx);
    const wy = weights(fy);
    let sum = 0;
    for (let j = 0; j < 4; j++) {
      let row = 0;
      for (let i = 0; i < 4; i++) row += wx[i] * at(gy - 1 + j, gx - 1 + i);
      sum += wy[j] * row;
    }
    return sum;
  }

  return { elevation, smoothElevation, node, tiles: loaded };
}
