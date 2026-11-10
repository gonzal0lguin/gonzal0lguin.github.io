// Brings the photo-topo editor and its example scenes over from the mountain-tools repository
// into public/assets/phototopo/. Run it by hand when the editor or the examples change there,
// and commit the result: `npm run phototopo` (the checkout is expected next to this one, or at
// the path in MOUNTAIN_TOOLS).
//
// WHY A COPY. The editor is written and tested in mountain-tools, where it ships inside the
// Python package and is served by `mtn topo edit`. It is plain ES modules and CSS with no build
// step precisely so the same files can sit on a static site. A copy, rather than a submodule or
// an npm dependency, keeps the GitHub Pages build free of a second repository: the deploy only
// ever sees files that are in this one.
//
// WHY IT WORKS WITHOUT A SERVER. A scene file stores terrain and track as bearing, elevation
// angle and distance from the camera, so the page can re-aim the camera with the numbers it
// has. What it cannot do here is move the camera's position: that needs the elevation model,
// which is the Python side, and the editor says so instead of offering it.
//
// The example photos are re-encoded smaller and without metadata; the originals carry the
// phone's GPS position in their EXIF.

import { copyFileSync, existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { join, resolve } from "node:path";
import sharp from "sharp";

const ROOT = fileURLToPath(new URL("../", import.meta.url));
const SOURCE = resolve(process.env.MOUNTAIN_TOOLS ?? join(ROOT, "../mountain-tools"));
const EDITOR = join(SOURCE, "src/mountain_tools/phototopo/editor");
const EXAMPLES = join(SOURCE, "examples");
const OUT = join(ROOT, "public/assets/phototopo");
const DEMOS = ["union", "san-gabriel", "algarrobo"];
const WIDTH = 1600;

if (!existsSync(EDITOR)) {
  console.error(`no mountain-tools checkout at ${SOURCE} (set MOUNTAIN_TOOLS to its path)`);
  process.exit(1);
}

mkdirSync(join(OUT, "demos"), { recursive: true });
for (const file of ["editor.js", "editor.css"]) copyFileSync(join(EDITOR, file), join(OUT, file));

for (const name of DEMOS) {
  const scene = JSON.parse(readFileSync(join(EXAMPLES, `${name}.scene.json`), "utf8"));
  // The scene keeps the dimensions it was fitted at; the editor scales any copy of the photo
  // to them, so a smaller picture needs no change to the numbers.
  await sharp(join(EXAMPLES, scene.photo.file)).rotate().resize(WIDTH).jpeg({ quality: 82, mozjpeg: true }).toFile(join(OUT, "demos", `${name}.jpg`));
  writeFileSync(join(OUT, "demos", `${name}.scene.json`), `${JSON.stringify({ ...scene, photo: { ...scene.photo, file: `${name}.jpg` } })}\n`);
  console.log(`${name}: ${scene.track.points.length} track points, skylines ${scene.fit.error} px apart`);
}
console.log(`editor and ${DEMOS.length} scenes → public/assets/phototopo/`);
