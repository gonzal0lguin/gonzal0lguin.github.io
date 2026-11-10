---
translationHash: "df68614da63ae1cd"
title: "mountain-tools: user manual"
category: "hobby-projects"
description: "The complete manual for mountain-tools, my Python toolbox for mountain data. For now it has one tool: photo topo, which draws your GPX route on a photo of the mountain by fitting the camera to the terrain."
pubDate: 2026-11-10
heroImage: /assets/img/headers/mountain-tools.jpg
author: Gonzalo Olguin
tags: [montaña, gpx, python, herramientas, fotos]
---

This is the manual for [mountain-tools](https://github.com/gonzal0lguin/mountain-tools), a Python repository where I'm collecting the tools I build for the mountains. It's a baby repo: today it has a single tool, **photo topo**, but it's set up so more can arrive, all behind one command (`mtn`) and sharing the same pieces: the elevation model, the GPX reader, photo metadata and mountain names.

If you're more interested in the *why* than the *how*, the story is in [Looking at a mountain the way a robot would](/en/projects/mirar-como-un-robot/): it's the same maths a robot uses to project its trajectory onto its camera image. Here I get straight to the point.

## The tools

| tool | command | what it does | where to try it |
| --- | --- | --- | --- |
| photo topo | `mtn topo` | fits a photo's camera to the terrain and draws a GPX route on it | [on this site](/en/ascents/tools/photo-topo/), with my photos |

When there are more, they'll show up in this table and each one will get its own section below.

## Installation

You need Python 3.10 or newer. The only dependencies are NumPy, SciPy and Pillow: no GDAL, no rasterio, nothing to compile.

```bash
git clone https://github.com/gonzal0lguin/mountain-tools && cd mountain-tools
python3 -m venv .venv && .venv/bin/pip install -e .
.venv/bin/mtn --help
```

I really recommend using the virtual environment (`.venv`). On my computer the system Python has a mix of NumPy 1 and 2 that falls over on import, and it took me a while to figure that out.

Elevation models and OpenStreetMap answers are kept in `~/.cache/mountain-tools`. If you want them somewhere else, set `MOUNTAIN_TOOLS_CACHE`.

## Photo topo

A route line on a photo is only right if you know the camera: where it stood, which way it pointed, how much it was tilted and rolled, and its focal length. Photo topo works all that out by making the photo agree with the terrain. It reads the skyline from the photo, computes the skyline [Copernicus GLO-30](https://dataspace.copernicus.eu/explore-data/data-collections/copernicus-contributing-missions/collections-description/COP-DEM) would show from a candidate camera, and keeps moving the camera until the two match. Through that camera every point of the track is projected, and for each one it checks whether there's a line of sight or a ridge hides it.

It's three steps:

```bash
mtn topo fit   photo.jpg route.gpx     # finds the camera → photo.scene.json
mtn topo edit  photo.scene.json        # adjust and style it in the browser
mtn topo render photo.scene.json       # → photo.topo.jpg
```

### What you need

- **A photo where the ridge shows against the sky.** That's what gets matched. A face that fills the whole frame, with no sky, also works but takes more effort (see [control points](#when-it-doesnt-fit)).
- **The GPX of the outing.** A watch track or a planned route, either is fine.
- **The original photo, with its EXIF.** That's where the GPS, the compass and the lens are, which tell the search where to start. WhatsApp and almost every messaging app strip that data, so use the photo off your phone, not the one someone sent you.

If the photo has no GPS, give the position by hand with `--lat` and `--lon`. If the compass is off, `--heading`; and if the lens is unknown, `--focal35` with the 35 mm equivalent focal length (the main camera of almost every phone is around 24–26 mm).

There's one thing that helps a lot and needs nothing from you: if you took the photo while recording the track, the photo's time says where you were standing at that second, and the search starts from there.

### Step 1: find the camera

```bash
.venv/bin/mtn topo fit examples/union.jpg examples/union.gpx
```

The first time in a region it's slow, because it has to download the elevation models: about 40 MB per 1° square, and the first search looks all 360° around so it can want four to six of them. It says so in the terminal, so if it looks stuck, it's probably downloading. After that a fit takes a few seconds and ends like this:

```
camera -33.74815, -70.05055 at 3255 m, 24 m from where it started
heading 210.6°, tilt 10.5°, roll 2.3°, lens 0.763 of the width
skylines 7.4 px apart on 96% of the ridge
wrote examples/union.scene.json
```

The two numbers on the last line are the ones that matter:

- **px apart**: how many pixels, on average, the photo's skyline is from the computed one.
- **of the ridge**: over how much of the skyline the two agree at all.

Read them together. A low error over 40% of the ridge may mean it fitted the wrong stretch perfectly. If agreement drops below 60% or the error goes over 0.8% of the photo's width, the command tells you the fit is weak and to check it in the editor.

All the options:

| option | what for |
| --- | --- |
| `-o`, `--output` | where to save the scene (default `photo.scene.json`, next to the photo) |
| `--lat`, `--lon` | camera position, if the photo has no GPS or a wrong one |
| `--heading` | which way it pointed, in degrees from north |
| `--focal35` | 35 mm equivalent focal length |
| `--reach` | how many kilometres out to look for skyline (default 60) |
| `--no-peaks` | don't ask OpenStreetMap for mountain names |

One detail: the scene refers to the photo and the GPX by name, so the three files have to be in the same folder. Otherwise the command refuses.

#### What happens inside

For anyone interested, there are four pieces, and each has its long explanation at the top of its file in the repo:

1. **Reading the photo's skyline** (`skyline.py`). Pixels aren't classified as sky or mountain, because snow under an overcast sky is whiter than the cloud. It looks for a boundary: one row per column with a strong change, smooth above, not jumping much from one column to the next. The cheapest left-to-right path is found with dynamic programming. On its own that gets fooled by clouds (the topmost edge is the cloud's), so it then learns this photo's sky colour and charges for every sky-coloured pixel left *below* the boundary. Columns it isn't sure about are left empty instead of guessed.
2. **Computing the model's skyline** (`terrain.py`). It's the same simulated LiDAR as in the [robot post](/en/projects/mirar-como-un-robot/): a fan of rays from the camera, walking the terrain and keeping the highest angle.
3. **Matching them** (`fit.py`). It's an extrinsic calibration against a mountain. Turning, tilting or zooming doesn't need to look at the terrain again, only to reproject, so it's cheap; moving the camera does, so the position moves little and carefully. The error saturates, so the column where the reader followed a tree loses the vote instead of dragging everything along.
4. **Projecting the route** (`scene.py`). Each track point becomes a bearing, angle and distance from the camera, and gets checked for whether it's seen or hidden.

### Step 2: the editor

```bash
.venv/bin/mtn topo edit examples/union.scene.json
```

It opens a page in your browser, served from your own computer (`127.0.0.1`, port 8765; change it with `--port`, and `--no-browser` opens nothing). It has three tabs.

#### Align

The terrain outline is drawn over the photo in white, and the skyline read from the photo in pink.

- **Drag** the photo to move the outline. **Scroll** to zoom the lens, **Shift + scroll** to roll it.
- **Snap to skyline** finishes the job after a rough drag.
- If what's wrong is the camera position, move it a few metres and **Refit from here**: the terrain is recomputed from that spot, starting from your alignment.
- If the pink line follows a cloud or a row of trees, **Draw** the real ridge over it or **Erase** the bad stretch, and snap again. **Fit again from scratch** forgets everything and starts over from what the photo said.
- **Control points**: if you recognise a summit in the photo, choose it, press *Click it on the photo* and click on it. The outline jumps there, and every later snap and refit keeps it in place.

#### Style

This is where most of the time goes, honestly.

- **Route**: colour, width, solid, dashed or dotted, opacity and smoothing. What to do with the part hidden behind the mountain (same, dashed or hidden). From and to which kilometre to draw, or just the way up.
- **Stretches**: *Select a stretch* and two clicks on the route, and that part gets its own colour, line and width. I use it for the variant, the glacier or the scrambling bit.
- **Symbols**: camp, hut, flag, water and others, with a name if you want. Dropped on the line it sticks to the route; anywhere else it sticks to that spot on the mountain even if you re-aim the camera.
- **Summits**: the named mountains in sight, from OpenStreetMap. Tick the ones you want, drag the name where it looks good and its pointer line follows. Height and capitals switch on and off.
- **Fix the route**: with 30-metre cells, near the camera the route sometimes ends up floating or buried. Switch on *Adjust route* and drag the line to where it really goes. Each drag leaves a handle (double-click to remove it) and the route bends around it.
- **Text** you can drag anywhere, and a **Frame** (`4:5`, `1:1`, `9:16`, `16:9`) for posting.

#### Export

JPG or PNG at the photo's full resolution and in the chosen frame, and the scene file to carry on another day. In the local editor, *save* overwrites the `.scene.json` on your disk.

### Step 3: draw it from the terminal

```bash
.venv/bin/mtn topo render examples/union.scene.json --width 1600
```

It reads the same style you left in the editor and writes `union.topo.jpg` (or whatever you say with `-o`). Useful for doing several at once or putting it in a script, and it's also the proof that the scene means the same thing outside the browser: the drawing rules are written twice on purpose, once in Python and once in JavaScript.

### Without installing anything

The editor is also [on this site](/en/ascents/tools/photo-topo/), with three of my photos loaded (Unión, San Gabriel and the [Algarrobo traverse](/en/ascents/algarrobo-traverse/)). It works without a server because the scene stores everything as angles from the camera: re-aiming is arithmetic the browser can do. What it can't do there is *move* the camera, because that needs the elevation model, which lives on the Python side. You can also drop a scene and a photo of your own onto that page.

:::gallery{cols=2 ratio=auto}
![The Algarrobo traverse track on a photo, with six named summits](mountain-tools/algarrobo-topo.jpg "Algarrobo traverse: the route crosses the whole photo and OpenStreetMap supplies the names")
![The San Gabriel ascent track on a photo at dawn](mountain-tools/san-gabriel-topo.jpg "San Gabriel at dawn: snow the same colour as the sky")
:::

### When it doesn't fit

The fit is automatic but not infallible. What has happened to me, and how I fix it:

- **It fitted another stretch of ridge.** A similar-looking piece of the range can fit almost as well. Drag the outline by eye to where it goes and *Snap to skyline*. If it's still off, *Refit from here*.
- **Clouds over the ridge.** It almost always handles them on its own. This photo of Punta Canaleta is the one that taught it: before, it followed the tops of the cumulus and ended up 60° off. If it still gets confused, erase the stretch of pink line that runs along the cloud.
- **Trees, cacti or a person in front of the ridge.** The reader can't tell them apart. Erase or draw.
- **A face that fills the photo, with no skyline.** That's what control points are for: with two or three summits marked by hand the camera is pinned down. A click is taken seriously but not blindly: if it goes strongly against everything else, a cyan line that won't close stays there, warning you something doesn't add up.
- **The route near the camera looks odd.** If the photo is from the same outing, the nearby stretch is drawn from the watch's positions and heights instead of the elevation model, starting in a straight line from your feet. Beyond a few hundred metres it goes back to the model, and at that hand-off the route sometimes kinks. That's what *Adjust route* is for.
- **The photo's GPS is wrong.** It happens more than you'd think. `--lat` and `--lon`, or move the camera in the local editor.

:::gallery{cols=2 ratio=auto}
![Photo of Punta Canaleta with cumulus over the ridge](mountain-tools/canaleta-foto.jpg "The original photo: cumulus right over the ridge")
![The same photo with the route drawn](mountain-tools/canaleta-topo.jpg "The result, with the hidden part dashed")
:::

### The scene file

`fit` writes a `*.scene.json` next to the photo. It's the whole result and the only connection between the parts: the fit writes it, the editor and `render` read it, and so can you.

The most important design rule in the repo is that **there are no terrain pixels in it**. Every point of the skyline, the ridges and the track is a **bearing, an elevation angle and a distance** from the camera. That's why turning, tilting or zooming is reprojecting numbers that are already there, and why the editor works as a static page. A small piece of the Unión scene:

```json
{
  "photo": { "file": "union.jpg", "width": 2016, "height": 1512 },
  "camera": { "lat": -33.74815, "lon": -70.050551, "ground": 3254.8, "eye": 1.6,
              "yaw": 210.6268, "pitch": 10.4911, "roll": 2.2778, "lens": 0.7634 },
  "prior": { "position": "photo GPS", "heading": "photo compass", "lens_from": "photo" },
  "fit": { "error": 7.4, "agreement": 0.96, "moved": 23.7 },
  "track": { "points": [[198.489, 18.023, 697, 1, 1.068, 3495, 0], "…"] },
  "peaks": [{ "name": "Cerro Unión", "ele": 3652, "az": 203.06, "alt": 22.605, "dist": 971, "show": true }]
}
```

| key | what it holds |
| --- | --- |
| `photo` | name, width and height of the photo the pixels refer to |
| `camera` | `lat`, `lon`, `ground` (m), `eye` (m above the ground), `yaw`, `pitch`, `roll` (degrees), `lens` (focal length divided by width, so it survives shrinking the photo) |
| `prior`, `fit` | where the search started and how it ended |
| `skyline`, `ridges` | the model's skyline and the crests inside it |
| `track.points` | per point: `[bearing, angle, distance, visible, km, elevation, from_track]` |
| `track.pins` | corrections made by hand in *Adjust route* |
| `peaks`, `labels`, `marks` | named summits, text anchored to a direction, symbols |
| `controls` | control points: a pixel (as a fraction of the photo) and the summit it is |
| `photo_skyline` | the skyline read from the photo, in pixels, with gaps where it wasn't trusted |
| `style` | everything visual; lengths are fractions of the photo's width |

The repo's README has the full table.

### As a library

Everything the command does can be called from Python. For example, fitting and drawing in another colour without going through the editor:

```python
from mountain_tools.phototopo.scene import build_scene, save_scene
from mountain_tools.phototopo.render import render

scene = build_scene("photo.jpg", "route.gpx")
scene["style"]["track"]["color"] = "#00c2ff"
scene["style"]["hidden"]["mode"] = "dashed"
save_scene(scene, "photo.scene.json")
render(scene, "photo.jpg").save("photo.topo.jpg", quality=92)
```

And the shared pieces work on their own:

```python
from mountain_tools.dem import Dem
from mountain_tools.gpx import read_gpx
from mountain_tools.phototopo.terrain import scan

dem = Dem()
dem.elevation(-33.7463, -70.0479)          # the model's height at that point, in metres
track = read_gpx("route.gpx")              # points, heights, times and waypoints
# the skyline from there, like a simulated LiDAR: bearing 210° ± 30°, a ray every 0.1°, out to 60 km
view = scan(dem, -33.7482, -70.0506, 1.6, center=210, half=30, step=0.1, reach=60_000)
view.skyline, view.skyline_distance        # angle and distance of the skyline at each bearing
```

All angles are in degrees, bearings clockwise from north and distances in metres.

### The editor on your own site

This is how it's set up on this site. The editor is two files, `editor.js` and `editor.css`, with nothing to compile and no dependencies. Copy them to any static host, put a scene and its photo next to them, and mount it:

```html
<link rel="stylesheet" href="/phototopo/editor.css" />
<div id="topo"></div>
<script type="module">
  import { mountEditor } from "/phototopo/editor.js";
  mountEditor(document.getElementById("topo"), {
    demos: [{ name: "Cerro Unión", scene: "/phototopo/union.scene.json", photo: "/phototopo/union.jpg" }],
    strings: { Align: "Alinear", Style: "Estilo", Export: "Exportar" },
  });
</script>
```

- `demos` are the scenes offered in the menu; the first one loads straight away.
- `strings` translates the interface, keyed by the English text. Anything missing stays in English.
- `api` points to a running `mtn topo edit`, if you want to be able to move the camera and save.

Colours come from CSS variables (`--pt-bg`, `--pt-panel`, `--pt-fg`, `--pt-muted`, `--pt-line`, `--pt-accent`, `--pt-accent-fg`), so it adapts to any site's theme. The photo can be a smaller copy of the original: the scene keeps the dimensions it was fitted at and the editor scales.

### Limits

- **The model has 30-metre cells.** Terrain closer than about 100 metres can't hide anything (otherwise the very knoll you're standing on would block the whole view), and a route a few hundred metres away can sit tens of pixels off. Distant faces come out best.
- **Only between 50° north and south.** Beyond that the Copernicus tiles change shape and the reader doesn't understand them.
- **No lens distortion.** Phones already correct it; an uncorrected fisheye won't fit.
- **One photo at a time.**
- The terminal renderer looks for a system font (DejaVu, Noto, Liberation, Arial). If it finds none, "Cerro Unión" comes out with a box instead of the ó; point `MOUNTAIN_TOOLS_FONT` at a `.ttf` you have.

## Getting your hands dirty

```bash
.venv/bin/pip install -e '.[dev]'
.venv/bin/python -m pytest -m "not slow"     # seconds
.venv/bin/python -m pytest                   # also refits the example photos
```

If you have ROS installed (like me), its pytest plugins sneak in and crash it at startup: prefix it with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.

The photos in `examples/` are the regression test, and each one broke something at some point: Unión (a lone cone, where the skyline barely says where you were standing), San Gabriel (snow the colour of the sky), Algarrobo, the Punta Canaleta cumulus one, and two taken on a snow arête, where the nearby route can't come from the model. The test tolerances come from fits I checked by eye, so if a change moves a camera outside them, the right thing is to look at the photo before widening the tolerance.

![The route on a photo taken on a snow arête](mountain-tools/arista-topo.jpg "On the Punta Canaleta arête: the stretch by the camera comes from the watch's track, not the model")

### Adding a tool

A new tool is a subpackage next to `phototopo` and a subcommand in `cli.py` (`mtn <tool> …`). Whatever two tools need moves up a level, next to `dem.py` and `gpx.py`:

```
src/mountain_tools/
  geo.py dem.py gpx.py exif.py osm.py   shared: geodesy, elevation, tracks, photos, names
  cli.py                                `mtn`, one subcommand per tool
  phototopo/                            the first one
```

Each module opens with a comment explaining what it's for and why it works that way, not what it does line by line. And a hand-written reader is preferred over a dependency when the format is small: the GPX reader is a handful of regular expressions, and the Copernicus GeoTIFF is read without GDAL.

## What's next

For photo topo, roughly in order: control points on places that aren't named summits (a col, a point on the route), a smoother hand-off between the stretch drawn from the track and the one drawn from the model, using prominence to choose which summits to name, colouring the track by elevation, and several photos of one route.

And beyond photo topo, the other tools that today live as loose scripts on this site, like the summit panoramas and the route analysis, look like they'll end up here too. But that's for when their turn comes.
