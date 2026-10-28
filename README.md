# Gonzalo Olguín | Portfolio & Mountaineering

Welcome to the source code for my personal portfolio, robotics/AI projects showcase, and mountain ascents blog. This website is built with [Astro](https://astro.build), [TailwindCSS](https://tailwindcss.com/), and [DaisyUI](https://daisyui.com/).

## Live Site
Visit the live site: [gonzal0lguin.github.io](https://gonzal0lguin.github.io)

## Local Development Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/gonzal0lguin/gonzal0lguin.github.io.git
   cd gonzal0lguin.github.io
   ```

2. **Install dependencies** (the GitHub Actions deploy uses `npm ci`, so stick to npm):
   ```bash
   npm install
   ```

3. **Start the development server:**
   ```bash
   npm run dev
   ```
   The site will be available at `http://localhost:4321` (English at `http://localhost:4321/en/`).

4. **Build for production:**
   ```bash
   npm run build
   ```

## Languages (Spanish / English)

The site is bilingual. **Spanish is the default** and lives at the root (`/cv`, `/ascents/...`); **English** lives under `/en/` (`/en/cv`, `/en/ascents/...`). The 🇨🇱/🇬🇧 buttons in the sidebar (and the mobile header) switch between the two versions of the current page.

- **UI text** (menu, buttons, section titles, short intros) is in `src/i18n/ui.ts`, with one entry per language.
- **Content** is written in Spanish. Each content file has an optional English twin in an `en/` subfolder with the same file name:
  ```
  src/content/ascents/volcan-osorno.md       ← Spanish (source)
  src/content/ascents/en/volcan-osorno.md    ← English
  ```
  If the English file doesn't exist, the English site shows the Spanish text with a "not translated yet" notice.
- **The CV and the home intro** are content too: `src/content/pages/cv.mdx` and `src/content/pages/home.md` (plus their `en/` versions).
- Pages are generated for both languages from `src/pages/[...lang]/`.

### Automatic translation

`npm run translate` uses Claude to create or refresh the English files from the Spanish ones. It needs an [Anthropic API key](https://console.anthropic.com/) in `ANTHROPIC_API_KEY`.

```bash
ANTHROPIC_API_KEY=sk-ant-... npm run translate   # translate missing or outdated English files
npm run translate -- --dry-run                    # only show what would be translated
npm run translate -- src/content/ascents/union.md # translate specific files
```

Each generated English file records a `translationHash` of the Spanish file it came from, so after you edit a Spanish file, the next run retranslates only that one. Only the `title`, `description`, `badge`, `mountain`, `greeting` and `subtitle` frontmatter fields are translated; dates, tags, coordinates, images, etc. are always copied from the Spanish file.

- **Fixing a translation by hand:** edit the English file, then run `npm run translate -- --stamp` to mark it as up to date. It will still be regenerated the next time you change the Spanish file.
- **Keeping a hand-written translation for good:** delete its `translationHash` line. The script then never touches it (unless you pass `--force`).

**On every push (optional):** add a repository secret named `ANTHROPIC_API_KEY` (Settings → Secrets and variables → Actions). The deploy workflow will then translate new or changed Spanish content, commit the English files back to `main`, and deploy them. Without the secret, that step is skipped.

## Managing Content

Everything below can also be done from a browser: `npm run dev` and open
**[localhost:4321/edit](http://localhost:4321/edit)**, a development-only editor for this
repository's own content — entries as editable blocks, drag & drop photos with their layout and
their place on the route map, the GPX routes in the route tool, plus checks and commits.
It is never part of a build or of the deployed site. See **[editing.md](editing.md)**.

This site uses Astro's Content Collections to manage structured markdown content. There are three main collections: **Projects**, **Ascents**, and **Blog**.

### 1. Projects
Located in `src/content/projects/`.
To add a new project, create a Markdown (`.md`) file in this folder. The file name becomes the URL: `gonzobot.md` → `/projects/gonzobot`.

**Frontmatter format:**
```yaml
---
title: "Project Title"
description: "A short description of the project"
pubDate: "2024-01-27"
heroImage: "/assets/img/headers/project-image.png"
badge: "Destacado"       # Optional badge to display on the card
category: "academic-projects"  # academic-projects | hobby-projects | personal
tags: ["ros2", "slam"]   # Optional tags
---
```
Write your project details using standard Markdown below the frontmatter.

### 2. Ascents
Located in `src/content/ascents/`.
This collection is specialized for mountain ascents and tracks specific mountain metadata.

**Frontmatter format:**
```yaml
---
title: "La Cruz"
description: "Ascent description"
pubDate: "2024-03-01"
mountain: "Cerro La Cruz"  # Optional
elevation: 2552            # Optional (in meters)
latitude: -33.435          # Optional
longitude: -70.470         # Optional
heroImage: "/assets/img/headers/mountain-image.jpg"
badge: "Solo"
route: "/assets/tracks/la-cruz.gpx"  # Optional GPS track (see below)
timezone: "America/Santiago" # Optional, for the route's clock times (this is the default)
tags: ["andes", "day-hike"]
---
```

#### GPS routes

Add a `route:` field pointing to a `.gpx` file in `public/` (e.g. export it from your watch, Strava or Garmin and save it in `public/assets/lib/`). The entry then shows an interactive route card above the text. Without a `route`, nothing is shown.

If the route was recorded in several files (the watch was stopped and restarted), list them all. They are put in chronological order by their timestamps, whatever order you list them in, and analyzed as one route; the download button offers the stitched file (`/routes/<entry>.gpx`).

```yaml
route:
  - "/assets/lib/punta-canaleta-17-10-2026_1.gpx"
  - "/assets/lib/punta-canaleta-18-10-2026_1.gpx"
```

To mark points that aren't in the GPX file, anywhere on the map, add `waypoints:` (`kind`: camp, summit, view, water, hut, crux, danger, parking or note, the default; `ele` and `note` are optional). A summit you didn't reach is the typical case: once a summit is marked away from the route, the route's own highest point shows as **High point** instead of Summit.

```yaml
waypoints:
  - { name: "Cerro Plomo", kind: summit, lat: -33.2385, lon: -70.2156, ele: 5424, note: "We turned back at 5,100 m" }
```

They appear on the map, in 3D, in the analysis and in the downloaded GPX (which is then generated at `/routes/<entry>.gpx`).

What the card shows:

- Map with topographic, satellite and street layers, colored by **grade**, **elevation**, **speed**, **vertical speed** or **heart rate** (picked in the legend). Start, summit, camps, km markers and the file's waypoints (with icons by kind). Clicking any point of the route, or a marker, opens it in **Google Earth 3D** / **Google Maps 3D** with a pin on it, and copies its coordinates.
- **3D** button: the route on real terrain (MapLibre + free AWS terrain tiles, satellite or topo). ▶ replay flies the camera along the route. MapLibre is only downloaded when 3D is opened.
- **Open in Google Earth**: downloads the route as KML (`/routes/<entry>.kml`, built by `src/lib/kml.ts`: the track draped on the terrain, a line per day, the points and the post's located photos) and opens Google Earth on the web, where it's imported with *Projects → New project → Import KML file from computer*. Google Earth has no way to load a file from a link, so this is as direct as it gets; Google Earth Pro and the phone app open the file directly.
- Elevation profile linked to the map. Drag across it to select a section: distance, gain/loss, average grade, moving and total time, vertical speed, speed and heart rate for that section.
- Stats, multi-day breakdown (a pause over 3 hours starts a new day; the spot is marked as a camp) and a "Download GPX" button.
- **Detailed analysis** (collapsed in posts): marked points, a summary (effort km, climbing/descent rate, pace vs. DIN 33466, time stopped, terrain share, oxygen at the high point, daylight), climbs and descents, splits, distance by grade, time by altitude, stops, heart rate zones, and a **planner** that estimates the timings at your pace and warns about finishing after dark.

The GPX files are analyzed when the site is built (`src/lib/gpx.ts`, `src/lib/gpx-core.ts`), so the browser only downloads a compact summary; the map code is only loaded on pages that have a route. Distance and elevation gain on the Mountaineering page are the totals of all entries with a route.

#### Route analyzer ("Procesar ruta")

`/ascents/route-tool` gives visitors the same card for their own GPX files, processed entirely in the browser. Several files are stitched like above. In the tool, points can be **marked** (camp, summit, viewpoint, water, hut, crux, danger, parking, note) by clicking the map or the profile, dragged, edited and deleted, and placed anywhere by typing their coordinates in the editor; stops can be marked from the Stops tab. Marks are saved in the downloaded GPX as standard waypoints (`<sym>` for Garmin and most apps, `<type>` for this site), and remembered in the browser for the same files. A GPX exported from the tool can be used as a post's `route`, with its marks.

The tool is a self-contained component (`src/components/RouteTool.astro`, `src/scripts/route-tool.ts`), ready to move to a future tools section.

#### Summits and summit panoramas

A summit is its own thing, separate from the outings that reach it: one YAML file per summit in `src/content/summits/` (the `summits` collection), and each ascent lists the summits it reached (or attempted) by file name:

```yaml
# src/content/ascents/provincia-invernal.md
summits: [provincia]          # a traverse lists several: [manquehue, manquehuito, el-penon]
attempt: true                 # only when the summit wasn't reached

# src/content/summits/provincia.yaml
name: Cerro Provincia
elevation: 2751
lat: -33.4265
lon: -70.4349
osm: 373621475                # the OpenStreetMap node, to recognise it in other summits' views
region: Santiago · Precordillera
panorama: true                # compute the view from the top (below)
photos:                       # optional: photos from the top, pinned on the panorama at their bearing
  - { src: /assets/img/cumbres/provincia-este.jpg, azimuth: 80, caption: "El Plomo" }
```

Every summit gets a page at `/ascents/summits/<file name>/` with all the outings to it and, with `panorama: true`, the view from the top: what's in sight (named summits, the highest and farthest ground, glaciers and how much ice shows, towns, lakes, the sea, and which of my other summits can be seen), the 360° drawing and 3D view, and the viewshed map with the glacier ice. Ascent posts show the same view in their **Summit panorama** tab, loaded from the summit page when the tab opens; posts are split in tabs (**Story**, **Route**, **Summit panorama**) when they have more than the story.

`npm run panoramas` computes the views (`scripts/panoramas.mjs`, on the same engine as the hill's panorama, `scripts/lib/viewshed.mjs`): summits without one yet, or `npm run panoramas -- provincia` / `-- --all`. About 30 s per summit once the elevation tiles are cached; it writes `src/data/panoramas/<id>.json` and `public/assets/panoramas/<id>/`. Summits, glaciers, towns and lakes come from OpenStreetMap through Overpass, fetched once per region and cached in `scripts/data/osm/` (delete a region's files to refresh them). The elevation reader only covers the southern and western hemispheres for now, so a summit in the Alps can be registered but gets no panorama.

The Mountaineering section has four tabs: **Log** (totals, map, my summits to scale, a month-by-year calendar of outings, and every outing grouped by year with search and filters), **Summits** (a card per summit with the horizon from its top, sortable and searchable), **Routes** (every GPS route in one sortable table with elevation profiles at a shared scale) and the route analyzer.

### 3. Blog
Located in `src/content/blog/`.
Standard blog posts.

**Frontmatter format:**
```yaml
---
title: "Blog Post Title"
description: "Blog description"
pubDate: "2024-01-01"
heroImage: "/assets/img/headers/blog-header.jpg"
tags: ["personal", "update"]
---
```

## Photos

### Where the files live

Hero images and small illustrations live in the repo:

- Headers/hero images in `public/assets/img/headers/`
- Post illustrations in `public/assets/img/posts/<post-name>/`

Referenced from frontmatter or Markdown with an absolute path from the public root, e.g. `/assets/img/headers/image.jpg`.

**Photo-heavy posts should not go in git.** A trip report with 30 photos is ~100 MB, and every version of every photo stays in the repository history forever, which git clones (and GitHub Pages deploys) in full. The fix is to keep the photos in object storage with a CDN in front and reference them by URL. Posts never hardcode that URL: they use a path relative to the photo library, and a single setting says where the library is.

```yaml
# .env (local), or a repository variable in GitHub Actions
PUBLIC_MEDIA_BASE_URL=https://photos.example.com
```

```markdown
![Summit ridge](la-cruz/ridge.jpg)      ->  https://photos.example.com/la-cruz/ridge.jpg
```

Unset, it defaults to `/assets/img/posts`, so photos can be developed locally and moved later without touching a single post. Resolution happens in `src/lib/media.mjs`. These paths are left exactly as written: absolute paths (`/assets/...`), full URLs, and `./photo.jpg` (relative to the Markdown file, which Astro resolves and optimizes itself).

Good options for the library, roughly in order of how well they fit this site:

| Option | Notes |
| --- | --- |
| **Cloudflare R2** + custom domain | S3-compatible, free egress, ~$0.015/GB/month. Upload with `rclone` or `aws s3`. Best default. |
| **Cloudinary** / **imgix** (free tier) | Same thing plus resizing and WebP/AVIF conversion from the URL — no need to downscale by hand. |
| **Backblaze B2** + Cloudflare | Cheapest storage; free egress through Cloudflare. |
| **A separate git repo** on a CDN (jsDelivr) | Free, no account; still a git repo, so history still grows. Fine up to a few hundred MB. |

Google Drive and Dropbox are *not* suitable: their share links are not stable image URLs, they rate-limit hotlinking and they block hosting a site's assets.

Whatever the host, downscale before uploading — a 6000 px phone photo is wasted on a 750 px column. ~1600 px wide at JPEG quality ~78 is a good compromise:

```bash
# needs imagemagick; sharp (already a dependency) works too
mogrify -path out/ -resize 1600x1600\> -quality 78 *.jpg
```

### Photos in a post

Three layouts, all plain Markdown — no imports, and they work the same in `.md` and `.mdx`.

**A single photo.** A normal Markdown image. The quoted title, if present, becomes the caption:

```markdown
![Alt text](la-cruz/ridge.jpg "The last 200 m of the ridge")
```

**A grid.** `cols` is the number of columns (1-4, default 3) and `ratio` the shape of each cell (default `4/3`; use `ratio=auto` to keep each photo's own proportions). `caption` is optional and describes the whole group:

```markdown
:::gallery{cols=2 ratio=1/1 caption="The approach"}
![Alt](la-cruz/about.jpg)
![Alt](la-cruz/cayutue.jpg "A caption for this one")
![Alt](la-cruz/profile.jpg)
![Alt](la-cruz/summit.jpg)
:::
```

On narrow screens 3- and 4-column grids fall back to 2 columns.

**A carousel** — one photo at a time, swiped on a phone, dragged or stepped through with the arrows and dots on a desktop:

```markdown
:::carousel{ratio=3/2}
![Alt](la-cruz/cayutue.jpg "1. Leaving the hut, 6:10 AM")
![Alt](la-cruz/about.jpg "2. The endless scree")
:::
```

Clicking any photo in a grid or carousel opens it full-screen, with the arrow keys and Escape doing what you would expect. Several loose images in a single paragraph (no blank line between them) also become a grid.

#### Photos on the route map

In a post with a `route`, photos whose location is known appear on the map as round thumbnails (with a **Fotos** button to hide them; the choice is remembered), as camera ticks on the profile, and get a small "show on map" button in the post. Clicking a thumbnail opens the photo and a link back to it in the post.

The location comes from, in this order:

1. **What you write after a `#` in the photo's path** (it wins over everything else):

   ```markdown
   ![Cumbre](la-cruz/summit.jpg#km=5.8)                 5.8 km along the route
   ![Cumbre](la-cruz/summit.jpg#time=12:41)             on the route at that time of day (post timezone)
   ![Cumbre](la-cruz/summit.jpg#time=2026-06-13T12:41)
   ![Cumbre](la-cruz/summit.jpg#-33.7571,-70.1047)      coordinates (off the route is fine)
   ![Cumbre](la-cruz/summit.jpg#off)                    never on the map
   ```

2. **The photo's EXIF GPS position**, or 3. **its EXIF capture time** matched against the route's timestamps, which places photos from cameras without GPS (keep the camera clock right).

Resizing and most CDNs strip EXIF, so the EXIF data is read from the originals and saved in `src/data/photo-locations.json` (commit it). That happens automatically before `npm run dev` and `npm run build` for the photos in `public/assets/img/posts/`; to read originals kept elsewhere, laid out like the posts reference them, run:

```sh
npm run photos -- ~/Pictures/web-originals
```

Saved entries are kept when the files are moved away, so you can run it once, then resize and upload the photos anywhere.

The syntax is handled by `src/plugins/remark-media.mjs` (markup), `src/styles/media.css` (styling) and `src/scripts/media.ts` (sliding and the lightbox). `src/content/ascents/la-cruz.md` uses all three layouts as a live example.

## Site Configuration

Global site configuration is managed in three places:

1. **`src/i18n/ui.ts`**: Site title and description (used for Open Graph tags and SEO) and all UI text, per language.
2. **`src/config.ts`**: View transitions and the light/dark theme names.
3. **`astro.config.mjs`**: The `site` URL and integrations (Tailwind, MDX, sitemap, math rendering).

## UI & Theming

- The sidebar, header, and footer components are located in `src/components/`. 
- To edit links in the sidebar, modify `src/components/SideBarMenu.astro` (labels are in `src/i18n/ui.ts`).
- The sun/moon button next to the language flags switches between a dark and a light theme; the choice is remembered in the browser. The themes are DaisyUI themes set in `src/config.ts` (`DARK_THEME = 'dim'`, `LIGHT_THEME = 'light'`). To use different ones, change them there and in the `themes` list in `tailwind.config.cjs`.

## Customizing the Favicon

The favicon is located at `public/favicon.svg`. To change it, simply replace this file with a new SVG image. You can use icons from collections like [Lucide](https://lucide.dev/) or [Heroicons](https://heroicons.com/).

## Tech Stack
- [Astro](https://astro.build/) - Web framework
- [TailwindCSS](https://tailwindcss.com/) - Utility-first CSS
- [DaisyUI](https://daisyui.com/) - Tailwind CSS components
- [Remark Math](https://github.com/remarkjs/remark-math) / [Rehype Katex](https://github.com/remarkjs/remark-math/tree/main/packages/rehype-katex) - LaTeX rendering support
