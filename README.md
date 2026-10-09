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

Add a `route:` field pointing to a `.gpx` file in `public/` (e.g. export it from your watch, Strava or Garmin and save it in `public/assets/tracks/`). The entry then shows an interactive route card above the text. Without a `route`, nothing is shown.

- Map with topographic, satellite and street layers. The line can be colored by **grade**, **elevation** or **heart rate** (when the file has it), and shows the start, the summit, camps and km markers.
- Elevation profile linked to the map: hover either one to see distance, elevation, grade, time of day and heart rate at that point. ▶ replays the route.
- Stats: distance, elevation gain/loss, highest/lowest point, moving and total time, heart rate, steepest grade, and a hiking-time estimate using DIN 33466 (the German Alpine Club's formula).
- Multi-day trips: any pause longer than 3 hours starts a new day, and the place where it happened is marked as a camp, with a day-by-day breakdown.
- A "Download GPX" button.

The GPX file is analyzed when the site is built (`src/lib/gpx.ts`), so the browser only downloads a compact summary; the map code is only loaded on pages that have a route. Distance and elevation gain on the Mountaineering page are the totals of all entries with a route.

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

## Adding Images

Store your images in the `public/assets/img/` directory.
- Headers/Hero images generally go in `public/assets/img/headers/`
- Post-specific images can go in `public/assets/img/posts/<post-name>/`

When referencing an image in your markdown or frontmatter, use the absolute path starting from the public root, e.g., `/assets/img/headers/image.jpg`.

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
