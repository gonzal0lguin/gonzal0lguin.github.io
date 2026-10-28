# Working on this repository

Astro 4 + Tailwind + DaisyUI. Personal portfolio and a mountaineering log, bilingual
(Spanish is the source and lives at the root, English under `/en/`). `README.md` explains the
content model — collections, frontmatter, GPX routes, summits — and `editing.md` the in-browser
editor at `/edit`. This file is about *working* here: the commands, the environment's traps, and
the things that cost an afternoon to find out once.

## Commands

| | |
| --- | --- |
| `npm run dev` | dev server on :4321, plus the `/edit` editor (dev only, never built) |
| `npm run build` | static build into `dist/` |
| `npm run shot -- /en/ascents` | screenshot built pages — see below |
| `npm run translate` | Spanish → English with Claude, needs `ANTHROPIC_API_KEY` |
| `npm run photos` | EXIF of post photos → `src/data/photo-locations.json` (runs on pre-dev/pre-build) |
| `npm run panoramas` | summit views; `-- provincia` for one, `-- --all` for all (~30 s each) |
| `npm run profiles` | every summit's outline for the to-scale chart → `src/data/profiles.json` (seconds) |
| `npm run terrain` / `climate` / `inventory` | data for the reforestation post |

Stick to **npm** — the GitHub Actions deploy uses `npm ci`.

## Looking at a page (screenshots)

`npm run build && npm run shot -- / /en/ascents` writes PNGs to `tmp/shots/` (gitignored).
`--full` for the whole page, `--width/--height`, `--out <dir>`. It starts and stops its own
`astro preview` on a free port. **It needs node ≥ 20**, which is not the node in PATH:

```bash
PATH=~/.local/share/node24/bin:$PATH npm run shot -- /en/ascents
```

This works now, but only because of a pile of environment traps. Do not "simplify" it back:

- **The system Firefox is a snap and cannot be used from an agent/CI shell.** A snap has to
  create a mount namespace to start, and `unshare` is blocked in the sandbox here, so
  `firefox --version` just **hangs with no output and exits 0** — no error to go on. A snap also
  gets a *private* `/tmp`, so even when it does run, `--screenshot /tmp/x.png` writes where
  nothing else can see it. `/snap/bin/geckodriver` is a symlink into the same snap and fails
  identically — that one is easy to miss, because it is on PATH and looks like a real tool.
- **Ubuntu's node is 18.** Playwright needs ≥ 20, the Firefox MCP needs ≥ 22. Node 24 is
  installed side by side at `~/.local/share/node24` (symlink to `node-v24.22.1-linux-x64`).
  The system node is untouched on purpose: the site itself builds fine on 18.
- **`XDG_CACHE_HOME` is redirected** to `~/snap/alacritty/common/.cache` when the terminal is the
  alacritty snap. Playwright installs and then looks up its browsers there, so an install made in
  one terminal is "missing" from the next. `scripts/shot.mjs` pins `PLAYWRIGHT_BROWSERS_PATH` to
  `~/.cache/ms-playwright` *before* importing playwright, which is when it reads the variable.
- **Firefox's content sandbox** also wants a user namespace, so the script sets
  `MOZ_DISABLE_CONTENT_SANDBOX=1`.

A map/chart page needs a moment after `load` before it is worth shooting; the script waits 1.5 s,
otherwise a half-drawn Leaflet map looks like a rendering bug.

### The other two routes

**Stock Firefox directly**, no node and no dependencies, when all that is wanted is one picture:

```bash
~/.local/share/firefox/firefox --headless --screenshot /abs/path/out.png http://127.0.0.1:4321/
```

That is Mozilla's tarball build at `~/.local/share/firefox` (159.0), installed *because* the snap
is unusable. It works even with a stripped environment, and it can write anywhere.

**The `firefox-devtools` MCP**, for anything interactive — clicking, console errors, network,
DOM snapshots. It was failing with `CONNECTION_CLOSED` purely because it was launched through
`npx` with node 18 and died on startup (`Node v18.19.1 is not supported`). `.mcp.json` now pins
all three moving parts: node 24, the server installed at `~/.local/share/firefox-devtools-mcp`
(not the `~/.npm/_npx` cache, which gets pruned), the stock Firefox via `--firefox-path`, and a
PATH with `~/.local/bin` ahead of `/snap/bin` so the **real** geckodriver
(`~/.local/bin/geckodriver`, 0.37.1) wins over the snap one. Set `FIREFOX_HEADLESS` to `0` there
to watch it work.

Note Playwright's Firefox and the MCP are *not* interchangeable: Playwright's build is patched for
its own protocol and geckodriver cannot drive it ("Timed out waiting for the WebDriver server").

### Last resort: rasterising SVG

For a self-contained inline `<svg>` (the panoramas, the charts) there is no need for a browser at
all: pull it out of the built HTML and render it with the project's own `sharp`. Substitute CSS
variables and `color-mix()` with literal colours first — librsvg supports neither — inline only
`src/styles/inventory.css` + `summits.css` (the whole Tailwind bundle renders solid black), wrap
the CSS in `<![CDATA[ ]]>`, and narrow the `viewBox` to crop a slice of the 19200-wide panorama.

## Data sources, and how they misbehave

- **Overpass (OpenStreetMap)** returns **406 without a real User-Agent** and is often at capacity
  (502/504). Always go through `scripts/lib/overpass.mjs`: GET, a real UA, backoff across two
  servers, and an on-disk cache in `scripts/data/osm/`. Delete a region's file to refresh it.
- **The Copernicus DEM reader** (`scripts/lib/dem.mjs`) reads any hemisphere, but only 1° × 1°
  tiles of 3600 px, so nothing past ±50° latitude (Copernicus narrows its tiles there). The
  panorama pipeline is only proven south and west of 0°, and `scripts/panoramas.mjs` still
  skips the rest: a summit in the Alps gets its outline (`npm run profiles`) but no panorama.
- Elevation tiles and OSM answers are cached; the first `npm run panoramas` is slow, the rest fast.

## Conventions

- Scripts in `scripts/` open with a **comment block saying what the script is for, what it writes
  and why it works that way** — the reasoning, not a restatement of the code. Match that voice;
  it is the repository's documentation, and the inline comments explain *why*, never *what*.
- Prefer hand-written parsing over a dependency (the GPX, KML and xlsx readers are all hand-rolled
  on node builtins). Keep it that way unless there is a real reason not to.
- Scripts are ESM `.mjs` on node builtins; site code is TypeScript under `src/`.
- Data a script computes goes to `src/data/` as JSON and is committed; heavy assets to `public/`.
- Content is written in Spanish; the English twin lives in an `en/` subfolder with the same
  filename and carries a `translationHash`. Do not hand-edit that hash — `npm run translate
  -- --stamp` sets it, and deleting the line pins a hand-written translation for good.
- `tmp/` is scratch and gitignored.

## Installed outside the repo

Nothing here needs root, and none of it is in `package.json` except `playwright`:

| Path | What |
| --- | --- |
| `~/.local/share/node24` | node 24.22.1, side by side with the system node 18 |
| `~/.local/share/firefox` | stock Firefox 159 (tarball), replaces the unusable snap |
| `~/.local/bin/geckodriver` | geckodriver 0.37.1, must precede `/snap/bin` on PATH |
| `~/.local/share/firefox-devtools-mcp` | the MCP server, pinned out of the npx cache |
| `~/.cache/ms-playwright` | Playwright's Firefox build, used only by `npm run shot` |
