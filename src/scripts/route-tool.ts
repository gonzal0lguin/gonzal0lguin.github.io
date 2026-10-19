// Browser side of <RouteTool>: reads the visitor's GPX files, stitches them in chronological
// order, analyzes the result with the same code as the posts and renders the same route card.

import { analyzeTrack, parseGpx, stitchTracks, toGpx, type GpxTrack, type Waypoint } from "../lib/gpx-core";
import { renderRouteCard } from "../lib/route-card";
import { formatClock, formatDay } from "../lib/route-format";
import { getLang, languages, useTranslations } from "../i18n/ui";
import { destroyRouteMaps, initRouteMaps } from "./route-map";

interface Part {
  id: number;
  file: File;
  track: GpxTrack;
}

let teardown: (() => void) | null = null;
document.addEventListener("astro:before-swap", () => {
  teardown?.();
  teardown = null;
});

export function initRouteTool() {
  const root = document.querySelector<HTMLElement>("[data-route-tool]:not([data-ready])");
  if (!root) return;
  root.dataset.ready = "";

  const lang = getLang(root.dataset.lang);
  const t = useTranslations(lang);
  const locale = languages[lang].locale;
  // Clock times in the visitor's own timezone.
  const timezone = Intl.DateTimeFormat().resolvedOptions().timeZone;
  const $ = <T extends HTMLElement = HTMLElement>(selector: string) => root.querySelector<T>(selector)!;

  const input = $<HTMLInputElement>("[data-input]");
  const dropzone = $("[data-dropzone]");
  const errorBox = $("[data-error]");
  const filesBox = $("[data-files]");
  const list = $("[data-list]");
  const result = $("[data-result]");

  let parts: Part[] = [];
  let nextId = 0;
  let downloadUrl: string | null = null;

  const revokeDownload = () => {
    if (downloadUrl) URL.revokeObjectURL(downloadUrl);
    downloadUrl = null;
  };
  teardown = revokeDownload;

  async function addFiles(files: Iterable<File>) {
    const errors: string[] = [];
    for (const file of files) {
      try {
        const track = parseGpx(await file.text());
        if (track.points.length === 0) throw new Error("empty");
        parts.push({ id: nextId++, file, track });
      } catch {
        errors.push(t("tools.route.errorFile", { name: file.name }));
      }
    }
    const added = render(errors);
    if (added) result.scrollIntoView({ behavior: "smooth", block: "start" });
  }

  /** Redraw the file list and the route card; returns whether a route is shown. */
  function render(errors: string[] = []) {
    const timed = parts.every((p) => p.track.points.every((pt) => pt.time != null));
    // The same order stitchTracks() uses, so the list matches the route.
    if (timed) parts.sort((a, b) => (a.track.points[0].time as number) - (b.track.points[0].time as number));
    renderList(timed);

    destroyRouteMaps(result);
    result.innerHTML = "";
    revokeDownload();
    let shown = false;
    if (parts.length) {
      try {
        // A copy: with a single file, stitchTracks returns that file's own track.
        const track = { ...stitchTracks(parts.map((p) => p.track)) };
        const key = trackKey(track);
        // Points marked earlier on this same route come back (kept in this browser only).
        const saved = loadMarks(key);
        if (saved) track.waypoints = saved;
        const route = analyzeTrack(track);
        const single = parts.length === 1;
        const fileTitle = parts[0].file.name.replace(/\.gpx$/i, "");
        const title = route.name || fileTitle || t("tools.route.defaultName");
        const downloadName = single ? parts[0].file.name : `${slugify(title)}.gpx`;
        // One untouched file is offered back as is; otherwise a GPX with the stitched track and the marks.
        const setDownload = (blob: Blob) => {
          revokeDownload();
          downloadUrl = URL.createObjectURL(blob);
          return downloadUrl;
        };
        const generated = () => new Blob([toGpx(track)], { type: "application/gpx+xml" });
        result.innerHTML = renderRouteCard(route, {
          lang,
          title,
          timezone,
          download: setDownload(single && !saved ? parts[0].file : generated()),
          downloadName,
          editable: true,
        });
        result.querySelector("[data-route-map]")?.addEventListener("route:waypoints", (event) => {
          track.waypoints = (event as CustomEvent<Waypoint[]>).detail;
          saveMarks(key, track.waypoints);
          const link = result.querySelector<HTMLAnchorElement>("a[download]");
          if (link) link.href = setDownload(generated());
        });
        initRouteMaps();
        shown = true;
      } catch {
        errors.push(t("tools.route.errorRoute"));
      }
    }
    errorBox.textContent = errors.join("\n");
    errorBox.hidden = errors.length === 0;
    return shown;
  }

  function renderList(timed: boolean) {
    filesBox.hidden = parts.length === 0;
    $("[data-count]").textContent = String(parts.length);
    $("[data-order]").textContent = parts.length > 1 ? t(timed ? "tools.route.ordered" : "tools.route.unordered") : "";
    list.replaceChildren(
      ...parts.map((part, i) => {
        const li = document.createElement("li");
        li.className = "flex items-center gap-3 rounded-xl bg-base-100/60 px-3 py-2";
        li.innerHTML = `
          <span class="grid h-7 w-7 shrink-0 place-items-center rounded-full bg-primary/15 text-xs font-bold text-primary">${i + 1}</span>
          <span class="min-w-0 flex-1">
            <span class="block truncate text-sm font-medium" data-name></span>
            <span class="block text-xs tabular-nums text-base-content/60" data-meta></span>
          </span>
          <button type="button" class="btn btn-circle btn-ghost btn-sm" data-remove>
            <svg class="h-4 w-4" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="M6 6l12 12M18 6 6 18"></path></svg>
          </button>`;
        li.querySelector("[data-name]")!.textContent = part.file.name;
        li.querySelector("[data-meta]")!.textContent = [timeRange(part.track), t("tools.route.points", { n: part.track.points.length.toLocaleString(locale) })].join(" · ");
        const remove = li.querySelector<HTMLButtonElement>("[data-remove]")!;
        remove.setAttribute("aria-label", t("tools.route.remove", { name: part.file.name }));
        remove.title = remove.getAttribute("aria-label")!;
        remove.addEventListener("click", () => {
          parts = parts.filter((p) => p.id !== part.id);
          render();
        });
        return li;
      }),
    );
  }

  function timeRange({ points }: GpxTrack) {
    const start = points[0].time;
    const end = points[points.length - 1].time;
    if (start == null || end == null) return t("tools.route.noTime");
    const startDay = formatDay(start, locale, timezone);
    const endDay = formatDay(end, locale, timezone);
    return `${startDay} ${formatClock(start, locale, timezone)} → ${endDay === startDay ? "" : `${endDay} `}${formatClock(end, locale, timezone)}`;
  }

  // --- Input: file picker and drag & drop

  input.addEventListener("change", () => {
    const files = [...(input.files ?? [])];
    input.value = "";
    addFiles(files);
  });
  $("[data-add]").addEventListener("click", () => input.click());
  $("[data-clear]").addEventListener("click", () => {
    parts = [];
    render();
  });

  for (const type of ["dragenter", "dragover"]) {
    dropzone.addEventListener(type, (event) => {
      event.preventDefault();
      dropzone.dataset.dragging = "";
    });
  }
  dropzone.addEventListener("dragleave", (event) => {
    if (!dropzone.contains(event.relatedTarget as Node | null)) delete dropzone.dataset.dragging;
  });
  dropzone.addEventListener("drop", (event) => {
    event.preventDefault();
    delete dropzone.dataset.dragging;
    addFiles([...(event.dataTransfer?.files ?? [])]);
  });
}

const MARKS_PREFIX = "route-tool:marks:";

/** Identifies a route by its points, so its marks can be restored when the same files come back. */
function trackKey({ points }: GpxTrack) {
  const first = points[0];
  const last = points[points.length - 1];
  return [points.length, first.lat, first.lon, first.time, last.lat, last.lon, last.time].join("|");
}

function loadMarks(key: string): Waypoint[] | null {
  try {
    const saved = localStorage.getItem(MARKS_PREFIX + key);
    return saved ? JSON.parse(saved) : null;
  } catch {
    return null;
  }
}

function saveMarks(key: string, waypoints: Waypoint[]) {
  try {
    localStorage.setItem(MARKS_PREFIX + key, JSON.stringify(waypoints));
  } catch {
    // Private mode or storage full: the marks still go into the downloaded GPX.
  }
}

function slugify(text: string) {
  const slug = text
    .normalize("NFD")
    .replace(/[̀-ͯ]/g, "")
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");
  return slug || "route";
}
