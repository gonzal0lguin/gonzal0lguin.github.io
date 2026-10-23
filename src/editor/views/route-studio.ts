// The GPX studio: the same route card the post shows, but editable, plus the tools to fix a
// track before it goes into the repository.
//
// It reuses the site's own code (src/lib/gpx-core.ts, src/lib/route-card.ts,
// src/scripts/route-map.ts), so what you see here is exactly what the page will render.
import * as api from "../api";
import { type GpxTrack, type RawPoint, type Waypoint, analyzeTrack, parseGpx, stitchTracks, toGpx } from "../../lib/gpx-core";
import { placePhotos, type PostPhoto } from "../../lib/photos";
import { renderRouteCard } from "../../lib/route-card";
import { destroyRouteMaps, initRouteMaps } from "../../scripts/route-map";
import { button, confirmDialog, el, fail, field, fill, formatBytes, icon, ICONS, promptDialog, toast } from "../dom";
import { photoUrl } from "../media-url";
import { gpxPicker } from "./frontmatter-view";
import type { EntryCtx } from "./entry";

interface Part {
  publicPath: string;
  track: GpxTrack;
}

export async function routeStudio(entry: EntryCtx): Promise<Node> {
  const files = entry.fm.list("route");
  const root = el("div", { class: "ed-pane" });
  const tools = el("div", { class: "ed-card ed-card--tools" });
  const card = el("div", { class: "ed-route" });
  const info = el("div", { class: "ed-note" });

  let parts: Part[] = [];
  let track: GpxTrack | null = null;
  /** Keys of the waypoints that came from the files, to tell them from the entry's own. */
  let fileKeys = new Set<string>();

  const key = (w: { lat: number; lon: number; name: string }) => `${w.lat.toFixed(5)}|${w.lon.toFixed(5)}|${w.name}`;

  async function load() {
    parts = [];
    const errors: string[] = [];
    for (const publicPath of files) {
      try {
        const { xml } = await api.get<{ xml: string }>("/gpx", { path: publicPath });
        const parsed = parseGpx(xml);
        if (parsed.points.length === 0) throw new Error("sin puntos");
        parts.push({ publicPath, track: parsed });
      } catch (error) {
        errors.push(`${publicPath}: ${error instanceof Error ? error.message : error}`);
      }
    }
    if (errors.length) toast(errors.join("\n"), "error");
    rebuild();
  }

  /** Back to the files as they are on disk, plus the points written in the entry. */
  function rebuild() {
    if (parts.length === 0) {
      track = null;
      draw();
      return;
    }
    const stitched = { ...stitchTracks(parts.map((part) => part.track)) };
    fileKeys = new Set(stitched.waypoints.map(key));
    const extra: Waypoint[] = entry.fm.maps("waypoints").map((point) => ({
      name: String(point.name ?? ""),
      lat: Number(point.lat),
      lon: Number(point.lon),
      kind: (point.kind ?? "note") as Waypoint["kind"],
      ele: point.ele != null ? Number(point.ele) : null,
      desc: point.note != null ? String(point.note) : undefined,
    }));
    track = { ...stitched, waypoints: [...stitched.waypoints, ...extra] };
    draw();
  }

  /** The entry's `waypoints` are the marks that are not in the GPX files. */
  function saveWaypointsToEntry() {
    if (!track) return;
    const extra = track.waypoints.filter((w) => !fileKeys.has(key(w)));
    entry.fm.set(
      "waypoints",
      extra.map((w) => ({
        name: w.name,
        kind: w.kind,
        lat: w.lat,
        lon: w.lon,
        ...(w.ele != null ? { ele: Math.round(w.ele) } : {}),
        ...(w.desc ? { note: w.desc } : {}),
      })) as never,
    );
    entry.markDirty();
  }

  function draw() {
    destroyRouteMaps(card);
    fill(card);
    drawTools();
    if (!track) {
      card.appendChild(el("p", { class: "ed-empty" }, files.length ? "No se pudo leer ningún archivo de la ruta." : "Esta entrada no tiene ruta. Añade un .gpx arriba."));
      return;
    }
    let route;
    try {
      route = analyzeTrack(track);
    } catch (error) {
      card.appendChild(el("p", { class: "ed-empty" }, `No se pudo analizar la ruta: ${error instanceof Error ? error.message : error}`));
      return;
    }

    const timezone = entry.fm.string("timezone") || "America/Santiago";
    const gpx = () => new Blob([toGpx(track!)], { type: "application/gpx+xml" });
    const download = URL.createObjectURL(gpx());
    card.innerHTML = renderRouteCard(route, {
      lang: entry.lang,
      title: entry.fm.string("mountain") || entry.fm.string("title"),
      timezone,
      download,
      downloadName: `${entry.key}.gpx`,
      photos: placePhotos(entryPhotos(entry), route, timezone),
      editable: true,
    });
    card.querySelector("[data-route-map]")?.addEventListener("route:waypoints", (event) => {
      track!.waypoints = (event as CustomEvent<Waypoint[]>).detail;
      saveWaypointsToEntry();
      drawTools();
    });
    initRouteMaps();

    const stats = route.stats;
    fill(info,
      el("span", {}, `${track.points.length.toLocaleString()} puntos`),
      el("span", {}, `${(stats.distance / 1000).toFixed(2)} km`),
      stats.gain != null ? el("span", {}, `+${Math.round(stats.gain)} m`) : null,
      el("span", {}, `${track.waypoints.length} marcas (${track.waypoints.filter((w) => !fileKeys.has(key(w))).length} en la entrada)`),
      el("span", {}, `${track.points.some((p) => p.hr != null) ? "con" : "sin"} pulso`),
      el("span", {}, `${track.points.some((p) => p.time != null) ? "con" : "sin"} tiempos`),
    );
  }

  function drawTools() {
    const changedFileMarks = track ? [...fileKeys].some((k) => !track!.waypoints.some((w) => key(w) === k)) : false;

    fill(tools,
      el(
        "header",
        { class: "ed-card__head" },
        el("h2", {}, "Archivos de la ruta", el("span", { class: "ed-chip" }, String(parts.length))),
        button("Recargar del disco", () => void load(), { variant: "ghost", iconPath: ICONS.refresh }),
      ),
      el(
        "ul",
        { class: "ed-list ed-list--tight" },
        ...parts.map((part, i) =>
          el(
            "li",
            { class: "ed-list__row" },
            el("span", { class: "ed-chip" }, String(i + 1)),
            el("code", { class: "ed-grow" }, part.publicPath),
            el("span", { class: "ed-list__meta" }, `${part.track.points.length.toLocaleString()} pts`),
            button("", () => {
                files.splice(files.indexOf(part.publicPath), 1);
                entry.fm.set("route", [...files]);
                entry.markDirty();
                void load();
              }, { variant: "ghost", iconPath: ICONS.trash, title: "Quitar de la entrada" }),
          ),
        ),
      ),
      gpxPicker(entry, (publicPath) => {
        if (files.includes(publicPath)) return;
        files.push(publicPath);
        entry.fm.set("route", [...files]);
        entry.markDirty();
        void load();
      }),
      info,
      track ? trackTools(entry, track, { rebuild, redraw: draw, parts }) : null,
      changedFileMarks
        ? el(
            "p",
            { class: "ed-warn" },
            "Borraste o moviste una marca que viene del archivo .gpx. La entrada sólo puede añadir marcas, no quitarlas: usa “Escribir el GPX” para dejar el archivo como lo ves aquí.",
          )
        : null,
      el(
        "div",
        { class: "ed-row" },
        button("Guardar marcas en la entrada", () => {
            saveWaypointsToEntry();
            toast("Las marcas nuevas van al frontmatter; guarda la entrada con ⌘/Ctrl+S", "ok");
          }, { variant: "primary", iconPath: ICONS.save, title: "Escribe `waypoints:` en el frontmatter de la entrada" }),
        track ? button("Escribir el GPX…", () => void writeGpx(entry, track!, files, load), { iconPath: ICONS.map, title: "Guarda el track (unido y con las marcas) como un .gpx del repositorio" }) : null,
      ),
    );
  }

  root.append(
    el("p", { class: "ed-note" }, "El mapa es el mismo de la página, en modo edición: con “Marcar” activo, pincha la ruta para añadir un punto, y pincha un punto para editarlo o borrarlo."),
    tools,
    card,
  );
  await load();
  return root;
}

/** The photos of the post, in the shape src/lib/photos.ts expects. */
function entryPhotos(entry: EntryCtx): PostPhoto[] {
  const photos: PostPhoto[] = [];
  for (const block of entry.blocks) {
    if (block.type !== "photos") continue;
    for (const photo of block.photos) {
      photos.push({ path: photo.path, src: photoUrl(photo.path), alt: photo.alt, caption: photo.caption, geo: photo.geo || null });
    }
  }
  return photos;
}

// --- Editing the track itself

function trackTools(entry: EntryCtx, track: GpxTrack, ctx: { rebuild: () => void; redraw: () => void; parts: Part[] }) {
  const total = track.points.length;
  const apply = (next: RawPoint[], what: string) => {
    if (next.length < 2) {
      toast("Quedarían menos de 2 puntos", "error");
      return;
    }
    track.points = next;
    toast(`${what}: ${next.length.toLocaleString()} puntos`, "ok");
    ctx.redraw();
  };

  const from = el("input", { class: "ed-input ed-input--sm", type: "number", min: 0, max: total - 2, value: 0 });
  const to = el("input", { class: "ed-input ed-input--sm", type: "number", min: 1, max: total - 1, value: total - 1 });
  const minutes = el("input", { class: "ed-input ed-input--sm", type: "number", step: 1, value: 0 });
  const spacing = el("input", { class: "ed-input ed-input--sm", type: "number", min: 1, step: 1, value: 5 });

  return el(
    "details",
    { class: "ed-details" },
    el("summary", {}, "Herramientas del track"),
    el(
      "div",
      { class: "ed-stack ed-stack--tight" },
      el("p", { class: "ed-note" }, "Cambian el track en memoria. Nada toca el disco hasta que uses “Escribir el GPX…”; los archivos originales quedan intactos y se puede volver con “Recargar del disco”."),
      el(
        "div",
        { class: "ed-row" },
        field("Recortar desde el punto", from),
        field("hasta", to),
        button("Recortar", () => apply(track.points.slice(Math.max(0, Number(from.value)), Number(to.value) + 1), "Recortado"), {}),
      ),
      el(
        "div",
        { class: "ed-row" },
        field("Desplazar los tiempos (min)", minutes),
        button("Aplicar", () => {
            const delta = Number(minutes.value) * 60;
            apply(track.points.map((point) => ({ ...point, time: point.time == null ? null : point.time + delta })), "Tiempos desplazados");
          }, {}),
      ),
      el(
        "div",
        { class: "ed-row" },
        field("Separación mínima (m)", spacing),
        button("Adelgazar", () => apply(thin(track.points, Math.max(1, Number(spacing.value))), "Adelgazado"), { title: "Quita puntos muy juntos para que el archivo pese menos" }),
      ),
      el(
        "div",
        { class: "ed-row" },
        button("Quitar el pulso", () => apply(track.points.map((point) => ({ ...point, hr: null })), "Pulso quitado"), {}),
        button("Quitar los tiempos", () => apply(track.points.map((point) => ({ ...point, time: null })), "Tiempos quitados"), { title: "El track pasa a no tener horas ni días" }),
        button("Deshacer todo", () => ctx.rebuild(), { variant: "ghost", iconPath: ICONS.refresh }),
      ),
    ),
  );
}

/** Save the current track as a .gpx in public/, and point the entry at it. */
async function writeGpx(entry: EntryCtx, track: GpxTrack, files: string[], reload: () => Promise<void>) {
  const suggestion = files.length === 1 ? files[0] : `/assets/lib/${entry.key}.gpx`;
  const publicPath = await promptDialog({
    title: "Escribir el GPX",
    label: "Ruta pública del archivo",
    value: suggestion,
    hint: "Se guarda en public/… El archivo anterior se copia a .editor-local/backups antes de sobrescribirlo.",
    confirm: "Escribir",
  });
  if (!publicPath) return;
  const replacing = files.includes(publicPath);
  if (
    !(await confirmDialog({
      title: replacing ? `¿Sobrescribir ${publicPath}?` : `¿Escribir ${publicPath}?`,
      message: `El track unido (${track.points.length.toLocaleString()} puntos, ${track.waypoints.length} marcas) se escribe en public${publicPath}. La entrada quedará apuntando sólo a ese archivo.`,
      confirm: "Escribir",
      danger: replacing,
    }))
  ) {
    return;
  }
  try {
    const result = await api.post<{ publicPath: string; size: number; backup: string | null }>("/gpx/save", { publicPath, xml: toGpx(track) });
    entry.fm.set("route", [result.publicPath]);
    // Marks now live in the file; the entry does not need to repeat them.
    entry.fm.remove("waypoints");
    entry.markDirty();
    files.splice(0, files.length, result.publicPath);
    toast(`Escrito ${result.publicPath} (${formatBytes(result.size)})${result.backup ? ` · copia: ${result.backup}` : ""}`, "ok");
    await entry.app.refresh();
    await reload();
  } catch (error) {
    fail("No se pudo escribir el GPX", error);
  }
}

/** Keep one point every `meters`, plus the first and the last. */
function thin(points: RawPoint[], meters: number): RawPoint[] {
  const kept: RawPoint[] = [points[0]];
  for (let i = 1; i < points.length - 1; i++) {
    if (distance(kept[kept.length - 1], points[i]) >= meters) kept.push(points[i]);
  }
  kept.push(points[points.length - 1]);
  return kept;
}

function distance(a: RawPoint, b: RawPoint) {
  const r = Math.PI / 180;
  const h = Math.sin(((b.lat - a.lat) * r) / 2) ** 2 + Math.cos(a.lat * r) * Math.cos(b.lat * r) * Math.sin(((b.lon - a.lon) * r) / 2) ** 2;
  return 2 * 6371008.8 * Math.asin(Math.sqrt(h));
}

