// The GPX files of the repository on their own: analyze one or several, fix them with the
// same tools as the entry studio, and write the result back.
import * as api from "../api";
import { type GpxTrack, type Waypoint, analyzeTrack, parseGpx, stitchTracks, toGpx } from "../../lib/gpx-core";
import { renderRouteCard } from "../../lib/route-card";
import { destroyRouteMaps, initRouteMaps } from "../../scripts/route-map";
import { actionsSlot, type Ctx } from "../app";
import { button, confirmDialog, el, fail, fill, formatBytes, icon, ICONS, promptDialog, toast } from "../dom";
import { entryRoute } from "./dashboard";

export async function routesView(ctx: Ctx): Promise<Node> {
  const files = ctx.state.gpx;
  ctx.heading("Rutas GPX", `${files.length} archivo(s) en public/`);

  const selected = new Set<string>();
  const card = el("div", { class: "ed-route" });
  let track: GpxTrack | null = null;

  const list = el("ul", { class: "ed-list ed-list--tight" });
  const draw = () => {
    fill(list,
      ...files.map((file) => {
        const check = el("input", { type: "checkbox", checked: selected.has(file.publicPath) }) as HTMLInputElement;
        check.addEventListener("change", () => {
          if (check.checked) selected.add(file.publicPath);
          else selected.delete(file.publicPath);
          void open();
        });
        return el(
          "li",
          { class: "ed-list__row" },
          check,
          el("code", { class: "ed-grow" }, file.publicPath),
          el("span", { class: "ed-list__meta" }, formatBytes(file.size)),
          ...file.usedBy.map((path) => el("a", { class: "ed-chip ed-chip--soft", href: `#/${entryRoute(path)}`, title: path }, path.split("/").pop()!)),
          file.usedBy.length === 0 ? el("span", { class: "ed-chip ed-chip--muted" }, "sin usar") : null,
        );
      }),
    );
  };

  async function open() {
    destroyRouteMaps(card);
    fill(card);
    track = null;
    if (selected.size === 0) {
      card.appendChild(el("p", { class: "ed-empty" }, "Marca uno o más archivos para analizarlos. Varios se unen en orden cronológico, como en una entrada."));
      return;
    }
    try {
      const parts = await Promise.all([...selected].map((path) => api.get<{ xml: string }>("/gpx", { path })));
      track = { ...stitchTracks(parts.map((part) => parseGpx(part.xml))) };
      const route = analyzeTrack(track);
      const download = URL.createObjectURL(new Blob([toGpx(track)], { type: "application/gpx+xml" }));
      card.innerHTML = renderRouteCard(route, {
        lang: "es",
        title: [...selected][0].split("/").pop()?.replace(/\.gpx$/i, ""),
        download,
        downloadName: "ruta.gpx",
        editable: true,
      });
      card.querySelector("[data-route-map]")?.addEventListener("route:waypoints", (event) => {
        if (track) track.waypoints = (event as CustomEvent<Waypoint[]>).detail;
      });
      initRouteMaps();
    } catch (error) {
      fail("No se pudo abrir la ruta", error);
      card.appendChild(el("p", { class: "ed-empty" }, String(error instanceof Error ? error.message : error)));
    }
  }

  fill(actionsSlot(),
    button("Subir .gpx", () => uploadGpx(ctx), { variant: "primary", iconPath: ICONS.plus }),
    button("Guardar lo marcado…", () => void save(), { iconPath: ICONS.save, title: "Escribe el track unido, con las marcas que hayas puesto, en un .gpx" }),
    button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }),
  );

  async function save() {
    if (!track) {
      toast("No hay ninguna ruta abierta", "error");
      return;
    }
    const single = selected.size === 1 ? [...selected][0] : "/assets/lib/ruta.gpx";
    const publicPath = await promptDialog({ title: "Guardar el GPX", label: "Ruta pública", value: single, hint: "El archivo anterior se copia a .editor-local/backups antes de sobrescribirlo.", confirm: "Guardar" });
    if (!publicPath) return;
    if (!(await confirmDialog({ title: `¿Escribir ${publicPath}?`, message: `${track.points.length.toLocaleString()} puntos y ${track.waypoints.length} marcas.`, confirm: "Escribir", danger: selected.has(publicPath) }))) return;
    try {
      const result = await api.post<{ publicPath: string; size: number }>("/gpx/save", { publicPath, xml: toGpx(track) });
      toast(`Escrito ${result.publicPath} (${formatBytes(result.size)})`, "ok");
      await ctx.refresh();
    } catch (error) {
      fail("No se pudo guardar", error);
    }
  }

  draw();
  await open();
  return el(
    "div",
    { class: "ed-stack" },
    el("section", { class: "ed-card" }, el("header", { class: "ed-card__head" }, el("h2", {}, "Archivos")), list),
    card,
  );
}

function uploadGpx(ctx: Ctx) {
  const input = el("input", { type: "file", accept: ".gpx", multiple: true }) as HTMLInputElement;
  input.addEventListener("change", async () => {
    for (const file of [...(input.files ?? [])]) {
      try {
        const result = await api.uploadFile(file, { kind: "gpx" });
        toast(`Subido ${result.publicPath}`, "ok");
      } catch (error) {
        fail(`No se pudo subir ${file.name}`, error);
      }
    }
    await ctx.refresh();
  });
  input.click();
}

