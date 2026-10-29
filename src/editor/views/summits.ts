// The summit registry: one YAML file per summit of the log (src/content/summits/), and the
// things the scripts compute from it — the panorama of the view from the top, the two images it
// draws with, and the outline for the "my summits to scale" chart.
//
// A summit is not a post: no body, no translation, no URL of its own to write. What it has
// instead is derived data that goes stale, which is most of what this view is for: it shows
// whether each summit has a panorama, whether the file changed after it was computed, and runs
// `npm run panoramas` / `npm run profiles` without leaving the editor.
import * as api from "../api";
import type { CollectionInfo, SummitInfo } from "../api";
import { actionsSlot, type Ctx } from "../app";
import { ICONS, button, confirmDialog, dialog, el, fail, field, fill, formatBytes, icon, slugify, toast } from "../dom";
import { Frontmatter, SUMMIT_KEY_ORDER } from "../frontmatter";
import { SUMMIT_TEMPLATE } from "../schema";
import { entryRoute, runTask } from "./dashboard";
import type { EntryCtx } from "./entry";
import { frontmatterView } from "./frontmatter-view";

export const summitRoute = (id: string) => `summits/${encodeURIComponent(id)}`;
const summitUrl = (id: string) => `/ascents/summits/${id}/`;

/** The registry is one view with a detail: #/summits, and #/summits/<id> for one summit. */
export function summitsView(ctx: Ctx, arg: string): Node | Promise<Node> {
  return arg ? summitDetail(ctx, arg) : summitList(ctx);
}

// ------------------------------------------------------------------ the list

function summitList(ctx: Ctx): Node {
  const summits = ctx.state.summits;
  const pending = summits.filter((summit) => summit.wants && !summit.panorama && !isNorthEast(summit));
  const withoutProfile = summits.filter((summit) => !summit.profile);
  ctx.heading("Cumbres", `${summits.length} en el registro · ${summits.filter((s) => s.panorama).length} con panorama`);

  fill(
    actionsSlot(),
    button("Nueva cumbre", () => void newSummit(ctx), { variant: "primary", iconPath: ICONS.plus }),
    pending.length
      ? button(`Calcular ${pending.length} panorama(s)`, () => void runTask("panoramas", [], "npm run panoramas").then(() => ctx.refresh()), {
          title: `Las cumbres con panorama: true y nada calculado todavía: ${pending.map((summit) => summit.id).join(", ")}. Minutos por cumbre.`,
        })
      : null,
    withoutProfile.length
      ? button(`Perfiles (${withoutProfile.length} sin él)`, () => void runTask("profiles", [], "npm run profiles").then(() => ctx.refresh()), {
          title: "Recalcula src/data/profiles.json: el contorno de cada cumbre para el gráfico a escala. Segundos por cumbre.",
        })
      : button("Perfiles", () => void runTask("profiles", [], "npm run profiles").then(() => ctx.refresh()), {
          title: "Recalcula src/data/profiles.json para todas las cumbres",
        }),
    button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }),
  );

  const search = el("input", { class: "ed-input ed-input--search", type: "search", placeholder: "Buscar por nombre, zona o archivo…" });
  const list = el("div", { class: "ed-stack" });

  const draw = () => {
    const term = search.value.trim().toLowerCase();
    const shown = summits.filter((summit) => !term || `${summit.name} ${summit.id} ${summit.region ?? ""}`.toLowerCase().includes(term));
    fill(
      list,
      el(
        "section",
        { class: "ed-card" },
        el(
          "header",
          { class: "ed-card__head" },
          el("h2", {}, "Registro", el("span", { class: "ed-chip" }, String(shown.length))),
          el("span", { class: "ed-note" }, "De mayor a menor altitud"),
        ),
        el(
          "p",
          { class: "ed-note" },
          "Un archivo por cumbre. Los posts la nombran en ",
          el("code", {}, "summits:"),
          ", y de aquí salen su página, el pin del mapa, el contorno del gráfico a escala y, con ",
          el("code", {}, "panorama: true"),
          ", la vista desde la cumbre.",
        ),
        shown.length === 0
          ? el("p", { class: "ed-empty" }, term ? "Nada coincide con la búsqueda" : "El registro está vacío")
          : el("ul", { class: "ed-list" }, ...shown.map((summit) => summitRow(ctx, summit))),
      ),
      pending.length
        ? el(
            "section",
            { class: "ed-card" },
            el("header", { class: "ed-card__head" }, el("h2", {}, "Panoramas pendientes", el("span", { class: "ed-chip ed-chip--warn" }, String(pending.length)))),
            el(
              "p",
              { class: "ed-note" },
              "Tienen ",
              el("code", {}, "panorama: true"),
              " y nada calculado: su página sale sin vista. Cada una son minutos de modelo de elevación y de Overpass, así que se calculan a mano.",
            ),
            el(
              "div",
              { class: "ed-row ed-row--tight" },
              ...pending.map((summit) =>
                button(summit.id, () => void runTask("panoramas", [summit.id], `npm run panoramas -- ${summit.id}`).then(() => ctx.refresh()), { iconPath: ICONS.map }),
              ),
            ),
          )
        : null,
    );
  };

  search.addEventListener("input", draw);
  draw();
  return el("div", { class: "ed-stack" }, el("div", { class: "ed-toolbar" }, search), list);
}

function summitRow(ctx: Ctx, summit: SummitInfo): Node {
  return el(
    "li",
    { class: "ed-list__row" },
    el(
      "a",
      { class: "ed-list__main", href: `#/${summitRoute(summit.id)}` },
      el("span", { class: "ed-list__title" }, summit.name),
      el(
        "span",
        { class: "ed-list__meta" },
        summit.elevation != null ? `${summit.elevation.toLocaleString("es-CL")} m` : "sin altitud",
        summit.region ? ` · ${summit.region}` : "",
        ` · ${summit.id}`,
      ),
    ),
    el("div", { class: "ed-list__badges" }, ...stateChips(summit)),
    el(
      "div",
      { class: "ed-list__actions" },
      el("a", { class: "ed-btn ed-btn--ghost", href: summitUrl(summit.id), target: "_blank", rel: "noreferrer", title: "Ver la página de la cumbre" }, icon(ICONS.external)),
      button("", () => void removeSummit(ctx, summit), { variant: "ghost", iconPath: ICONS.trash, title: "Mover a la papelera del editor" }),
    ),
  );
}

/** The chips that say, at a glance, what this summit has and what it is missing. */
function stateChips(summit: SummitInfo): (Node | null)[] {
  const panorama = summit.panorama;
  return [
    summit.error ? el("span", { class: "ed-chip ed-chip--warn", title: summit.error }, "YAML roto") : null,
    summit.ascents.length
      ? el("span", { class: "ed-chip ed-chip--soft", title: summit.ascents.map((ascent) => ascent.title).join("\n") }, `${summit.ascents.length} ascenso${summit.ascents.length > 1 ? "s" : ""}`)
      : el("span", { class: "ed-chip ed-chip--muted", title: "Ningún post la nombra en summits:" }, "sin ascensos"),
    panorama?.broken
      ? el("span", { class: "ed-chip ed-chip--warn" }, "panorama ilegible")
      : panorama?.moved
        ? el("span", { class: "ed-chip ed-chip--warn", title: "El panorama se calculó desde otras coordenadas" }, "panorama movido")
        : panorama?.stale
          ? el("span", { class: "ed-chip ed-chip--warn", title: `El archivo cambió después de calcularlo (${panorama.generated})` }, "panorama viejo")
          : panorama
            ? el("span", { class: "ed-chip ed-chip--ok", title: `Calculado el ${panorama.generated}` }, "panorama")
            : summit.wants
              ? el("span", { class: "ed-chip ed-chip--warn", title: isNorthEast(summit) ? "Al norte o al este de 0°: el panorama se salta" : "panorama: true, sin calcular" }, isNorthEast(summit) ? "fuera de alcance" : "sin calcular")
              : el("span", { class: "ed-chip ed-chip--muted", title: "panorama: false" }, "sin panorama"),
    (panorama?.images ?? []).some((image) => image.missing) ? el("span", { class: "ed-chip ed-chip--warn" }, "falta una imagen") : null,
    summit.profile ? el("span", { class: "ed-chip ed-chip--soft", title: `Contorno desde ${summit.profile.from}°` }, "contorno") : el("span", { class: "ed-chip ed-chip--warn", title: "No está en src/data/profiles.json: npm run profiles" }, "sin contorno"),
    summit.photos ? el("span", { class: "ed-chip ed-chip--soft" }, `${summit.photos} foto${summit.photos > 1 ? "s" : ""}`) : null,
  ];
}

/** The panorama pipeline is only proven south and west of 0° (see CLAUDE.md). */
const isNorthEast = (summit: SummitInfo) => (summit.lat ?? 0) > 0 || (summit.lon ?? 0) > 0;

// ------------------------------------------------------------------ one summit

type TabId = "data" | "panorama" | "raw";

async function summitDetail(app: Ctx, id: string): Promise<Node> {
  const info = app.state.summits.find((summit) => summit.id === id);
  if (!info) return el("p", { class: "ed-empty" }, `No hay ninguna cumbre ${id} en el registro.`);
  return summitEditor(app, info);
}

async function summitEditor(app: Ctx, info: SummitInfo): Promise<Node> {
  const id = info.id;
  const file = await api.get<api.FileContent>("/summit", { path: info.path });

  let mtime = file.mtime;
  let dirty = false;
  let active: TabId = "data";
  const fm = new Frontmatter(file.text, SUMMIT_KEY_ORDER);
  // The file is the mapping itself, not a frontmatter block between two `---`.
  const currentText = () => `${fm.toYaml()}\n`.replace(/\n+$/, "\n");

  const saveButton = button("Guardar", () => void save(), { variant: "primary", iconPath: ICONS.save, title: "⌘/Ctrl+S" });
  const dirtyDot = el("span", { class: "ed-dot", hidden: true });

  function setDirty(value: boolean) {
    dirty = value;
    dirtyDot.hidden = !value;
    saveButton.classList.toggle("is-dirty", value);
    app.guard(value ? () => dirty : null);
  }

  async function save(force = false) {
    try {
      const result = await api.post<{ mtime: number }>("/summit/save", { path: info.path, text: currentText(), mtime, force });
      mtime = result.mtime;
      setDirty(false);
      toast(`Guardado ${info.path}`, "ok");
      reloadPreview();
      // The chips (and the panorama's "viejo") are computed from the file's mtime on the server.
      void api.get<api.EditorState>("/state").then((state) => {
        app.state.summits = state.summits;
        app.state.git = state.git;
      }).catch(() => {});
    } catch (error) {
      if ((error as Error & { status?: number }).status === 409) {
        const overwrite = await confirmDialog({
          title: "El archivo cambió en el disco",
          message: `${info.path} fue modificado por otra cosa desde que lo abriste. Puedes sobrescribirlo, o recargar el editor para empezar desde el archivo nuevo.`,
          confirm: "Sobrescribir",
          danger: true,
        });
        if (overwrite) await save(true);
        return;
      }
      fail("No se pudo guardar", error);
    }
  }

  // The summit's own page, as the site builds it.
  const previewUrl = summitUrl(id);
  const frame = el("iframe", { class: "ed-preview__frame", src: previewUrl, title: "Vista previa" });
  const reloadPreview = () => {
    frame.src = `${previewUrl}?t=${Date.now()}`;
  };
  const preview = el(
    "section",
    { class: "ed-preview" },
    el(
      "header",
      { class: "ed-preview__bar" },
      el("span", { class: "ed-preview__url" }, previewUrl),
      button("", () => reloadPreview(), { variant: "ghost", iconPath: ICONS.refresh, title: "Recargar la vista previa" }),
      el("a", { class: "ed-btn ed-btn--ghost", href: previewUrl, target: "_blank", rel: "noreferrer", title: "Abrir en una pestaña" }, icon(ICONS.external)),
      button("", () => layout.classList.toggle("is-collapsed"), { variant: "ghost", iconPath: ICONS.close, title: "Ocultar la vista previa" }),
    ),
    frame,
  );

  // The form is the one the posts use: a summit is just another collection of fields
  // (FIELDS.summits), so it gets the map picker, the image picker and the untouched-keys note.
  const collection: CollectionInfo = { id: "summits", label: "Cumbres", urlBase: "/ascents/summits", mediaFolder: false, entries: [] };
  const entry: EntryCtx = {
    app,
    path: info.path,
    collection,
    key: id,
    lang: "es",
    fm,
    blocks: [],
    folder: "cumbres",
    markDirty: () => setDirty(true),
    save: () => save(),
    reloadPreview: () => reloadPreview(),
    redraw: () => drawTab(active),
    openTab: () => {},
  };

  const tabBar = el("div", { class: "ed-tabs" });
  const panel = el("div", { class: "ed-panel" });
  const TABS: { id: TabId; label: string }[] = [
    { id: "data", label: "Datos" },
    { id: "panorama", label: "Panorama" },
    { id: "raw", label: "Archivo" },
  ];

  function drawTab(tab: TabId) {
    active = tab;
    for (const node of tabBar.querySelectorAll("button")) node.classList.toggle("is-active", node.dataset.tab === tab);
    if (tab === "data") fill(panel, frontmatterView(entry));
    else if (tab === "panorama") fill(panel, panoramaPane(app, info));
    else fill(panel, rawPane(currentText, (text) => {
      // The whole file, for what the form cannot say. Re-reading it keeps the two in step.
      const next = new Frontmatter(text, SUMMIT_KEY_ORDER);
      for (const key of new Set([...fm.keys(), ...next.keys()])) fm.set(key, next.get(key));
      setDirty(true);
    }));
  }

  for (const tab of TABS) {
    tabBar.appendChild(el("button", { type: "button", class: "ed-tab", dataset: { tab: tab.id }, onclick: () => drawTab(tab.id) }, tab.label));
  }

  fill(
    actionsSlot(),
    dirtyDot,
    saveButton,
    button("Calcular el panorama", () => void recompute(app, info), {
      title: `npm run panoramas -- ${id}: recalcula la vista desde la cumbre. Minutos.`,
      iconPath: ICONS.map,
    }),
    button("Vista previa", () => layout.classList.toggle("is-collapsed"), { variant: "ghost", iconPath: ICONS.eye }),
    button("Volver", () => app.navigate("summits"), { variant: "ghost", iconPath: ICONS.back }),
  );
  app.heading(info.name, `${info.path}${info.elevation != null ? ` · ${info.elevation.toLocaleString("es-CL")} m` : ""}`);

  const layout = el("div", { class: "ed-split" }, el("section", { class: "ed-editor" }, tabBar, panel), preview);
  const onKey = (event: KeyboardEvent) => {
    if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === "s") {
      event.preventDefault();
      void save();
    }
  };
  document.addEventListener("keydown", onKey);
  let mounted = false;
  new MutationObserver((_, observer) => {
    if (layout.isConnected) {
      mounted = true;
      return;
    }
    if (!mounted) return;
    document.removeEventListener("keydown", onKey);
    observer.disconnect();
  }).observe(document.body, { childList: true, subtree: true });

  drawTab("data");
  return layout;
}

/** What `npm run panoramas` and `npm run profiles` left for this summit, and how to redo it. */
function panoramaPane(ctx: Ctx, summit: SummitInfo): Node {
  const panorama = summit.panorama;
  const state = !panorama
    ? summit.wants
      ? isNorthEast(summit)
        ? "Esta cumbre está al norte o al este de 0°, donde el panorama todavía no se ha probado: tiene contorno, pero no vista. scripts/panoramas.mjs la salta."
        : "Marcada con panorama: true, pero no hay nada calculado: la página sale sin vista."
      : "Sin panorama: la página muestra los ascensos y nada más. Marca «Calcular el panorama» en Datos para pedir uno."
    : panorama.broken
      ? "El JSON del panorama no se puede leer. Hay que calcularlo otra vez."
      : panorama.moved
        ? "El panorama se calculó desde otras coordenadas: lo que muestra la página es la vista del sitio anterior."
        : panorama.stale
          ? "El archivo de la cumbre cambió después de calcular el panorama. Si lo que cambió son las coordenadas, la altitud o el nodo de OSM, conviene recalcularlo."
          : "Al día.";

  return el(
    "div",
    { class: "ed-pane ed-stack" },
    el(
      "section",
      { class: "ed-card" },
      el(
        "header",
        { class: "ed-card__head" },
        el("h2", {}, "La vista desde la cumbre"),
        button("Calcular", () => void recompute(ctx, summit), { variant: panorama ? "" : "primary", iconPath: ICONS.map, title: `npm run panoramas -- ${summit.id}` }),
      ),
      el("p", { class: panorama && !panorama.broken && !panorama.stale && !panorama.moved ? "ed-note" : "ed-note ed-warn" }, state),
      panorama
        ? el(
            "p",
            { class: "ed-note" },
            "Calculado el ",
            el("strong", {}, panorama.generated ?? "?"),
            " · ",
            el("code", {}, panorama.path),
          )
        : null,
      panorama
        ? el(
            "div",
            { class: "ed-picker" },
            ...panorama.images.map((image) =>
              el(
                "div",
                { class: `ed-picker__item ${image.missing ? "is-unused" : ""}` },
                image.missing
                  ? el("div", { class: "ed-filecard" }, el("span", {}, "NO ESTÁ"))
                  : el("a", { href: image.publicPath, target: "_blank", rel: "noreferrer" }, el("img", { src: image.publicPath, alt: "", loading: "lazy" })),
                el("span", { title: image.path }, image.kind === "terrain" ? "Malla del terreno (vista 3D)" : "Cuenca visual (capa del mapa)"),
                el("small", {}, image.missing ? "¡falta! hay que recalcular" : formatBytes(image.size)),
              ),
            ),
          )
        : null,
    ),
    el(
      "section",
      { class: "ed-card" },
      el(
        "header",
        { class: "ed-card__head" },
        el("h2", {}, "Contorno a escala"),
        button("Recalcular todos", () => void runTask("profiles", [], "npm run profiles").then(() => ctx.refresh()), { iconPath: ICONS.refresh, title: "npm run profiles: segundos por cumbre" }),
      ),
      summit.profile
        ? el(
            "p",
            { class: "ed-note" },
            `El contorno está en src/data/profiles.json: ${summit.profile.points} puntos, visto desde ${summit.profile.from}°`,
            summit.profile.side === "track" ? " (el lado por donde empieza el track)." : " (desde el norte, sin track que diga otra cosa).",
          )
        : el("p", { class: "ed-note ed-warn" }, "Esta cumbre no está en src/data/profiles.json, así que no sale en el gráfico «mis cumbres a escala» de la página de ascensos."),
    ),
    el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, "Ascensos a esta cumbre", el("span", { class: "ed-chip" }, String(summit.ascents.length)))),
      summit.ascents.length === 0
        ? el("p", { class: "ed-note" }, "Ningún post la nombra todavía. Se añade en el campo «Cumbres» del post, en su pestaña Datos.")
        : el(
            "ul",
            { class: "ed-list ed-list--tight" },
            ...summit.ascents.map((ascent) =>
              el(
                "li",
                { class: "ed-list__row" },
                el(
                  "a",
                  { class: "ed-list__main", href: `#/${entryRoute(ascent.path)}` },
                  el("span", { class: "ed-list__title" }, ascent.title),
                  el("span", { class: "ed-list__meta" }, ascent.pubDate ?? "sin fecha"),
                ),
                ascent.attempt ? el("span", { class: "ed-chip ed-chip--soft", title: "El post está marcado como intento" }, "intento") : null,
              ),
            ),
          ),
    ),
  );
}

function rawPane(read: () => string, write: (text: string) => void) {
  const area = el("textarea", { class: "ed-raw", spellcheck: "false" }, read());
  area.addEventListener("input", () => write(area.value));
  return el(
    "div",
    { class: "ed-pane" },
    el("p", { class: "ed-note" }, "El archivo YAML completo. Los cambios se aplican al formulario en cuanto escribes; guarda con ⌘/Ctrl+S."),
    area,
  );
}

// ------------------------------------------------------------------ actions

/** Run the panorama for one summit. It is minutes of work, so it says so before starting. */
async function recompute(ctx: Ctx, summit: SummitInfo) {
  if (isNorthEast(summit)) {
    toast("scripts/panoramas.mjs salta las cumbres al norte o al este de 0°: no se calculará nada", "error");
    return;
  }
  const ok = await confirmDialog({
    title: `¿Calcular el panorama de ${summit.name}?`,
    message:
      "Lee el modelo de elevación y pregunta a Overpass por las cumbres, glaciares, pueblos y lagos de la región. La primera vez de una zona son varios minutos; después, con la caché, bastante menos. Escribe src/data/panoramas y public/assets/panoramas.",
    confirm: "Calcular",
  });
  if (!ok) return;
  await runTask("panoramas", [summit.id], `npm run panoramas -- ${summit.id}`);
  await ctx.refresh();
}

/** A new summit: the file name is what the posts write in `summits:`, so it is asked for here. */
async function newSummit(ctx: Ctx) {
  const created = (await dialog((close) => {
    const name = el("input", { class: "ed-input", placeholder: "Cerro Provincia" });
    const id = el("input", { class: "ed-input", placeholder: "provincia" });
    const hint = el("p", { class: "ed-field__hint" });
    const show = () => {
      hint.textContent = `Archivo src/content/summits/${id.value || "nombre"}.yaml · página /ascents/summits/${id.value || "nombre"}/`;
    };
    let touched = false;
    name.addEventListener("input", () => {
      if (!touched) id.value = slugify(name.value);
      show();
    });
    id.addEventListener("input", () => {
      touched = true;
      id.value = id.value.toLowerCase().replace(/[^a-z0-9-]/g, "-");
      show();
    });
    show();
    setTimeout(() => name.focus(), 30);

    return el(
      "form",
      {
        class: "ed-dialog__body",
        onsubmit: (event: Event) => {
          event.preventDefault();
          if (!name.value.trim() || !id.value.trim()) {
            toast("Hacen falta un nombre y un nombre de archivo", "error");
            return;
          }
          close({ name: name.value.trim(), id: id.value.trim() });
        },
      },
      el("h2", { class: "ed-dialog__title" }, "Nueva cumbre"),
      field("Nombre", name, "Como se muestra en la página, con cerro, volcán o punta"),
      field("Nombre de archivo", id, "Lo que los posts escriben en summits: y la URL de su página"),
      hint,
      el("p", { class: "ed-note" }, "Se crea con la altitud y las coordenadas en cero: se rellenan en el formulario, con el mapa."),
      el("div", { class: "ed-dialog__actions" }, button("Cancelar", () => close(null)), el("button", { type: "submit", class: "ed-btn ed-btn--primary" }, "Crear")),
    );
  }, { width: "min(34rem, 92vw)" })) as { name: string; id: string } | null;
  if (!created) return;

  try {
    const result = await api.post<{ id: string }>("/summit/create", { id: created.id, text: SUMMIT_TEMPLATE(created.name) });
    toast(`Creada ${result.id}: pon la altitud y las coordenadas antes de calcular nada`, "ok");
    await ctx.refresh();
    ctx.navigate(summitRoute(result.id));
  } catch (error) {
    fail("No se pudo crear la cumbre", error);
  }
}

/**
 * Take a summit out of the registry. The posts that name it are listed first: their `summits:`
 * has to lose the reference too, or the build fails on a link to a file that is not there.
 */
async function removeSummit(ctx: Ctx, summit: SummitInfo) {
  const used = summit.ascents.map((ascent) => ascent.title).join(", ");
  const ok = await confirmDialog({
    title: `¿Borrar “${summit.name}”?`,
    message:
      `El archivo, su panorama y sus imágenes se mueven a ${ctx.state.localDir}/trash, no se eliminan.` +
      (used ? ` Ojo: ${summit.ascents.length} post(s) la nombran en summits: (${used}). Hay que quitarla de ahí, o el build falla.` : ""),
    confirm: "Mover a la papelera",
    danger: true,
  });
  if (!ok) return;
  try {
    const result = await api.post<{ trashed: string[] }>("/summit/delete", { id: summit.id });
    toast(`A la papelera: ${result.trashed.join(", ")}`, "ok");
    await ctx.refresh();
  } catch (error) {
    fail("No se pudo borrar", error);
  }
}
