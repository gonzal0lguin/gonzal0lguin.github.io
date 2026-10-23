// The list of everything that can be edited: the three post collections and the standalone
// pages. Creating, renaming and deleting entries starts here too.
import * as api from "../api";
import type { CollectionInfo, EntrySummary } from "../api";
import type { Ctx } from "../app";
import { actionsSlot } from "../app";
import { ICONS, button, confirmDialog, dialog, el, fail, field, fill, icon, outputDialog, promptDialog, slugify, toast, today } from "../dom";
import { TEMPLATES } from "../schema";

export const entryRoute = (path: string) => `entry/${encodeURIComponent(path)}`;

/** Collections a new entry can be created in; `pages` is tied to the site's own routes. */
const CREATABLE = ["ascents", "blog", "projects"];

/** The page a given entry is published at, for the preview pane and the "open" links. */
export function entryUrl(collection: CollectionInfo | undefined, key: string, lang: "es" | "en" = "es") {
  const prefix = lang === "en" ? "/en" : "";
  if (!collection) return `${prefix}/`;
  if (collection.id === "pages") return key === "home" ? `${prefix}/` : `${prefix}/${key}`;
  return `${prefix}${collection.urlBase}/${key}`;
}

export function dashboardView(ctx: Ctx): Node {
  const total = ctx.state.collections.reduce((n, c) => n + c.entries.length, 0);
  ctx.heading("Contenido", `${total} entradas · ${ctx.state.root}`);

  const search = el("input", { class: "ed-input ed-input--search", type: "search", placeholder: "Buscar por título, slug o tag…" });
  const list = el("div", { class: "ed-stack" });
  const filters = el("div", { class: "ed-row ed-row--tight" });
  let only = "";

  const draw = () => {
    const term = search.value.trim().toLowerCase();
    fill(
      list,
      ...ctx.state.collections
        .filter((collection) => !only || collection.id === only)
        .map((collection) => {
          const entries = collection.entries.filter(
            (entry) => !term || `${entry.title} ${entry.key} ${entry.tags.join(" ")}`.toLowerCase().includes(term),
          );
          return el(
            "section",
            { class: "ed-card" },
            el(
              "header",
              { class: "ed-card__head" },
              el("h2", {}, collection.label, el("span", { class: "ed-chip" }, String(entries.length))),
              CREATABLE.includes(collection.id)
                ? button(`Nuevo en ${collection.label.toLowerCase()}`, () => void newEntry(ctx, collection.id), { variant: "primary", iconPath: ICONS.plus })
                : el("span", { class: "ed-note" }, "Páginas fijas del sitio"),
            ),
            entries.length === 0
              ? el("p", { class: "ed-empty" }, term ? "Nada coincide con la búsqueda" : "Nada por aquí")
              : el("ul", { class: "ed-list" }, ...entries.map((entry) => entryRow(ctx, collection, entry))),
          );
        }),
    );
  };

  const drawFilters = () => {
    fill(
      filters,
      ...[{ id: "", label: "Todo" }, ...ctx.state.collections.map((c) => ({ id: c.id, label: c.label }))].map((option) =>
        el(
          "button",
          {
            type: "button",
            class: `ed-pill ${only === option.id ? "is-active" : ""}`,
            onclick: () => {
              only = option.id;
              drawFilters();
              draw();
            },
          },
          option.label,
        ),
      ),
    );
  };

  search.addEventListener("input", draw);
  drawFilters();
  draw();

  fill(
    actionsSlot(),
    button("Nueva entrada", () => void newEntry(ctx), { variant: "primary", iconPath: ICONS.plus }),
    button("Fotos (EXIF)", () => void runTask("photos", [], "npm run photos"), { title: "Lee la ubicación EXIF de las fotos y actualiza src/data/photo-locations.json" }),
    button("Build", () => void runTask("build", [], "npm run build"), { title: "Compila el sitio completo: la comprobación más fiable antes de publicar" }),
    button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }),
  );

  return el("div", { class: "ed-stack" }, el("div", { class: "ed-toolbar" }, search, filters), list);
}

function entryRow(ctx: Ctx, collection: CollectionInfo, entry: EntrySummary) {
  const translation = entry.translation;
  const badges = [
    entry.route.length > 0 &&
      el("span", { class: "ed-chip ed-chip--route", title: entry.route.join("\n") }, `GPX ${entry.route.length > 1 ? `×${entry.route.length}` : ""}`),
    ...entry.tags.map((tag) => el("span", { class: "ed-chip ed-chip--soft" }, tag)),
    (!translation
        ? button("sin EN", () => void addTranslation(ctx, entry), { variant: "chip", title: "Crear la versión en inglés" })
        : el(
            "a",
            {
              class: `ed-chip ${translation.state === "stale" ? "ed-chip--warn" : translation.state === "manual" ? "ed-chip--soft" : "ed-chip--ok"}`,
              href: `#/${entryRoute(translation.path)}`,
              title:
                translation.state === "stale"
                  ? "La traducción quedó desactualizada"
                  : translation.state === "manual"
                    ? "Traducción escrita a mano"
                    : "Traducción al día",
            },
            `EN ${translation.state === "stale" ? "⟳" : translation.state === "manual" ? "✎" : "✓"}`,
          )),
  ];

  return el(
    "li",
    { class: "ed-list__row" },
    el(
      "a",
      { class: "ed-list__main", href: `#/${entryRoute(entry.path)}` },
      el("span", { class: "ed-list__title" }, entry.title),
      el("span", { class: "ed-list__meta" }, entry.pubDate ?? "sin fecha", " · ", entry.key),
    ),
    el("div", { class: "ed-list__badges" }, ...badges),
    el(
      "div",
      { class: "ed-list__actions" },
      el("a", { class: "ed-btn ed-btn--ghost", href: entryUrl(collection, entry.key), target: "_blank", rel: "noreferrer", title: "Ver la página" }, icon(ICONS.external)),
      button("", () => void rename(ctx, entry), { variant: "ghost", iconPath: ICONS.rename, title: "Cambiar el slug (y la URL)" }),
      button("", () => void remove(ctx, entry), { variant: "ghost", iconPath: ICONS.trash, title: "Mover a la papelera del editor" }),
    ),
  );
}

/** One dialog for a new entry of any collection: where, title, slug and date. */
async function newEntry(ctx: Ctx, preset?: string) {
  const collections = ctx.state.collections.filter((collection) => CREATABLE.includes(collection.id));
  const created = (await dialog((close) => {
    const where = el(
      "select",
      { class: "ed-select" },
      ...collections.map((collection) => el("option", { value: collection.id, selected: collection.id === (preset ?? "ascents") }, collection.label)),
    ) as HTMLSelectElement;
    const title = el("input", { class: "ed-input", placeholder: "Travesía del Arenas" });
    const slug = el("input", { class: "ed-input", placeholder: "travesia-del-arenas" });
    const date = el("input", { class: "ed-input", type: "date", value: today() });
    const url = el("p", { class: "ed-field__hint" });

    const showUrl = () => {
      const collection = collections.find((c) => c.id === where.value);
      url.textContent = `La página quedará en ${entryUrl(collection, slug.value || "slug")}`;
    };
    // The slug follows the title until it is edited by hand.
    let slugTouched = false;
    title.addEventListener("input", () => {
      if (!slugTouched) slug.value = slugify(title.value);
      showUrl();
    });
    slug.addEventListener("input", () => {
      slugTouched = true;
      slug.value = slug.value.toLowerCase().replace(/[^a-z0-9-]/g, "-");
      showUrl();
    });
    where.addEventListener("change", showUrl);
    showUrl();
    setTimeout(() => title.focus(), 30);

    return el(
      "form",
      {
        class: "ed-dialog__body",
        onsubmit: (event: Event) => {
          event.preventDefault();
          if (!title.value.trim() || !slug.value.trim()) {
            toast("Hacen falta un título y un slug", "error");
            return;
          }
          close({ collection: where.value, title: title.value.trim(), key: slug.value.trim(), date: date.value || today() });
        },
      },
      el("h2", { class: "ed-dialog__title" }, "Nueva entrada"),
      field("Colección", where),
      field("Título", title),
      field("Slug", slug),
      url,
      field("Fecha", date),
      el("div", { class: "ed-dialog__actions" }, button("Cancelar", () => close(null)), el("button", { type: "submit", class: "ed-btn ed-btn--primary" }, "Crear")),
    );
  }, { width: "min(34rem, 92vw)" })) as { collection: string; title: string; key: string; date: string } | null;

  if (!created) return;
  const text = `---\ntitle: "${created.title.replace(/"/g, '\\"')}"\ndescription: ""\npubDate: "${created.date}"\ntags: []\n---\n${TEMPLATES[created.collection] ?? "\n"}`;
  try {
    const result = await api.post<{ path: string }>("/entry/create", { collection: created.collection, key: created.key, lang: "es", ext: ".md", text });
    toast(`Creado ${result.path}`, "ok");
    await ctx.refresh();
    ctx.navigate(entryRoute(result.path));
  } catch (error) {
    fail("No se pudo crear la entrada", error);
  }
}

/** Create the English version: with Claude, or as a copy to translate by hand. */
async function addTranslation(ctx: Ctx, entry: EntrySummary) {
  const choice = (await dialog((close) =>
    el(
      "div",
      { class: "ed-dialog__body" },
      el("h2", { class: "ed-dialog__title" }, "Versión en inglés"),
      el("p", { class: "ed-dialog__text" }, `${entry.title} todavía no tiene traducción.`),
      el(
        "div",
        { class: "ed-dialog__actions" },
        button("Cancelar", () => close(null)),
        button("Copia para traducir a mano", () => close("copy"), { title: "Copia el archivo español a en/ y lo deja para editar. npm run translate no lo tocará." }),
        button("Traducir con Claude", () => close("claude"), { variant: "primary", title: "npm run translate -- " + entry.path }),
      ),
    ), { width: "min(36rem, 92vw)" })) as string | null;
  if (!choice) return;

  if (choice === "claude") {
    await runTask("translate", [entry.path], `npm run translate -- ${entry.path}`);
    await ctx.refresh();
    return;
  }
  try {
    const source = await api.get<api.FileContent>("/entry", { path: entry.path });
    const created = await api.post<{ path: string }>("/entry/create", {
      collection: entry.collection,
      key: entry.key,
      lang: "en",
      ext: entry.path.endsWith(".mdx") ? ".mdx" : ".md",
      text: source.text,
    });
    toast(`Creado ${created.path}: una copia del original, sin translationHash (npm run translate no la sobrescribirá)`, "ok");
    await ctx.refresh();
    ctx.navigate(entryRoute(created.path));
  } catch (error) {
    fail("No se pudo crear la traducción", error);
  }
}

async function rename(ctx: Ctx, entry: EntrySummary) {
  const key = await promptDialog({
    title: "Cambiar el slug",
    label: "Nuevo slug",
    value: entry.key,
    hint: "Cambia la URL de la página; la versión en inglés se mueve también. Los enlaces antiguos dejarán de funcionar.",
    confirm: "Renombrar",
  });
  if (!key || key === entry.key) return;
  try {
    await api.post("/entry/rename", { path: entry.path, key });
    toast(`Renombrado a ${key}`, "ok");
    await ctx.refresh();
  } catch (error) {
    fail("No se pudo renombrar", error);
  }
}

async function remove(ctx: Ctx, entry: EntrySummary) {
  const ok = await confirmDialog({
    title: `¿Borrar “${entry.title}”?`,
    message: `El archivo (y su traducción) se mueven a ${ctx.state.localDir}/trash, no se eliminan. Puedes recuperarlos desde ahí.`,
    confirm: "Mover a la papelera",
    danger: true,
  });
  if (!ok) return;
  try {
    const result = await api.post<{ trashed: string[] }>("/entry/delete", { path: entry.path });
    toast(`A la papelera: ${result.trashed.join(", ")}`, "ok");
    await ctx.refresh();
  } catch (error) {
    fail("No se pudo borrar", error);
  }
}

/** Run an npm script and show its output while it runs. */
export async function runTask(task: api.TaskName, files: string[], label: string) {
  const output = outputDialog(label);
  try {
    await api.runTask(task, files, (chunk) => output.write(chunk));
  } catch (error) {
    output.write(`\n[editor] ${error instanceof Error ? error.message : String(error)}\n`);
  }
}
