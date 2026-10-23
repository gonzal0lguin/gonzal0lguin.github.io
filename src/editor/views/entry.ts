// One entry, open for editing: the block editor, the frontmatter form, the GPX studio, the
// photos of the post and the raw file, next to a live preview of the real page.
import * as api from "../api";
import type { CollectionInfo, FileContent } from "../api";
import { actionsSlot, type Ctx } from "../app";
import { type Block, parseBlocks, serializeBlocks } from "../blocks";
import { button, confirmDialog, debounce, el, fail, fill, icon, ICONS, toast } from "../dom";
import { Frontmatter, parseFile, serializeFile } from "../frontmatter";
import { blocksView } from "./blocks-view";
import { entryUrl, runTask } from "./dashboard";
import { frontmatterView } from "./frontmatter-view";
import { photosView } from "./photos-view";
import { routeStudio } from "./route-studio";

export interface EntryCtx {
  app: Ctx;
  path: string;
  collection: CollectionInfo;
  key: string;
  lang: "es" | "en";
  fm: Frontmatter;
  blocks: Block[];
  /** Folder under public/assets/img/posts that new photos of this entry go to */
  folder: string;
  markDirty(): void;
  save(): Promise<void>;
  reloadPreview(): void;
  /** Redraw the open tab, e.g. after blocks were added from another panel. */
  redraw(): void;
  /** Jump to another tab. */
  openTab(id: TabId): void;
}

type TabId = "content" | "data" | "route" | "photos" | "raw";

const DRAFT_PREFIX = "editor:draft:";

export async function entryView(app: Ctx, path: string): Promise<Node> {
  if (!path) return el("p", { class: "ed-empty" }, "No entry selected");
  const file = await api.get<FileContent>("/entry", { path });
  const collectionId = path.split("/")[2] ?? "";
  const collection = app.state.collections.find((c) => c.id === collectionId)!;
  const lang: "es" | "en" = /\/en\//.test(path) ? "en" : "es";
  const key = path.replace(/^src\/content\/[^/]+\//, "").replace(/^en\//, "").replace(/\.mdx?$/, "");

  let text = file.text;
  let mtime = file.mtime;

  // A draft left behind by a previous session (the tab was closed without saving).
  const draftKey = DRAFT_PREFIX + path;
  const draft = readDraft(draftKey);
  if (draft && draft.text !== text) {
    const restore = await confirmDialog({
      title: "Hay un borrador sin guardar",
      message: `Se cerró el editor con cambios en ${path} sin guardar (${new Date(draft.at).toLocaleString()}). ¿Recuperarlos?`,
      confirm: "Recuperar",
    });
    if (restore) text = draft.text;
    else localStorage.removeItem(draftKey);
  }

  const parsed = parseFile(text);
  const entry: EntryCtx = {
    app,
    path,
    collection,
    key,
    lang,
    fm: parsed.fm,
    blocks: parseBlocks(parsed.body),
    folder: key,
    markDirty: () => setDirty(true),
    save: () => save(),
    reloadPreview: () => reloadPreview(),
    redraw: () => drawTab(active),
    openTab: (id) => selectTab(id),
  };

  let dirty = false;
  let active: TabId = "content";

  const currentText = () => serializeFile({ fm: entry.fm, body: serializeBlocks(entry.blocks), bare: parsed.bare });

  const saveDraft = debounce(() => {
    if (dirty) localStorage.setItem(draftKey, JSON.stringify({ text: currentText(), at: Date.now() }));
  }, 800);

  const saveButton = button("Guardar", () => void save(), { variant: "primary", iconPath: ICONS.save, title: "⌘/Ctrl+S" });
  const dirtyDot = el("span", { class: "ed-dot", hidden: true });

  function setDirty(value: boolean) {
    dirty = value;
    dirtyDot.hidden = !value;
    saveButton.classList.toggle("is-dirty", value);
    app.guard(value ? () => dirty : null);
    if (value) saveDraft();
  }

  async function save(force = false) {
    const text = currentText();
    try {
      const result = await api.post<{ mtime: number; backup: string | null }>("/entry/save", { path, text, mtime, force });
      mtime = result.mtime;
      localStorage.removeItem(draftKey);
      setDirty(false);
      toast(`Guardado ${path}`, "ok");
      reloadPreview();
      // The sidebar's git count and the entry list are stale now.
      void api.get<api.GitStatus>("/git/status").then((git) => (app.state.git = git)).catch(() => {});
    } catch (error) {
      const status = (error as Error & { status?: number }).status;
      if (status === 409) {
        const overwrite = await confirmDialog({
          title: "El archivo cambió en el disco",
          message: `${path} fue modificado por otra cosa (¿npm run translate, git, otra pestaña?) desde que lo abriste. Puedes sobrescribirlo con lo que tienes aquí, o recargar la página del editor para empezar desde el archivo nuevo.`,
          confirm: "Sobrescribir",
          danger: true,
        });
        if (overwrite) await save(true);
        return;
      }
      fail("No se pudo guardar", error);
    }
  }

  // --- Preview

  const previewUrl = entryUrl(collection, key, lang);
  const frame = el("iframe", { class: "ed-preview__frame", src: previewUrl, title: "Vista previa" });
  const reloadPreview = () => {
    // Astro reloads the page itself on save; this is for when it does not (or for a hard refresh).
    frame.src = `${previewUrl}${previewUrl.includes("?") ? "&" : "?"}t=${Date.now()}`;
  };
  const collapse = button("", () => layout.classList.toggle("is-collapsed"), { variant: "ghost", iconPath: ICONS.close, title: "Ocultar la vista previa" });
  const preview = el(
    "section",
    { class: "ed-preview" },
    el(
      "header",
      { class: "ed-preview__bar" },
      el("span", { class: "ed-preview__url" }, previewUrl),
      button("", () => reloadPreview(), { variant: "ghost", iconPath: ICONS.refresh, title: "Recargar la vista previa" }),
      el("a", { class: "ed-btn ed-btn--ghost", href: previewUrl, target: "_blank", rel: "noreferrer", title: "Abrir en una pestaña" }, icon(ICONS.external)),
      collapse,
    ),
    frame,
  );

  // --- Tabs

  const TABS: { id: TabId; label: string; show?: () => boolean }[] = [
    { id: "content", label: "Contenido" },
    { id: "data", label: "Datos" },
    { id: "route", label: "Ruta", show: () => collection.id === "ascents" },
    { id: "photos", label: "Fotos" },
    { id: "raw", label: "Archivo" },
  ];
  const tabBar = el("div", { class: "ed-tabs" });
  const panel = el("div", { class: "ed-panel" });

  function selectTab(id: TabId) {
    active = id;
    for (const node of tabBar.querySelectorAll("button")) node.classList.toggle("is-active", node.dataset.tab === id);
    drawTab(id);
  }

  function drawTab(id: TabId) {
    fill(panel, el("div", { class: "ed-loading" }, "…"));
    const build = async () => {
      switch (id) {
        case "content":
          return blocksView(entry);
        case "data":
          return frontmatterView(entry);
        case "route":
          return routeStudio(entry);
        case "photos":
          return photosView(entry);
        case "raw":
          return rawView(entry, currentText, (value) => {
            const next = parseFile(value);
            entry.fm = next.fm;
            entry.blocks = parseBlocks(next.body);
            setDirty(true);
          });
      }
    };
    void Promise.resolve(build())
      .then((node) => fill(panel, node))
      .catch((error) => {
        fail("No se pudo abrir la pestaña", error);
        fill(panel, el("p", { class: "ed-empty" }, String(error instanceof Error ? error.message : error)));
      });
  }

  for (const tab of TABS) {
    if (tab.show && !tab.show()) continue;
    tabBar.appendChild(el("button", { type: "button", class: "ed-tab", dataset: { tab: tab.id }, onclick: () => selectTab(tab.id) }, tab.label));
  }

  // --- Top bar

  const translation = collection.entries.find((e) => e.key === key)?.translation;
  fill(actionsSlot(),
    dirtyDot,
    saveButton,
    lang === "es"
      ? button(translation ? "Retraducir" : "Traducir", () => void runTask("translate", [path], `npm run translate -- ${path}`).then(() => app.refresh()), {
          title: "Genera o actualiza la versión en inglés con Claude (necesita ANTHROPIC_API_KEY)",
        })
      : null,
    lang === "en"
      ? button("Marcar al día", () => void runTask("stamp", [sourceOf(path)], "npm run translate -- --stamp").then(() => app.refresh()), {
          title: "Guarda el hash del original: la traducción se considera escrita a mano y al día",
        })
      : null,
    button("Vista previa", () => layout.classList.toggle("is-collapsed"), { variant: "ghost", iconPath: ICONS.eye, title: "Mostrar u ocultar el panel de la derecha" }),
    button("Volver", () => app.navigate(""), { variant: "ghost", iconPath: ICONS.back }),
  );

  app.heading(entry.fm.string("title") || key, `${path}${lang === "en" ? " · traducción" : ""}`);

  const layout = el("div", { class: "ed-split" }, el("section", { class: "ed-editor" }, tabBar, panel), preview);
  const onKey = (event: KeyboardEvent) => {
    if ((event.metaKey || event.ctrlKey) && event.key.toLowerCase() === "s") {
      event.preventDefault();
      void save();
    }
  };
  document.addEventListener("keydown", onKey);
  // The shell replaces the content node on navigation, so drop the shortcut when this editor
  // goes away. `mounted` guards the moment before the shell has appended it.
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

  selectTab("content");
  return layout;
}

const sourceOf = (path: string) => path.replace("/en/", "/");

function readDraft(key: string): { text: string; at: number } | null {
  try {
    const raw = localStorage.getItem(key);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

/** The whole file as text, for the edits a form cannot express. */
function rawView(entry: EntryCtx, read: () => string, write: (text: string) => void) {
  const area = el("textarea", { class: "ed-raw", spellcheck: "false" }, read());
  area.addEventListener("input", () => write(area.value));
  return el(
    "div",
    { class: "ed-pane" },
    el("p", { class: "ed-note" }, "El archivo completo. Los cambios aquí se aplican al resto del editor en cuanto escribes; guarda con ⌘/Ctrl+S."),
    area,
  );
}
