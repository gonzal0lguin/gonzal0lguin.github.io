// The editor shell: loads the state, draws the sidebar and swaps views on the hash.
// Reached at /edit while `npm run dev` is running, and nowhere else.

import * as api from "./api";
import type { EditorState } from "./api";
import { ICONS, clear, el, fail, icon, toast } from "./dom";
import { setMediaBase } from "./media-url";
import { dashboardView } from "./views/dashboard";
import { entryView } from "./views/entry";
import { gitView } from "./views/git";
import { healthView } from "./views/health";
import { mediaView } from "./views/media";
import { routesView } from "./views/routes";
import { siteView } from "./views/site";
import { summitsView } from "./views/summits";

export interface Ctx {
  state: EditorState;
  /** Reload everything from disk and redraw the current view. */
  refresh(): Promise<void>;
  navigate(route: string): void;
  /** While set, leaving the view asks for confirmation (unsaved changes). */
  guard(fn: (() => boolean) | null): void;
  /** Shown in the top bar. */
  heading(title: string, subtitle?: string): void;
}

interface Route {
  view: string;
  arg: string;
}

const NAV = [
  { route: "", label: "Contenido", iconPath: ICONS.save },
  { route: "site", label: "Sitio", iconPath: ICONS.home },
  { route: "summits", label: "Cumbres", iconPath: ICONS.mountain },
  { route: "media", label: "Fotos", iconPath: ICONS.image },
  { route: "routes", label: "Rutas GPX", iconPath: ICONS.map },
  { route: "git", label: "Cambios", iconPath: ICONS.check },
  { route: "health", label: "Revisión", iconPath: ICONS.eye },
];

const parseHash = (): Route => {
  const hash = location.hash.replace(/^#\/?/, "");
  const [view, ...rest] = hash.split("/");
  return { view: view ?? "", arg: decodeURIComponent(rest.join("/")) };
};

export async function startEditor(root: HTMLElement) {
  let state: EditorState;
  let dirtyGuard: (() => boolean) | null = null;

  const title = el("h1", { class: "ed-topbar__title" }, "Editor");
  const subtitle = el("p", { class: "ed-topbar__subtitle" });
  const content = el("div", { class: "ed-content" });
  const navList = el("nav", { class: "ed-nav" });
  const statusLine = el("div", { class: "ed-sidebar__status" });

  const ctx: Ctx = {
    get state() {
      return state;
    },
    async refresh() {
      state = await api.get<EditorState>("/state");
      setMediaBase(state.mediaBaseUrl);
      await render();
    },
    navigate(route: string) {
      location.hash = `#/${route}`;
    },
    guard(fn) {
      dirtyGuard = fn;
    },
    heading(text: string, sub = "") {
      title.textContent = text;
      subtitle.textContent = sub;
      document.title = `${text} · Editor`;
    },
  };

  const views: Record<string, (ctx: Ctx, arg: string) => Node | Promise<Node>> = {
    "": dashboardView,
    entry: entryView,
    site: siteView,
    summits: summitsView,
    media: mediaView,
    routes: routesView,
    git: gitView,
    health: healthView,
  };

  async function render() {
    const { view, arg } = parseHash();
    const build = views[view];
    drawNav(view);
    clear(content);
    if (!build) {
      content.appendChild(el("p", { class: "ed-empty" }, `Vista desconocida: ${view}`));
      return;
    }
    content.appendChild(el("div", { class: "ed-loading" }, "Cargando…"));
    try {
      const node = await build(ctx, arg);
      clear(content);
      content.appendChild(node);
    } catch (error) {
      clear(content);
      fail("No se pudo abrir la vista", error);
      content.appendChild(el("p", { class: "ed-empty" }, String(error instanceof Error ? error.message : error)));
    }
    drawStatus();
  }

  function drawNav(active: string) {
    clear(navList);
    for (const item of NAV) {
      navList.appendChild(
        el(
          "a",
          { class: `ed-nav__item ${item.route === active || (item.route === "" && active === "entry") ? "is-active" : ""}`, href: `#/${item.route}` },
          icon(item.iconPath),
          el("span", {}, item.label),
        ),
      );
    }
  }

  function drawStatus() {
    const changed = state.git?.files?.length ?? 0;
    clear(statusLine);
    statusLine.append(
      el("span", { class: "ed-chip" }, state.git?.branch ?? "git?"),
      el("span", { class: `ed-chip ${changed ? "ed-chip--warn" : ""}` }, changed ? `${changed} sin guardar en git` : "árbol limpio"),
    );
    if (state.mediaBaseUrl) statusLine.appendChild(el("span", { class: "ed-chip" }, `fotos: ${state.mediaBaseUrl}`));
  }

  // The guard catches both a hash change and closing the tab.
  let lastHash = location.hash;
  addEventListener("hashchange", () => {
    if (dirtyGuard?.()) {
      if (!confirm("Hay cambios sin guardar. ¿Salir de todas formas?")) {
        location.hash = lastHash;
        return;
      }
    }
    dirtyGuard = null;
    lastHash = location.hash;
    void render();
  });
  addEventListener("beforeunload", (event) => {
    if (dirtyGuard?.()) event.preventDefault();
  });

  root.append(
    el(
      "aside",
      { class: "ed-sidebar" },
      el("a", { class: "ed-brand", href: "#/" }, el("span", { class: "ed-brand__dot" }), "Editor del sitio"),
      navList,
      el("div", { class: "ed-sidebar__spacer" }),
      statusLine,
      el("a", { class: "ed-sidebar__link", href: "/", target: "_blank", rel: "noreferrer" }, "Abrir el sitio ↗"),
    ),
    el("main", { class: "ed-main" }, el("header", { class: "ed-topbar" }, el("div", {}, title, subtitle), el("div", { class: "ed-topbar__actions", id: "ed-actions" })), content),
  );

  try {
    state = await api.get<EditorState>("/state");
    setMediaBase(state.mediaBaseUrl);
  } catch (error) {
    root.appendChild(
      el(
        "div",
        { class: "ed-fatal" },
        el("h2", {}, "El editor no responde"),
        el("p", {}, "La API del editor sólo existe con el servidor de desarrollo (`npm run dev`). Detalle:"),
        el("pre", {}, String(error instanceof Error ? error.message : error)),
      ),
    );
    return;
  }
  if (!location.hash) location.hash = "#/";
  await render();
  toast("Editor listo — guarda con ⌘/Ctrl+S", "ok");
}

/** The slot in the top bar where a view puts its own buttons. */
export const actionsSlot = () => document.getElementById("ed-actions")!;
