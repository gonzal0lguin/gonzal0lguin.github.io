// The tabs of an ascent post (src/layouts/AscentLayout.astro): the story, the GPS route and the
// view from the summit. The tab in use is kept in the address (#route, #panorama) so it can be
// linked to. The summit panorama is heavy — a 360° drawing, a terrain mesh, a map — and lives on
// the summit's own page, so the tab fetches it from there the first time it opens.
//
// Anything inside a hidden tab can ask to be shown by dispatching a bubbling "tabs:reveal"
// event (the route map does, when a photo in the story points at it).

const loads = new WeakMap<HTMLElement, Promise<void>>();

export function initAscentTabs() {
  const root = document.querySelector<HTMLElement>("[data-ascent]");
  if (!root || root.dataset.tabsReady) return;
  root.dataset.tabsReady = "";

  const buttons = [...root.querySelectorAll<HTMLButtonElement>("[data-tab]")];
  const panels = [...root.querySelectorAll<HTMLElement>("[data-tab-panel]")];
  const ids = buttons.map((button) => button.dataset.tab!);

  const show = (id: string, { focus = false } = {}) => {
    for (const button of buttons) {
      const on = button.dataset.tab === id;
      button.setAttribute("aria-selected", String(on));
      button.tabIndex = on ? 0 : -1;
      if (on && focus) button.focus();
    }
    for (const panel of panels) panel.hidden = panel.dataset.tabPanel !== id;
    const hash = id === "story" ? "" : `#${id}`;
    if (location.hash !== hash) history.replaceState(history.state, "", `${location.pathname}${location.search}${hash}`);
    if (id === "panorama") {
      const frame = root.querySelector<HTMLElement>("[data-summit-frame]:not([hidden])");
      if (frame) load(frame);
    }
  };

  buttons.forEach((button, i) => {
    button.addEventListener("click", () => show(button.dataset.tab!));
    // Arrow keys move along the tabs, as a tablist should.
    button.addEventListener("keydown", (event) => {
      const step = event.key === "ArrowRight" ? 1 : event.key === "ArrowLeft" ? -1 : 0;
      if (!step) return;
      event.preventDefault();
      show(ids[(i + step + ids.length) % ids.length], { focus: true });
    });
  });

  root.addEventListener("tabs:reveal", (event) => {
    const panel = (event.target as Element).closest<HTMLElement>("[data-tab-panel]");
    if (panel?.hidden) show(panel.dataset.tabPanel!);
  });

  // Several summits (a traverse): one panorama at a time.
  const picks = [...root.querySelectorAll<HTMLButtonElement>("[data-summit-pick]")];
  for (const pick of picks) {
    pick.addEventListener("click", () => {
      for (const other of picks) other.setAttribute("aria-pressed", String(other === pick));
      for (const frame of root.querySelectorAll<HTMLElement>("[data-summit-frame]")) {
        frame.hidden = frame.dataset.summitId !== pick.dataset.summitPick;
        if (!frame.hidden) load(frame);
      }
    });
  }

  const wanted = location.hash.slice(1);
  if (ids.includes(wanted)) show(wanted);
}

/** Brings the summit's view over from its page, once, and sets up its drawing and map. */
function load(frame: HTMLElement) {
  if (!loads.has(frame)) loads.set(frame, fetchView(frame));
  return loads.get(frame)!;
}

async function fetchView(frame: HTMLElement) {
  const status = frame.querySelector<HTMLElement>("[data-summit-status]");
  try {
    const response = await fetch(frame.dataset.summitFrame!);
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const page = new DOMParser().parseFromString(await response.text(), "text/html");
    const view = page.querySelector("[data-summit-view]");
    if (!view) throw new Error("no summit view on the page");
    status?.remove();
    frame.append(document.importNode(view, true));
    const [{ initMountainPanorama }, { initViewshedMap }] = await Promise.all([import("./mountain-panorama"), import("./viewshed-map")]);
    initMountainPanorama(frame);
    initViewshedMap(frame);
  } catch {
    loads.delete(frame);
    if (status) {
      status.textContent = "";
      const link = document.createElement("a");
      link.href = frame.dataset.summitFrame!;
      link.textContent = document.documentElement.lang.startsWith("en") ? "The panorama didn't load — open it on the summit's page →" : "No se pudo cargar el panorama — ábrelo en la página de la cumbre →";
      status.append(link);
    }
  }
}
