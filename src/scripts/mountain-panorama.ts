// Makes the panorama in src/components/MountainPanorama.astro something you can use: an
// endless strip you can drag along (it wraps round at north), bearing and zoom buttons, a
// heading readout, summits that light up on both sides — drawing and table — and the 3D tab,
// whose code (three.js and the terrain) only downloads when someone opens it.
//
// Everything it needs is already in the server-rendered markup; this only adds behaviour.

import type { PanoramaView3D } from "./panorama-3d";

const COMPASS = {
  es: ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSO", "SO", "OSO", "O", "ONO", "NO", "NNO"],
  en: ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"],
};
/** Where the hill's strip opens: the Andes, Aconcagua behind Mocoen and everything else worth the walk up. */
const OPENING_AZIMUTH = 78;

/** Open 3D views, so they can be torn down when the page goes (each holds a WebGL context). */
const views3d = new Set<PanoramaView3D>();
document.addEventListener("astro:before-swap", () => {
  for (const view of views3d) view.destroy();
  views3d.clear();
});

/** Sets up every panorama on the page (or inside `scope`) that isn't set up yet. */
export function initMountainPanorama(scope: ParentNode = document) {
  for (const root of scope.querySelectorAll<HTMLElement>("[data-panorama]:not([data-panorama-ready])")) setUp(root);
}

function setUp(root: HTMLElement) {
  const scroller = root.querySelector<HTMLElement>("[data-panorama-scroll]");
  const svg = scroller?.querySelector<SVGSVGElement>("svg");
  if (!scroller || !svg) return;
  root.dataset.panoramaReady = "true";
  const opening = Number(root.dataset.opening ?? OPENING_AZIMUTH);
  let view3d: PanoramaView3D | null = null;

  const lang = document.documentElement.lang?.startsWith("en") ? "en" : "es";
  const compass = (azimuth: number) => COMPASS[lang][Math.round(azimuth / 22.5) % 16];
  const PX = Number(svg.dataset.px);
  const OVERLAP = Number(svg.dataset.overlap);
  const FULL = 360 * PX;
  const heading = root.querySelector<HTMLElement>("[data-mp-heading]");

  let zoom = 1;
  const normalise = (azimuth: number) => ((azimuth % 360) + 360) % 360;
  /** Bearing at the middle of the visible strip. */
  const centre = () => ((scroller.scrollLeft + scroller.clientWidth / 2) / zoom - OVERLAP) / PX;
  const leftFor = (azimuth: number) => (OVERLAP + azimuth * PX) * zoom - scroller.clientWidth / 2;
  const lookAt = (azimuth: number, behavior: ScrollBehavior = "smooth") => {
    // Of the three copies of that bearing, take the nearest one the strip can actually scroll to.
    const now = centre();
    const max = scroller.scrollWidth - scroller.clientWidth;
    const base = normalise(azimuth);
    const target = [base - 360, base, base + 360]
      .filter((candidate) => leftFor(candidate) >= 0 && leftFor(candidate) <= max)
      .sort((a, b) => Math.abs(a - now) - Math.abs(b - now))[0] ?? base;
    scroller.scrollTo({ left: leftFor(target), behavior });
  };

  // The strip repeats 360° on each side of the real one; whenever the view drifts into a copy,
  // jump back by exactly one turn. The two look identical, so the jump can't be seen.
  let wrapping = false;
  const wrap = () => {
    if (wrapping) return;
    const az = centre();
    if (az < -10 || az > 370) {
      wrapping = true;
      scroller.scrollLeft += (az < 0 ? FULL : -FULL) * zoom;
      requestAnimationFrame(() => (wrapping = false));
    }
  };
  const showHeading = () => {
    if (!heading) return;
    const az = normalise(centre());
    heading.textContent = `${compass(az)} ${Math.round(az)}°`;
  };
  scroller.addEventListener("scroll", () => {
    wrap();
    showHeading();
  }, { passive: true });

  // Drag with the mouse; touch already scrolls natively.
  let drag: { x: number; left: number; moved: boolean } | null = null;
  scroller.addEventListener("pointerdown", (event) => {
    if (event.pointerType !== "mouse" || event.button !== 0) return;
    drag = { x: event.clientX, left: scroller.scrollLeft, moved: false };
  });
  window.addEventListener("pointermove", (event) => {
    if (!drag) return;
    const dx = event.clientX - drag.x;
    if (Math.abs(dx) > 3 && !drag.moved) {
      drag.moved = true;
      scroller.classList.add("is-dragging");
    }
    if (drag.moved) scroller.scrollLeft = drag.left - dx;
  });
  window.addEventListener("pointerup", () => {
    if (drag?.moved) {
      // Swallow the click that ends a drag, so it doesn't select a summit or follow a link.
      scroller.addEventListener("click", (event) => event.stopPropagation(), { capture: true, once: true });
      setTimeout(() => scroller.classList.remove("is-dragging"), 0);
    }
    drag = null;
  });
  scroller.addEventListener("keydown", (event) => {
    if (event.target !== scroller) return;
    if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
      event.preventDefault();
      lookAt(centre() + (event.key === "ArrowLeft" ? -10 : 10));
    }
  });

  // Zoom resizes the whole drawing; keep looking at the same bearing.
  const zoomButtons = [...root.querySelectorAll<HTMLButtonElement>("[data-mp-zoom]")];
  for (const button of zoomButtons) {
    button.addEventListener("click", () => {
      const az = centre();
      zoom = Number(button.dataset.mpZoom);
      svg.style.setProperty("--mp-zoom", String(zoom));
      for (const other of zoomButtons) other.classList.toggle("is-active", other === button);
      lookAt(az, "auto");
    });
  }

  // Summits: a pick on either side lights up both.
  const groups = [...root.querySelectorAll<SVGGElement>("[data-peak]")];
  const rows = [...root.querySelectorAll<HTMLTableRowElement>("[data-peak-row]")];
  const select = (id: string | null, from: "chart" | "table") => {
    for (const group of groups) group.classList.toggle("is-active", group.dataset.peak === id);
    for (const row of rows) row.classList.toggle("is-active", row.dataset.peakRow === id);
    if (!id) return;
    const group = groups.find((candidate) => candidate.dataset.peak === id);
    if (from === "table" && group) {
      const azimuth = Number(group.dataset.azimuth);
      if (view3d && !root.querySelector<HTMLElement>("[data-panorama-3d]")!.hidden) view3d.lookAt(id);
      else lookAt(azimuth);
    }
    if (from === "chart") {
      const row = rows.find((candidate) => candidate.dataset.peakRow === id);
      const details = row?.closest("details");
      if (row && details?.open) row.scrollIntoView({ block: "nearest", behavior: "smooth" });
    }
  };
  for (const group of groups) {
    const pick = () => select(group.dataset.peak!, "chart");
    group.addEventListener("click", pick);
    group.addEventListener("keydown", (event) => {
      if (event.key === "Enter" || event.key === " ") {
        event.preventDefault();
        pick();
      }
    });
  }
  for (const row of rows) {
    row.querySelector<HTMLButtonElement>("[data-peak-show]")?.addEventListener("click", () => {
      select(row.dataset.peakRow!, "table");
      root.querySelector(".mp-stage")?.scrollIntoView({ block: "nearest", behavior: "smooth" });
    });
  }
  svg.addEventListener("click", (event) => {
    if (!(event.target as Element).closest("[data-peak]")) select(null, "chart");
  });

  for (const button of root.querySelectorAll<HTMLButtonElement>("[data-jump]")) {
    button.addEventListener("click", () => {
      const azimuth = Number(button.dataset.jump);
      if (view3d && !root.querySelector<HTMLElement>("[data-panorama-3d]")!.hidden) view3d.turnTo(azimuth);
      else lookAt(azimuth);
    });
  }

  // The 3D tab.
  const container3d = root.querySelector<HTMLElement>("[data-panorama-3d]")!;
  const tabs = [...root.querySelectorAll<HTMLButtonElement>("[data-mp-tab]")];
  const zoomGroup = root.querySelector<HTMLElement>("[data-mp-zoom-group]");
  const showTab = async (tab: string) => {
    const is3d = tab === "3d";
    // Read the bearing while the strip is still on screen: hidden, it has no width.
    const facing = scroller.hidden ? null : normalise(centre());
    for (const button of tabs) button.setAttribute("aria-selected", String(button.dataset.mpTab === tab));
    scroller.hidden = is3d;
    if (heading) heading.hidden = is3d;
    container3d.hidden = !is3d;
    if (zoomGroup) zoomGroup.hidden = is3d;
    for (const hint of root.querySelectorAll<HTMLElement>("[data-mp-hint]")) hint.hidden = hint.dataset.mpHint !== tab;
    if (!is3d) {
      view3d?.pause();
      lookAt(view3d?.heading() ?? centre(), "auto");
      return;
    }
    if (view3d) {
      view3d.resume();
      if (facing !== null) view3d.turnTo(facing);
      return;
    }
    const data = JSON.parse(root.querySelector("[data-panorama-data]")!.textContent!);
    const { createPanoramaView3D } = await import("./panorama-3d");
    view3d = await createPanoramaView3D(container3d, data, facing ?? opening);
    views3d.add(view3d);
  };
  for (const button of tabs) button.addEventListener("click", () => showTab(button.dataset.mpTab!));

  // A strip set up while hidden (in a tab) has no width yet: aim it when it first shows.
  if (scroller.clientWidth) lookAt(opening, "auto");
  else {
    const observer = new ResizeObserver(() => {
      if (!scroller.clientWidth) return;
      observer.disconnect();
      lookAt(opening, "auto");
      showHeading();
    });
    observer.observe(scroller);
  }
  showHeading();
}
