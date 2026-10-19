// Browser side of the photo layouts built by src/plugins/remark-media.mjs: the carousel
// controls and the lightbox shared by every gallery. Everything degrades gracefully —
// without this script the carousel is still a scrollable row of photos.

import { getLangFromUrl, useTranslations } from "../i18n/ui";

const ICONS = {
  prev: '<path d="M14.5 5 8 12l6.5 7" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>',
  next: '<path d="M9.5 5 16 12l-6.5 7" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>',
  close: '<path d="M6 6l12 12M18 6 6 18" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>',
};

const smooth = () => (window.matchMedia("(prefers-reduced-motion: reduce)").matches ? "auto" : "smooth");
const icon = (name: keyof typeof ICONS) =>
  `<svg viewBox="0 0 24 24" width="20" height="20" aria-hidden="true" focusable="false">${ICONS[name]}</svg>`;

export function initMedia() {
  document.querySelectorAll<HTMLElement>("[data-carousel]:not([data-ready])").forEach(initCarousel);
  initLightbox();
}

/* ---------------------------------- carousel ---------------------------------- */

function initCarousel(root: HTMLElement) {
  const track = root.querySelector<HTMLElement>(".photo-carousel__track");
  if (!track) return;
  root.dataset.ready = "";

  const slides = Array.from(track.children) as HTMLElement[];
  const dots = Array.from(root.querySelectorAll<HTMLButtonElement>("[data-slide]"));
  const arrows = Array.from(root.querySelectorAll<HTMLButtonElement>("[data-direction]"));
  let index = 0;

  // A slide's scroll offset: its layout position inside the track, which neither the
  // current scroll position nor the gap between slides affects.
  const offsetOf = (slide: HTMLElement) => slide.offsetLeft - track.offsetLeft;

  const goTo = (to: number) => {
    const clamped = Math.max(0, Math.min(to, slides.length - 1));
    track.scrollTo({ left: offsetOf(slides[clamped]), behavior: smooth() });
  };

  const sync = () => {
    const center = track.scrollLeft + track.clientWidth / 2;
    index = slides.reduce((best, slide, i) => {
      const distance = Math.abs(offsetOf(slide) + slide.offsetWidth / 2 - center);
      return distance < Math.abs(offsetOf(slides[best]) + slides[best].offsetWidth / 2 - center) ? i : best;
    }, 0);
    dots.forEach((dot, i) => dot.setAttribute("aria-current", String(i === index)));
    arrows.forEach((arrow) => {
      arrow.disabled = arrow.dataset.direction === "prev" ? index === 0 : index === slides.length - 1;
    });
  };

  let queued = false;
  track.addEventListener("scroll", () => {
    if (queued) return;
    queued = true;
    requestAnimationFrame(() => {
      queued = false;
      sync();
    });
  }, { passive: true });

  arrows.forEach((arrow) => arrow.addEventListener("click", () => goTo(index + (arrow.dataset.direction === "prev" ? -1 : 1))));
  dots.forEach((dot) => dot.addEventListener("click", () => goTo(Number(dot.dataset.slide))));

  track.addEventListener("keydown", (event) => {
    const step = event.key === "ArrowLeft" ? -1 : event.key === "ArrowRight" ? 1 : 0;
    if (!step) return;
    event.preventDefault();
    goTo(index + step);
  });

  window.addEventListener("resize", sync, { passive: true });
  sync();
}

/* ---------------------------------- lightbox ---------------------------------- */

interface Lightbox {
  root: HTMLElement;
  lang: ReturnType<typeof lang>;
  image: HTMLImageElement;
  caption: HTMLElement;
  count: HTMLElement;
  arrows: HTMLButtonElement[];
  close: HTMLButtonElement;
}

let lightbox: Lightbox | null = null;
let photos: HTMLImageElement[] = [];
let current = 0;
let opener: HTMLElement | null = null;

const lang = () => getLangFromUrl(new URL(location.href));

function initLightbox() {
  if (document.querySelector("[data-lightbox]")) getLightbox();
}

/**
 * The single lightbox shared by every gallery, built on demand. A view transition replaces
 * `document.body`, which detaches it, and may also land on the other language, so it is
 * rebuilt whenever it is no longer in the page or its labels no longer match.
 */
function getLightbox(): Lightbox {
  if (!lightbox || !lightbox.root.isConnected || lightbox.lang !== lang()) {
    lightbox?.root.remove();
    lightbox = createLightbox();
    document.body.append(lightbox.root);
  }
  return lightbox;
}

function createLightbox(): Lightbox {
  const t = useTranslations(lang());
  const root = document.createElement("div");
  root.className = "photo-lightbox";
  root.hidden = true;
  root.setAttribute("role", "dialog");
  root.setAttribute("aria-modal", "true");
  root.innerHTML = `
    <button type="button" class="photo-lightbox__close" aria-label="${t("media.close")}">${icon("close")}</button>
    <div class="photo-lightbox__stage"><img alt="" /></div>
    <div class="photo-lightbox__bar">
      <button type="button" data-direction="prev" aria-label="${t("media.prev")}">${icon("prev")}</button>
      <span class="photo-lightbox__caption"></span>
      <span class="photo-lightbox__count"></span>
      <button type="button" data-direction="next" aria-label="${t("media.next")}">${icon("next")}</button>
    </div>`;

  const view: Lightbox = {
    root,
    lang: lang(),
    image: root.querySelector("img")!,
    caption: root.querySelector(".photo-lightbox__caption")!,
    count: root.querySelector(".photo-lightbox__count")!,
    arrows: Array.from(root.querySelectorAll<HTMLButtonElement>("[data-direction]")),
    close: root.querySelector(".photo-lightbox__close")!,
  };

  // Clicking the photo itself or a control does its own thing; anything else closes.
  root.addEventListener("click", (event) => {
    const target = event.target as HTMLElement;
    if (target.closest("[data-direction]") || target === view.image) return;
    closeLightbox();
  });
  view.arrows.forEach((arrow) => arrow.addEventListener("click", () => show(current + (arrow.dataset.direction === "prev" ? -1 : 1))));
  return view;
}

// One delegated listener for every gallery on the page, now and after a page swap.
document.addEventListener("click", (event) => {
  const image = (event.target as HTMLElement).closest<HTMLImageElement>("[data-lightbox] img");
  if (!image) return;
  const group = image.closest<HTMLElement>("[data-lightbox]")!;
  photos = Array.from(group.querySelectorAll("img"));
  opener = image;
  const view = getLightbox();
  show(photos.indexOf(image));
  view.root.hidden = false;
  document.documentElement.style.overflow = "hidden";
  view.close.focus();
});

document.addEventListener("keydown", (event) => {
  if (!lightbox || lightbox.root.hidden) return;
  if (event.key === "Escape") closeLightbox();
  else if (event.key === "ArrowLeft") show(current - 1);
  else if (event.key === "ArrowRight") show(current + 1);
});

// A view transition replaces the page under the lightbox.
document.addEventListener("astro:before-swap", () => {
  if (lightbox && !lightbox.root.hidden) closeLightbox();
});

function show(to: number) {
  if (!lightbox || photos.length === 0) return;
  const t = useTranslations(lightbox.lang);
  current = Math.max(0, Math.min(to, photos.length - 1));
  const photo = photos[current];
  lightbox.image.src = photo.currentSrc || photo.src;
  lightbox.image.alt = photo.alt;
  lightbox.caption.textContent = photo.closest("figure")?.querySelector("figcaption")?.textContent ?? "";
  lightbox.count.textContent = photos.length > 1 ? t("media.count", { i: current + 1, n: photos.length }) : "";
  lightbox.arrows.forEach((arrow) => {
    arrow.hidden = photos.length < 2;
    arrow.disabled = arrow.dataset.direction === "prev" ? current === 0 : current === photos.length - 1;
  });
}

function closeLightbox() {
  if (!lightbox) return;
  lightbox.root.hidden = true;
  lightbox.image.removeAttribute("src");
  document.documentElement.style.removeProperty("overflow");
  opener?.focus({ preventScroll: true });
  opener = null;
}
