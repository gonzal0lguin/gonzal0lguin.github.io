// HTML of the route card (stats, map, elevation profile, days, download), shared by the posts
// (rendered at build time by RouteMap.astro) and the route tool (rendered in the browser after
// analyzing the visitor's files). src/scripts/route-map.ts then brings the map and profile to life.

import type { RouteData } from "./gpx-core";
import type { RoutePhoto } from "./photos";
import {
  formatClock,
  formatDateRange,
  formatDay,
  formatDistance,
  formatDuration,
  formatElevation,
  formatPercent,
} from "./route-format";
import { languages, useTranslations, type Lang } from "../i18n/ui";

export interface RouteCardOptions {
  lang: Lang;
  title?: string | null;
  /** IANA timezone used to show clock times */
  timezone?: string;
  /** URL of the GPX file offered for download */
  download: string;
  /** File name for the download; the URL's own name when omitted */
  downloadName?: string;
  /** URL of the route as KML, to open it whole in Google Earth */
  kml?: string;
  kmlName?: string;
  /** Post photos placed on the route */
  photos?: RoutePhoto[];
  /** Route tool: points can be marked, and the analysis starts open */
  editable?: boolean;
}

const icons: Record<string, string> = {
  distance: "M3 17l6-6 4 4 8-8M14 7h7v7",
  up: "M12 19V5M5 12l7-7 7 7",
  peak: "M3 20l6.5-11 4 6.5 2.5-4L21 20z",
  time: "M12 7v5l3 2M21 12a9 9 0 1 1-18 0 9 9 0 0 1 18 0z",
};

const esc = (value: unknown) =>
  String(value).replace(/&/g, "&amp;").replace(/</g, "&lt;").replace(/>/g, "&gt;").replace(/"/g, "&quot;");

export function renderRouteCard(
  route: RouteData,
  { lang, title, timezone = "America/Santiago", download, downloadName, kml, kmlName, photos = [], editable = false }: RouteCardOptions,
) {
  const t = useTranslations(lang);
  const locale = languages[lang].locale;
  const { stats, days, camps } = route;

  const primaryStats = [
    { label: t("route.distance"), value: formatDistance(stats.distance, locale), icon: "distance" },
    stats.gain != null && { label: t("route.gain"), value: `+${formatElevation(stats.gain, locale)}`, icon: "up" },
    stats.maxEle != null && { label: t("route.maxEle"), value: formatElevation(stats.maxEle, locale), icon: "peak" },
    stats.movingTime != null
      ? { label: t("route.moving"), value: formatDuration(stats.movingTime), icon: "time" }
      : stats.estimatedTime != null && { label: t("route.estimate"), value: formatDuration(stats.estimatedTime), icon: "time" },
  ].filter(Boolean) as { label: string; value: string; icon: string }[];

  const secondaryStats = [
    stats.loss != null && { label: t("route.loss"), value: `−${formatElevation(stats.loss, locale)}` },
    stats.minEle != null && { label: t("route.minEle"), value: formatElevation(stats.minEle, locale) },
    stats.totalTime != null && { label: t("route.total"), value: formatDuration(stats.totalTime) },
    stats.avgHr != null &&
      stats.maxHr != null && {
        label: t("route.heartRate"),
        value: `${t("route.heartRateValue", { avg: stats.avgHr, max: stats.maxHr })} ${t("route.bpm")}`,
      },
    stats.maxGrade != null && { label: t("route.maxGrade"), value: formatPercent(stats.maxGrade, locale) },
    stats.movingTime != null &&
      stats.estimatedTime != null && { label: t("route.estimate"), value: formatDuration(stats.estimatedTime), hint: t("route.estimateHint") },
  ].filter(Boolean) as { label: string; value: string; hint?: string }[];

  const dateRange =
    route.startTime != null && stats.totalTime != null
      ? formatDateRange(route.startTime, route.startTime + stats.totalTime, locale, timezone)
      : null;

  // Everything the map script needs.
  const payload = { ...route, lang, locale, timezone, photos, editable };

  const modes = [
    { id: "grade", label: t("route.mode.grade") },
    route.hasElevation && { id: "elevation", label: t("route.mode.elevation") },
    route.hasTime && { id: "speed", label: t("route.mode.speed") },
    route.hasTime && route.hasElevation && { id: "vspeed", label: t("route.mode.vspeed") },
    route.hasHeartRate && { id: "hr", label: t("route.mode.hr") },
  ].filter(Boolean) as { id: string; label: string }[];

  const toolButton = (attr: string, label: string, content: string, extra = "") =>
    `<button type="button" class="btn btn-sm bg-base-100/90 shadow backdrop-blur ${extra}" ${attr} title="${esc(label)}" aria-label="${esc(label)}">${content}</button>`;
  const svgIcon = (d: string, fill = false) =>
    `<svg class="h-4 w-4" viewBox="0 0 24 24" fill="${fill ? "currentColor" : "none"}" stroke="${fill ? "none" : "currentColor"}" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="${d}"></path></svg>`;

  const header = `
  <header class="flex flex-wrap items-end justify-between gap-3 px-5 pt-5 pb-4">
    <div>
      <p class="text-xs font-semibold uppercase tracking-[0.2em] text-primary">
        ${esc(t("route.kicker"))}${dateRange ? `<span class="text-base-content/60"> · ${esc(dateRange)}</span>` : ""}
      </p>
      <h2 class="text-2xl font-bold leading-tight">${esc(title ?? route.name ?? "")}</h2>
    </div>
    <div class="flex flex-wrap gap-2">
      <span class="badge badge-outline">${esc(t(`route.shape.${stats.shape}`))}</span>
      ${days.length > 1 ? `<span class="badge badge-outline">${esc(t("route.days", { n: days.length }))}</span>` : ""}
    </div>
  </header>`;

  const primary = `
  <dl class="grid grid-cols-2 gap-px bg-base-300 sm:grid-cols-4">
    ${primaryStats
      .map(
        (stat) => `
      <div class="bg-base-200 px-4 py-4 sm:px-5">
        <dt class="flex items-center gap-1.5 text-xs uppercase tracking-wide text-base-content/60">
          <svg class="h-3.5 w-3.5" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="${icons[stat.icon]}" /></svg>
          ${esc(stat.label)}
        </dt>
        <dd class="mt-1 whitespace-nowrap text-xl font-bold tabular-nums sm:text-2xl">${esc(stat.value)}</dd>
      </div>`,
      )
      .join("")}
  </dl>`;

  const stage = `
  <div class="route-map__stage relative">
    <div class="route-map__map isolate h-[26rem] w-full bg-base-300" data-map></div>

    <div class="route-map__3d absolute inset-0 z-[450] bg-base-300" data-map3d hidden></div>

    <div class="pointer-events-none absolute inset-x-3 top-3 z-[500] flex items-start justify-end gap-2">
      <div class="pointer-events-auto flex flex-wrap justify-end gap-1">
        ${
          photos.length
            ? toolButton(
                `data-photos aria-pressed="true"`,
                t("route.photosToggle"),
                `${svgIcon("M4 8h3l2-3h6l2 3h3v11H4zM12 17a4 4 0 1 0 0-8 4 4 0 0 0 0 8z")}<span class="hidden sm:inline">${esc(t("route.photos"))}</span><span class="badge badge-xs">${photos.length}</span>`,
                "gap-1.5 px-2.5",
              )
            : ""
        }
        ${editable ? toolButton(`data-mark aria-pressed="false"`, t("route.markHelp"), `${svgIcon("M12 21s-6-5.6-6-11a6 6 0 0 1 12 0c0 5.4-6 11-6 11zM12 7v6M9 10h6")}<span>${esc(t("route.mark"))}</span>`, "gap-1.5 px-2.5") : ""}
        ${route.hasElevation ? toolButton(`data-3d aria-pressed="false"`, t("route.view3dTitle"), `<span class="font-bold">3D</span>`, "px-2.5") : ""}
        ${toolButton("data-recenter", t("route.recenter"), `<svg class="h-4 w-4" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><circle cx="12" cy="12" r="3"></circle><path d="M12 2v3M12 19v3M2 12h3M19 12h3"></path></svg>`, "btn-square")}
        ${toolButton("data-fullscreen", t("route.fullscreen"), svgIcon("M8 3H3v5M16 3h5v5M8 21H3v-5M16 21h5v-5"), "btn-square")}
      </div>
    </div>

    <div class="pointer-events-none absolute left-1/2 top-14 z-[500] w-max max-w-[calc(100%-1.5rem)] -translate-x-1/2 rounded-full bg-primary px-4 py-1.5 text-center text-xs font-medium text-primary-content shadow-lg" data-mark-hint hidden>${esc(t("route.markHint"))}</div>

    <div class="route-map__readout pointer-events-none absolute bottom-[6.5rem] left-1/2 z-[500] w-max max-w-[calc(100%-1.5rem)] -translate-x-1/2 rounded-full bg-base-100/90 px-4 py-1.5 text-center text-xs sm:bottom-8 sm:text-sm font-medium tabular-nums shadow-lg backdrop-blur" data-readout hidden></div>

    <div class="absolute bottom-8 left-3 z-[500] rounded-lg bg-base-100/90 px-3 py-2 text-[0.7rem] shadow backdrop-blur" data-legend>
      <label class="flex items-center gap-1.5 text-base-content/70">
        <span class="sr-only sm:not-sr-only">${esc(t("route.colorBy"))}</span>
        <select class="select select-ghost select-xs h-6 min-h-0 pl-1 pr-7 font-semibold text-base-content" data-mode-select ${modes.length < 2 ? "disabled" : ""}>
          ${modes.map((mode) => `<option value="${mode.id}">${esc(mode.label)}</option>`).join("")}
        </select>
      </label>
      <div class="route-map__legend-bar mt-1 h-1.5 w-36 rounded-full" data-legend-bar></div>
      <div class="mt-1 flex justify-between tabular-nums text-base-content/70"><span data-legend-min></span><span data-legend-max></span></div>
    </div>
  </div>`;

  const profile = route.hasElevation
    ? `
  <div class="flex items-stretch gap-3 px-3 pt-3 sm:px-5">
    <button type="button" class="btn btn-circle btn-primary btn-sm mt-6 shrink-0" data-play title="${esc(t("route.play"))}" aria-label="${esc(t("route.play"))}">
      <svg class="route-map__icon-play h-4 w-4" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true"><path d="M8 5v14l11-7z"></path></svg>
      <svg class="route-map__icon-pause h-4 w-4" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true"><path d="M6 5h4v14H6zM14 5h4v14h-4z"></path></svg>
    </button>
    <figure class="min-w-0 flex-1" aria-label="${esc(t("route.profile"))}">
      <div class="route-map__profile h-44 w-full touch-none select-none" data-profile></div>
      <figcaption class="text-center text-[0.7rem] text-base-content/50" data-select-hint>${esc(t("route.selectHint"))}</figcaption>
    </figure>
  </div>
  <div class="mx-3 mt-2 flex flex-wrap items-center gap-x-4 gap-y-1 rounded-xl bg-base-100/70 px-4 py-2 text-sm sm:mx-5" data-selection hidden>
    <span class="text-xs font-semibold uppercase tracking-wide text-primary">${esc(t("route.selection"))}</span>
    <span class="flex flex-wrap gap-x-4 gap-y-1 tabular-nums" data-selection-stats></span>
    <span class="ml-auto flex gap-1">
      <button type="button" class="btn btn-ghost btn-xs" data-selection-zoom>${esc(t("route.zoomTo"))}</button>
      <button type="button" class="btn btn-circle btn-ghost btn-xs" data-selection-clear title="${esc(t("route.clearSelection"))}" aria-label="${esc(t("route.clearSelection"))}">✕</button>
    </span>
  </div>`
    : "";

  const secondary = secondaryStats.length
    ? `
  <dl class="flex flex-wrap gap-x-6 gap-y-2 px-5 pt-4 text-sm">
    ${secondaryStats
      .map(
        (stat) => `
      <div class="flex items-baseline gap-2"${stat.hint ? ` title="${esc(stat.hint)}"` : ""}>
        <dt class="text-base-content/60">${esc(stat.label)}${stat.hint ? `<span class="ml-0.5 cursor-help text-base-content/40">ⓘ</span>` : ""}</dt>
        <dd class="font-semibold tabular-nums">${esc(stat.value)}</dd>
      </div>`,
      )
      .join("")}
  </dl>`
    : "";

  const dayList =
    days.length > 1
      ? `
  <ol class="mt-5 flex flex-col gap-0 border-t border-base-300 px-5 py-4 sm:flex-row sm:items-stretch">
    ${days
      .map((day, i) => {
        const camp = i < days.length - 1 ? camps[i] : null;
        const summary = [
          formatDistance(day.distance, locale),
          day.gain != null && `+${formatElevation(day.gain, locale)}`,
          day.loss != null && `−${formatElevation(day.loss, locale)}`,
          `${formatDuration(day.movingTime)} ${t("route.moving").toLowerCase()}`,
        ].filter(Boolean);
        return `
      <li class="flex-1 rounded-xl bg-base-100/60 p-4">
        <p class="text-xs font-semibold uppercase tracking-wide text-primary">${esc(t("route.day", { n: i + 1 }))} · ${esc(formatDay(day.start, locale, timezone))}</p>
        <p class="mt-1 text-lg font-semibold tabular-nums">${esc(formatClock(day.start, locale, timezone))} → ${esc(formatClock(day.end, locale, timezone))}</p>
        <p class="mt-1 text-sm tabular-nums text-base-content/70">${esc(summary.join(" · "))}</p>
      </li>
      ${
        camp
          ? `
      <li class="flex items-center justify-center gap-2 px-3 py-3 text-sm text-base-content/70 sm:flex-col sm:py-0 sm:text-center">
        <svg class="h-6 w-6 text-[#7c3aed]" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M2 20h20M4 20 12 5l8 15M12 5l2-2M12 5l-2-2M9.5 20l2.5-5 2.5 5" /></svg>
        <span>
          <span class="font-semibold text-base-content">${esc(t("route.camp"))}</span>${camp.ele != null ? ` · ${esc(formatElevation(camp.ele, locale))}` : ""}
          <br class="hidden sm:block" />
          <span class="sm:text-xs"> ${esc(t("route.campRest", { time: formatDuration(days[i + 1].start - day.end) }))}</span>
        </span>
      </li>`
          : ""
      }`;
      })
      .join("")}
  </ol>`
      : "";

  const analysis = `
  <div class="mt-4 border-t border-base-300" data-analysis${editable ? " data-open" : ""}>
    <button type="button" class="flex w-full items-center justify-between gap-2 px-5 py-3 text-left text-sm font-semibold hover:bg-base-100/40" data-analysis-toggle aria-expanded="${editable}">
      <span class="flex items-center gap-2">
        <svg class="h-4 w-4 text-primary" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="M4 20V10M10 20V4M16 20v-7M22 20H2"></path></svg>
        ${esc(t("route.analysis"))}
      </span>
      <svg class="route-map__chevron h-4 w-4 transition-transform" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="m6 9 6 6 6-6"></path></svg>
    </button>
    <div class="px-3 pb-2 sm:px-5" data-analysis-body${editable ? "" : " hidden"}></div>
  </div>`;

  const footer = `
  <footer class="flex flex-wrap items-center justify-between gap-2 px-5 pb-5 pt-3 text-xs text-base-content/60">
    <span>${esc(t("route.zoomHint"))}</span>
    <span class="flex flex-wrap gap-2">
      ${
        kml
          ? `<a href="${esc(kml)}" download${kmlName ? `="${esc(kmlName)}"` : ""} class="btn btn-outline btn-sm" data-open-earth title="${esc(t("route.openRouteEarthHint"))}">
        <svg class="h-4 w-4" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" aria-hidden="true"><circle cx="12" cy="12" r="9"></circle><path d="M3 12h18M12 3c2.5 2.6 3.8 5.6 3.8 9s-1.3 6.4-3.8 9c-2.5-2.6-3.8-5.6-3.8-9S9.5 5.6 12 3z"></path></svg>
        ${esc(t("route.openRouteEarth"))}
      </a>`
          : ""
      }
      <a href="${esc(download)}" download${downloadName ? `="${esc(downloadName)}"` : ""} class="btn btn-outline btn-sm">
        <svg class="h-4 w-4" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true"><path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4M7 10l5 5 5-5M12 15V3"></path></svg>
        ${esc(t("route.download"))}
      </a>
    </span>
    <p class="w-full rounded-xl bg-primary/10 px-4 py-3 text-sm text-base-content" data-earth-help hidden></p>
  </footer>`;

  const data = `<script type="application/json" data-route-data>${JSON.stringify(payload).replace(/</g, "\\u003c")}</script>`;

  return `<section class="route-map not-prose my-8 overflow-hidden overflow-clip rounded-2xl bg-base-200 text-base-content shadow-xl" data-route-map${editable ? " data-editable" : ""}>${header}${primary}${stage}${profile}${secondary}${dayList}${analysis}${footer}${data}</section>`;
}
