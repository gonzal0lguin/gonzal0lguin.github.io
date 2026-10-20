// The route card's analysis panel: tabs with the marked points, a summary of effort and pace,
// climbs and descents, splits, grade and altitude distributions, stops, heart rate zones and a
// time planner. Rows are linked to the map: hovering highlights, clicking selects or flies there.
// Loaded by route-map.ts the first time the panel is opened.

import {
  altitudeBands,
  altitudeStep,
  cumulativeDin,
  detectClimbs,
  detectStops,
  effortKm,
  gradeBands,
  heartRateZones,
  oxygenFraction,
  paceFactor,
  splitDistance,
  splits,
  sunTimes,
  DIST,
  ELE,
  LAT,
  LON,
  TIME,
  type Band,
} from "../lib/route-analysis";
import {
  formatClock,
  formatDay,
  formatDistance,
  formatDuration,
  formatElevation,
  formatPace,
  formatPercent,
  formatSpeed,
  fromLocal,
  googleEarthUrl,
  googleMaps3dUrl,
  localParts,
} from "../lib/route-format";
import { WAYPOINT_KINDS, type WaypointKind } from "../lib/waypoint-kinds";
import { esc, gradeColor, type RouteMapApi } from "./route-map";

type Tab = "points" | "summary" | "climbs" | "splits" | "grade" | "altitude" | "stops" | "hr" | "plan";

const HR_COLORS = ["#2563eb", "#22c55e", "#facc15", "#f97316", "#dc2626"];

export function createPanels(container: HTMLElement, api: RouteMapApi) {
  const { data, t } = api;
  const { points, locale, timezone, stats } = data;
  const total = points[points.length - 1][DIST];
  const timed = data.hasTime;

  // Computed once; the route doesn't change.
  const climbs = data.hasElevation ? detectClimbs(points, stats.gain ?? 0) : [];
  const stops = detectStops(points, data.startTime);
  const factor = paceFactor(data);
  const din = data.hasElevation ? cumulativeDin(points) : null;

  const tabs = (
    [
      ["points", t("route.tab.points")],
      ["summary", t("route.tab.summary")],
      data.hasElevation && ["climbs", t("route.tab.climbs")],
      ["splits", t("route.tab.splits")],
      data.hasElevation && ["grade", t("route.tab.grade")],
      data.hasElevation && ["altitude", t("route.tab.altitude")],
      timed && ["stops", t("route.tab.stops")],
      data.hasHeartRate && ["hr", t("route.tab.hr")],
      data.hasElevation && ["plan", t("route.tab.plan")],
    ].filter(Boolean) as [Tab, string][]
  );
  let active: Tab = data.editable || data.waypoints.length ? "points" : "summary";
  let maxHr = stats.maxHr ?? 0;
  const plan = {
    start: defaultStart(),
    factor: factor != null ? clamp(round(factor, 0.05), 0.5, 1.6) : 1,
    breaks: 10,
  };

  container.innerHTML = `
    <div role="tablist" class="route-tabs tabs tabs-bordered flex overflow-x-auto" aria-label="${esc(t("route.analysis"))}">
      ${tabs.map(([id, label]) => `<button type="button" role="tab" class="tab shrink-0 whitespace-nowrap" data-tab="${id}">${esc(label)}</button>`).join("")}
    </div>
    <div class="pt-4" role="tabpanel" data-panel></div>`;
  const panel = container.querySelector<HTMLElement>("[data-panel]")!;

  const render = () => {
    container.querySelectorAll<HTMLButtonElement>("[data-tab]").forEach((button) => {
      const on = button.dataset.tab === active;
      button.classList.toggle("tab-active", on);
      button.setAttribute("aria-selected", String(on));
    });
    panel.innerHTML = RENDER[active]();
  };

  container.addEventListener("click", (event) => {
    const target = event.target as HTMLElement;
    const tab = target.closest<HTMLElement>("[data-tab]");
    if (tab) {
      active = tab.dataset.tab as Tab;
      api.clearHighlight();
      return render();
    }
    const el = target.closest<HTMLElement>("[data-select], [data-focus], [data-wp], [data-edit-wp], [data-mark-stop]");
    if (!el) return;
    if (el.dataset.select) {
      const [a, b] = el.dataset.select.split(",").map(Number);
      api.select(a, b, true);
    } else if (el.dataset.focus) api.focusPoint(Number(el.dataset.focus));
    else if (el.dataset.wp) api.focusWaypoint(Number(el.dataset.wp));
    else if (el.dataset.editWp) api.editWaypoint(Number(el.dataset.editWp));
    else if (el.dataset.markStop) {
      const stop = stops[Number(el.dataset.markStop)];
      api.addWaypointAt(stop.index, "note", `${t("route.stop")} · ${formatDuration(stop.duration)}`);
    }
  });
  // Hover: highlight the section, or move the cursor to the point.
  container.addEventListener("pointerover", (event) => {
    const el = (event.target as HTMLElement).closest<HTMLElement>("[data-select], [data-focus], [data-cursor-at]");
    if (!el) return;
    if (el.dataset.select) {
      const [a, b] = el.dataset.select.split(",").map(Number);
      api.highlight(a, b);
    } else api.setCursor(Number(el.dataset.focus ?? el.dataset.cursorAt));
  });
  container.addEventListener("pointerleave", () => {
    api.clearHighlight();
    api.hideCursor();
  });
  container.addEventListener("input", (event) => {
    const input = event.target as HTMLInputElement;
    if (input.name === "max-hr") {
      maxHr = Number(input.value) || maxHr;
      panel.querySelector("[data-hr-zones]")!.innerHTML = hrZonesHtml();
    } else if (input.name === "plan-start" || input.name === "plan-factor" || input.name === "plan-breaks") {
      if (input.name === "plan-start" && input.value) plan.start = input.value;
      if (input.name === "plan-factor") plan.factor = Number(input.value);
      if (input.name === "plan-breaks") plan.breaks = Number(input.value);
      panel.querySelector("[data-plan-output]")!.innerHTML = planOutputHtml();
      panel.querySelector("[data-plan-factor]")!.textContent = t("route.plan.factorValue", { x: plan.factor.toLocaleString(locale, { minimumFractionDigits: 2 }) });
      panel.querySelector("[data-plan-breaks]")!.textContent = t("route.plan.breaksValue", { n: plan.breaks });
    }
  });

  // --- Helpers

  const km = (meters: number) => (meters / 1000).toLocaleString(locale, { maximumFractionDigits: 1 });
  const clockAt = (i: number) => (data.startTime != null && points[i][TIME] != null ? formatClock(data.startTime + points[i][TIME]!, locale, timezone) : null);
  const tile = (label: string, value: string, hint?: string) => `
    <div class="rounded-xl bg-base-100/60 px-4 py-3"${hint ? ` title="${esc(hint)}"` : ""}>
      <dt class="text-xs text-base-content/60">${esc(label)}${hint ? ` <span class="cursor-help text-base-content/40">ⓘ</span>` : ""}</dt>
      <dd class="mt-0.5 text-lg font-semibold tabular-nums">${esc(value)}</dd>
    </div>`;
  const empty = (text: string) => `<p class="rounded-xl bg-base-100/60 px-4 py-6 text-center text-sm text-base-content/60">${esc(text)}</p>`;
  const ENDS = {
    start: { color: "#16a34a", icon: '<path d="M5 21V4m0 0h11l-2 4 2 4H5" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linejoin="round"/>' },
    finish: { color: "#111827", icon: '<path d="M5 21V4h14v9H5" fill="none" stroke="currentColor" stroke-width="2.4" stroke-linejoin="round"/>' },
    highPoint: { color: "#78716c", icon: WAYPOINT_KINDS.summit.icon },
  };
  const icon = (kind: WaypointKind | keyof typeof ENDS, size = 22) => {
    const { color, icon: svg } = kind in ENDS ? ENDS[kind as keyof typeof ENDS] : WAYPOINT_KINDS[kind as WaypointKind];
    return `<span class="route-kind-dot" style="--kind:${color};width:${size}px;height:${size}px"><svg viewBox="0 0 24 24" width="${size * 0.6}" height="${size * 0.6}">${svg}</svg></span>`;
  };
  const links = (lat: number, lon: number, ele: number | null | undefined) => `
    <a class="link link-hover text-xs" href="${googleEarthUrl(lat, lon, ele)}" target="_blank" rel="noopener">${esc(t("route.openEarth"))} ↗</a>
    <a class="link link-hover text-xs" href="${googleMaps3dUrl(lat, lon, ele)}" target="_blank" rel="noopener">${esc(t("route.openMaps"))} ↗</a>`;

  /**
   * A horizontal bar list (one series, so no legend): label · bar · value at the tip.
   * Bars are thin with a rounded data end; the tooltip carries the secondary value.
   */
  const bars = (rows: { label: string; value: number; text: string; color: string; title: string; select?: string }[]) => {
    const max = Math.max(...rows.map((r) => r.value), 1);
    return `<ul class="route-bars">${rows
      .map(
        (r) => `
        <li title="${esc(r.title)}"${r.select ? ` data-select="${r.select}"` : ""}>
          <span class="route-bars__label">${esc(r.label)}</span>
          <span class="route-bars__track"><span class="route-bars__bar" style="--v:${(r.value / max).toFixed(4)};background:${r.color}"></span><span class="route-bars__value">${esc(r.text)}</span></span>
        </li>`,
      )
      .join("")}</ul>`;
  };

  // --- Tabs

  const RENDER: Record<Tab, () => string> = {
    points: () => {
      const rows: string[] = [];
      const fixed = [
        { kind: "start" as const, label: data.stats.shape === "one-way" ? t("route.start") : t("route.startFinish"), i: 0 },
        ...highPoints(),
        ...data.camps.map((camp) => ({ kind: "camp" as WaypointKind, label: t("route.camp"), i: nearest(camp.lat, camp.lon) })),
        ...(data.stats.shape === "one-way" ? [{ kind: "finish" as const, label: t("route.finish"), i: points.length - 1 }] : []),
      ];
      const entries = [
        ...fixed.map((f) => ({ d: points[f.i][DIST], html: pointRow(icon(f.kind), f.label, t("route.points.auto"), f.i, points[f.i][LAT], points[f.i][LON], points[f.i][ELE], `data-focus="${f.i}"`, "") })),
        ...api.waypoints().map((w, k) => {
          const i = api.waypointIndex(w);
          const away = i == null ? ` · ${t("route.offRoute", { d: formatDistance(api.distanceToRoute(w), locale) })}` : "";
          return {
            // Off the route: listed next to the closest point of the route.
            d: points[i ?? nearest(w.lat, w.lon)][DIST],
            html: pointRow(
              icon(w.kind),
              w.name,
              t(`route.kind.${w.kind}`) + away + (w.desc ? ` — ${w.desc}` : ""),
              i,
              w.lat,
              w.lon,
              w.ele ?? (i != null ? points[i][ELE] : null),
              `data-wp="${k}"${i != null ? ` data-cursor-at="${i}"` : ""}`,
              data.editable ? `<button type="button" class="btn btn-ghost btn-xs" data-edit-wp="${k}">${esc(t("route.edit"))}</button>` : "",
            ),
          };
        }),
      ].sort((a, b) => a.d - b.d);
      rows.push(...entries.map((e) => e.html));
      const hint = data.editable && api.waypoints().length === 0 ? `<p class="mb-3 rounded-xl bg-primary/10 px-4 py-3 text-sm">${esc(t("route.points.hint"))}</p>` : "";
      return `${hint}<ul class="flex flex-col gap-1.5">${rows.join("")}</ul>`;
    },

    summary: () => {
      const climbsUp = climbs.filter((c) => c.direction === "up" && c.movingTime);
      const climbsDown = climbs.filter((c) => c.direction === "down" && c.movingTime);
      const rate = (list: typeof climbs, key: "gain" | "loss") => {
        const time = list.reduce((acc, c) => acc + (c.movingTime ?? 0), 0);
        return time > 0 ? (list.reduce((acc, c) => acc + c[key], 0) / time) * 3600 : null;
      };
      const up = rate(climbsUp, "gain");
      const down = rate(climbsDown, "loss");
      const bands = gradeBands(points);
      const share = (from: number, to: number) => bands.filter((b) => b.from >= from && b.to <= to).reduce((acc, b) => acc + b.distance, 0) / Math.max(1, total);
      const sun = data.startTime != null ? sunTimes(data.startTime, points[0][LAT], points[0][LON]) : null;
      const stopped = stats.totalTime != null && stats.movingTime != null ? stats.totalTime - stats.movingTime - campTime() : null;
      const tiles = [
        tile(t("route.effortKm"), `${effortKm(stats.distance, stats.gain ?? 0, stats.loss ?? 0).toLocaleString(locale, { maximumFractionDigits: 1 })} km`, t("route.effortKmHint")),
        stats.movingTime && tile(t("route.avgSpeed"), formatSpeed(stats.distance / stats.movingTime, locale)),
        up != null && tile(t("route.climbRate"), `${Math.round(up).toLocaleString(locale)} m/h`, t("route.climbRateHint")),
        down != null && tile(t("route.descentRate"), `${Math.round(down).toLocaleString(locale)} m/h`),
        factor != null && tile(t("route.paceVsDin"), t("route.paceVsDinValue", { x: factor.toLocaleString(locale, { maximumFractionDigits: 2 }) }), t("route.paceVsDinHint")),
        stopped != null && stopped > 0 && tile(t("route.stopped"), formatDuration(stopped), t("route.stoppedHint")),
        data.hasElevation &&
          tile(
            t("route.terrainShare"),
            `${Math.round(share(3, Infinity) * 100)} % ↗ · ${Math.round(share(-3, 3) * 100)} % → · ${Math.round(share(-Infinity, -3) * 100)} % ↘`,
            t("route.terrainShareHint"),
          ),
        stats.maxEle != null && tile(t("route.oxygen"), t("route.oxygenValue", { pct: formatPercent(oxygenFraction(stats.maxEle), locale) }), t("route.oxygenHint")),
        sun && tile(t("route.daylight"), `☀ ${formatClock(sun.sunrise, locale, timezone)} – ${formatClock(sun.sunset, locale, timezone)}`, t("route.daylightHint")),
      ].filter(Boolean);
      return `<dl class="grid grid-cols-2 gap-2 sm:grid-cols-3">${tiles.join("")}</dl>`;
    },

    climbs: () => {
      if (!climbs.length) return empty(t("route.climbs.empty"));
      let up = 0, down = 0;
      return `<ul class="flex flex-col gap-1.5">${climbs
        .map((c) => {
          const n = c.direction === "up" ? ++up : ++down;
          const change = c.direction === "up" ? `+${formatElevation(c.gain, locale)}` : `−${formatElevation(c.loss, locale)}`;
          const facts = [
            formatDistance(c.distance, locale),
            c.avgGrade != null && `${t("route.avgGrade")} ${formatPercent(Math.abs(c.avgGrade), locale)}`,
            c.maxGrade != null && `${t("route.maxGradeShort")} ${formatPercent(Math.abs(c.maxGrade), locale)}`,
            c.movingTime && formatDuration(c.movingTime),
            c.vam && `${Math.round(c.vam).toLocaleString(locale)} m/h`,
            c.avgHr && `♥ ${c.avgHr}`,
          ].filter(Boolean);
          return `
          <li class="route-row" data-select="${c.from},${c.to}" tabindex="0">
            <span class="route-row__icon" style="background:${gradeColor(Math.abs(c.avgGrade ?? 0))}">${c.direction === "up" ? "↗" : "↘"}</span>
            <span class="min-w-0 flex-1">
              <span class="block font-semibold">${esc(t(c.direction === "up" ? "route.climb.up" : "route.climb.down", { n }))} <span class="font-normal text-base-content/60">· km ${km(points[c.from][DIST])}–${km(points[c.to][DIST])}</span></span>
              <span class="block text-xs tabular-nums text-base-content/70">${esc(facts.join(" · "))}</span>
            </span>
            <span class="text-lg font-bold tabular-nums">${esc(change)}</span>
          </li>`;
        })
        .join("")}</ul>`;
    },

    splits: () => {
      const every = splitDistance(total);
      const list = splits(points, every);
      const maxTime = Math.max(...list.map((s) => s.movingTime ?? 0), 1);
      return `
      <div class="overflow-x-auto">
        <table class="table table-sm route-table">
          <thead><tr>
            <th>${esc(t("route.col.km"))}</th>
            ${timed ? `<th>${esc(t("route.col.time"))}</th><th>${esc(t("route.col.pace"))}</th>` : ""}
            <th>${esc(t("route.col.gain"))}</th><th>${esc(t("route.col.grade"))}</th>
            ${timed && data.hasElevation ? `<th>${esc(t("route.col.vam"))}</th>` : ""}
            ${data.hasHeartRate ? `<th>${esc(t("route.col.hr"))}</th>` : ""}
          </tr></thead>
          <tbody>${list
            .map(
              (s) => `
            <tr data-select="${s.from},${s.to}">
              <td class="font-semibold">${km(points[s.from][DIST])}–${km(points[s.to][DIST])}</td>
              ${
                timed
                  ? `<td><span class="route-inline-bar" style="--w:${(((s.movingTime ?? 0) / maxTime) * 100).toFixed(0)}%"></span>${s.movingTime ? formatDuration(s.movingTime) : "—"}</td>
                     <td>${s.movingTime && s.distance > 50 ? esc(formatPace((s.movingTime / s.distance) * 1000)) : "—"}</td>`
                  : ""
              }
              <td>+${s.gain} / −${s.loss}</td>
              <td><span class="route-grade-dot" style="background:${gradeColor(s.avgGrade ?? 0)}"></span>${s.avgGrade != null ? formatPercent(s.avgGrade, locale) : "—"}</td>
              ${timed && data.hasElevation ? `<td>${s.vam ? `${s.gain >= s.loss ? "↑" : "↓"} ${Math.round(s.vam).toLocaleString(locale)}` : "—"}</td>` : ""}
              ${data.hasHeartRate ? `<td>${s.avgHr ?? "—"}</td>` : ""}
            </tr>`,
            )
            .join("")}</tbody>
        </table>
      </div>
      <p class="mt-2 text-xs text-base-content/60">${esc(t("route.splitsNote", { d: formatDistance(every, locale, 0) }))}</p>`;
    },

    grade: () => {
      const bands = gradeBands(points).reverse(); // steepest climbs on top
      const label = (b: Band) =>
        b.from === -3 ? `${t("route.flat")} (±3 %)` : b.to === Infinity ? `↗ > ${b.from} %` : b.from === -Infinity ? `↘ > ${-b.to} %` : b.from >= 3 ? `↗ ${b.from}–${b.to} %` : `↘ ${-b.to}–${-b.from} %`;
      const mid = (b: Band) => (Number.isFinite(b.from) && Number.isFinite(b.to) ? (b.from + b.to) / 2 : b.from === -Infinity ? b.to - 5 : b.from + 5) / 100;
      return `
        <p class="mb-3 text-sm text-base-content/70">${esc(t("route.grade.intro"))}</p>
        ${bars(
          bands.map((b) => ({
            label: label(b),
            value: b.distance,
            text: `${formatDistance(b.distance, locale)} · ${Math.round((b.distance / total) * 100)} %`,
            color: gradeColor(mid(b)),
            title: [label(b), formatDistance(b.distance, locale), b.time != null && `${formatDuration(b.time)} ${t("route.moving").toLowerCase()}`].filter(Boolean).join(" · "),
          })),
        )}`;
    },

    altitude: () => {
      const step = altitudeStep(stats.minEle ?? 0, stats.maxEle ?? 0);
      const bands = altitudeBands(points, step);
      const useTime = timed;
      return `
        <p class="mb-3 text-sm text-base-content/70">${esc(t(useTime ? "route.altitude.introTime" : "route.altitude.introDistance"))}</p>
        ${bars(
          bands.map((b) => ({
            label: `${b.from.toLocaleString(locale)}–${b.to.toLocaleString(locale)} m`,
            value: useTime ? (b.time ?? 0) : b.distance,
            text: useTime ? formatDuration(b.time ?? 0) : formatDistance(b.distance, locale),
            color: "oklch(var(--p))",
            title: [formatDistance(b.distance, locale), b.time != null && formatDuration(b.time), `${t("route.oxygen")}: ${formatPercent(oxygenFraction((b.from + b.to) / 2), locale)}`].filter(Boolean).join(" · "),
          })),
        )}`;
    },

    stops: () => {
      if (!stops.length) return empty(t("route.stops.empty"));
      return `<ul class="flex flex-col gap-1.5">${stops
        .map(
          (s, k) => `
          <li class="route-row" data-focus="${s.index}" tabindex="0">
            <span class="route-row__icon bg-base-300 text-base-content"><svg viewBox="0 0 24 24" width="14" height="14" aria-hidden="true"><path d="M7 5h3.5v14H7zM13.5 5H17v14h-3.5z" fill="currentColor"/></svg></span>
            <span class="min-w-0 flex-1">
              <span class="block font-semibold">${esc(formatDuration(s.duration))} <span class="font-normal text-base-content/60">· ${esc(formatClock(s.start, locale, timezone))}</span></span>
              <span class="block text-xs tabular-nums text-base-content/70">km ${km(s.distance)}${s.ele != null ? ` · ${formatElevation(s.ele, locale)}` : ""}</span>
            </span>
            ${data.editable ? `<button type="button" class="btn btn-ghost btn-xs" data-mark-stop="${k}">${esc(t("route.mark"))}</button>` : ""}
          </li>`,
        )
        .join("")}</ul>`;
    },

    hr: () => `
      <label class="mb-3 flex flex-wrap items-center gap-2 text-sm">
        <span>${esc(t("route.hr.max"))}</span>
        <input type="number" name="max-hr" min="120" max="230" value="${maxHr}" class="input input-bordered input-sm w-24 tabular-nums" />
        <span class="text-xs text-base-content/60">${esc(t("route.hr.maxHint"))}</span>
      </label>
      <div data-hr-zones>${hrZonesHtml()}</div>`,

    plan: () => `
      <p class="mb-3 text-sm text-base-content/70">${esc(t("route.plan.intro"))}</p>
      <div class="mb-4 grid gap-3 sm:grid-cols-3">
        <label class="form-control">
          <span class="label-text mb-1 text-xs">${esc(t("route.plan.start"))}</span>
          <input type="datetime-local" name="plan-start" value="${plan.start}" class="input input-bordered input-sm" />
        </label>
        <label class="form-control">
          <span class="label-text mb-1 flex justify-between text-xs"><span>${esc(t("route.plan.factor"))}</span><b data-plan-factor>${esc(t("route.plan.factorValue", { x: plan.factor.toLocaleString(locale, { minimumFractionDigits: 2 }) }))}</b></span>
          <input type="range" name="plan-factor" min="0.4" max="1.6" step="0.05" value="${plan.factor}" class="range range-primary range-xs mt-2" />
          <span class="mt-1 text-[0.7rem] text-base-content/60">${esc(factor != null ? t("route.plan.factorYours", { x: factor.toLocaleString(locale, { maximumFractionDigits: 2 }) }) : t("route.plan.factorHint"))}</span>
        </label>
        <label class="form-control">
          <span class="label-text mb-1 flex justify-between text-xs"><span>${esc(t("route.plan.breaks"))}</span><b data-plan-breaks>${esc(t("route.plan.breaksValue", { n: plan.breaks }))}</b></span>
          <input type="range" name="plan-breaks" min="0" max="20" step="1" value="${plan.breaks}" class="range range-primary range-xs mt-2" />
        </label>
      </div>
      <div data-plan-output>${planOutputHtml()}</div>`,
  };

  function pointRow(iconHtml: string, title: string, subtitle: string, i: number | null, lat: number, lon: number, ele: number | null | undefined, attrs: string, actions: string) {
    const facts = [i != null && `km ${km(points[i][DIST])}`, ele != null && formatElevation(ele, locale), i != null && clockAt(i)].filter(Boolean);
    return `
      <li class="route-row" ${attrs} tabindex="0">
        ${iconHtml}
        <span class="min-w-0 flex-1">
          <span class="block truncate font-semibold">${esc(title)} <span class="font-normal text-base-content/60">· ${esc(subtitle)}</span></span>
          <span class="flex flex-wrap gap-x-3 text-xs tabular-nums text-base-content/70"><span>${esc(facts.join(" · "))}</span>${links(lat, lon, ele)}</span>
        </span>
        ${actions}
      </li>`;
  }

  function hrZonesHtml() {
    const { zones } = heartRateZones(points, maxHr);
    const sum = zones.reduce((acc, z) => acc + z.time, 0) || 1;
    return bars(
      zones
        .slice()
        .reverse()
        .map((z) => ({
          label: `Z${z.zone} · ${t(`route.hr.z${z.zone}` as "route.hr.z1")}`,
          value: z.time,
          text: `${formatDuration(z.time)} · ${Math.round((z.time / sum) * 100)} %`,
          color: HR_COLORS[z.zone - 1],
          title: `${z.from}–${z.to} ${t("route.bpm")}`,
        })),
    );
  }

  function planOutputHtml() {
    if (!din) return "";
    const [date, clock] = plan.start.split("T");
    const [y, mo, d] = date.split("-").map(Number);
    const [h, mi] = (clock ?? "07:00").split(":").map(Number);
    const start = fromLocal(y, mo, d, h, mi, 0, timezone);
    const scale = plan.factor * (1 + plan.breaks / 60);
    const eta = (i: number) => start + din[i] * scale;
    const end = eta(points.length - 1);
    const sun = sunTimes(start, points[0][LAT], points[0][LON]);
    const dark = (time: number) => sun != null && (time > sun.sunset || time < sun.sunrise);
    const dayLabel = (time: number) => (localParts(time, timezone).day !== d ? `${formatDay(time, locale, timezone)} ` : "");

    const checkpoints = [
      { name: t("route.start"), kind: "start" as WaypointKind | "start" | "finish" | "highPoint", i: 0 },
      ...api.waypoints().flatMap((w) => {
        const i = api.waypointIndex(w);
        return i != null ? [{ name: w.name, kind: w.kind, i }] : [];
      }),
      ...highPoints().map((h) => ({ name: h.label, kind: h.kind, i: h.i })),
      ...climbs.filter((c) => c.direction === "up").map((c, k) => ({ name: t("route.plan.climbTop", { n: k + 1 }), kind: "crux" as WaypointKind, i: c.to })),
      { name: t("route.finish"), kind: "finish" as const, i: points.length - 1 },
    ]
      .sort((a, b) => points[a.i][DIST] - points[b.i][DIST])
      .filter((c, k, all) => k === 0 || Math.abs(points[c.i][DIST] - points[all[k - 1].i][DIST]) > 150 || c.i === points.length - 1);

    const notes: string[] = [];
    if (sun) {
      notes.push(t("route.plan.sun", { rise: formatClock(sun.sunrise, locale, timezone), set: formatClock(sun.sunset, locale, timezone) }));
      if (end > sun.sunset) notes.push(`⚠ ${t("route.plan.late", { start: formatClock(sun.sunset - (end - start), locale, timezone) })}`);
      if (data.summit && data.stats.shape !== "one-way") {
        const back = (din[points.length - 1] - din[nearest(data.summit.lat, data.summit.lon)]) * scale;
        notes.push(t("route.plan.turnaround", { time: formatClock(sun.sunset - back, locale, timezone) }));
      }
    }

    return `
      <dl class="mb-3 grid grid-cols-2 gap-2 sm:grid-cols-3">
        ${tile(t("route.plan.total"), formatDuration(end - start))}
        ${tile(t("route.plan.arrival"), `${dayLabel(end)}${formatClock(end, locale, timezone)}`)}
        ${tile(t("route.effortKm"), `${effortKm(stats.distance, stats.gain ?? 0, stats.loss ?? 0).toLocaleString(locale, { maximumFractionDigits: 1 })} km`, t("route.effortKmHint"))}
      </dl>
      <ul class="flex flex-col gap-1.5">${checkpoints
        .map(
          (c) => `
          <li class="route-row" data-focus="${c.i}" tabindex="0">
            ${icon(c.kind)}
            <span class="min-w-0 flex-1">
              <span class="block truncate font-semibold">${esc(c.name)}</span>
              <span class="block text-xs tabular-nums text-base-content/70">km ${km(points[c.i][DIST])}${points[c.i][ELE] != null ? ` · ${formatElevation(points[c.i][ELE]!, locale)}` : ""}</span>
            </span>
            <span class="text-right tabular-nums"><span class="font-semibold">${esc(dayLabel(eta(c.i)) + formatClock(eta(c.i), locale, timezone))}</span>${dark(eta(c.i)) ? `<span class="block text-xs text-warning">☾ ${esc(t("route.plan.dark"))}</span>` : ""}</span>
          </li>`,
        )
        .join("")}</ul>
      ${notes.length ? `<ul class="mt-3 space-y-1 text-sm text-base-content/80">${notes.map((n) => `<li>${esc(n)}</li>`).join("")}</ul>` : ""}
      <p class="mt-3 text-xs text-base-content/60">${esc(t("route.plan.method"))}</p>`;
  }

  function campTime() {
    return data.days.slice(1).reduce((acc, day, k) => acc + (day.start - data.days[k].end), 0);
  }

  /** The route's high point as the map shows it: none when a marked summit sits on it. */
  function highPoints() {
    const top = api.highPoint();
    return top ? [{ kind: top.kind as WaypointKind | "highPoint", label: top.label, i: top.i }] : [];
  }

  function nearest(lat: number, lon: number) {
    let best = 0;
    let bestDistance = Infinity;
    points.forEach((p, i) => {
      const d = (p[LAT] - lat) ** 2 + ((p[LON] - lon) * Math.cos((lat * Math.PI) / 180)) ** 2;
      if (d < bestDistance) (bestDistance = d), (best = i);
    });
    return best;
  }

  /** The route's own start for recorded tracks; otherwise tomorrow at 07:00. Local to `timezone`. */
  function defaultStart() {
    const at = data.startTime ?? Date.now() / 1000 + 86400;
    const p = localParts(at, timezone);
    const pad = (n: number) => String(n).padStart(2, "0");
    return `${p.year}-${pad(p.month)}-${pad(p.day)}T${data.startTime != null ? `${pad(p.hour)}:${pad(p.minute)}` : "07:00"}`;
  }

  render();
  return {
    refresh: () => {
      if (active === "points" || active === "plan") render();
    },
  };
}

function clamp(value: number, min: number, max: number) {
  return Math.min(max, Math.max(min, value));
}

function round(value: number, step: number) {
  return Math.round(value / step) * step;
}
