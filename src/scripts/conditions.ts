// Live weather on the summits, straight from the visitor's browser to Open-Meteo: free, no key,
// and one request covers every summit. Answers are kept for 20 minutes in sessionStorage, because
// moving between pages shouldn't cost a stranger's API quota.

const API = "https://api.open-meteo.com/v1/forecast";
const TTL = 20 * 60_000;
/** The chart's point: the central Andes, where nearly all the log's summits are. */
const CHART_SUMMIT = "provincia";

interface Forecast {
  current: { time: string; temperature_2m: number; apparent_temperature: number; wind_speed_10m: number; wind_gusts_10m: number; weather_code: number };
  hourly: { time: string[]; freezing_level_height: (number | null)[] };
}
interface Labels {
  locale: string;
  error: string;
  updated: string;
  now: string;
  chartLevel: string;
  above: string;
  below: string;
  sky: Record<string, string>;
}

function skyGroup(code: number) {
  if (code === 0) return "clear";
  if (code === 1) return "mostly";
  if (code === 2) return "partly";
  if (code === 3) return "overcast";
  if (code === 45 || code === 48) return "fog";
  if (code >= 51 && code <= 57) return "drizzle";
  if ((code >= 71 && code <= 77) || code === 85 || code === 86) return "snow";
  if (code >= 95) return "storm";
  return "rain";
}

async function fetchForecasts(rows: HTMLElement[]): Promise<Forecast[]> {
  const list = (key: string) => rows.map((row) => row.dataset[key]).join(",");
  const query = new URLSearchParams({
    latitude: list("lat"),
    longitude: list("lon"),
    elevation: list("ele"),
    current: "temperature_2m,apparent_temperature,wind_speed_10m,wind_gusts_10m,weather_code",
    hourly: "freezing_level_height",
    forecast_days: "3",
    timezone: "auto",
  });
  const key = `conditions:${list("id")}`;
  try {
    const saved = JSON.parse(sessionStorage.getItem(key) ?? "null");
    if (saved && Date.now() - saved.at < TTL) return saved.data;
  } catch {
    // Storage may be blocked; the request below still works.
  }
  const response = await fetch(`${API}?${query}`);
  if (!response.ok) throw new Error(String(response.status));
  const body = await response.json();
  const data: Forecast[] = Array.isArray(body) ? body : [body];
  try {
    sessionStorage.setItem(key, JSON.stringify({ at: Date.now(), data }));
  } catch {
    // Same: not worth failing over.
  }
  return data;
}

const hourIndex = (forecast: Forecast) => Math.max(0, forecast.hourly.time.indexOf(`${forecast.current.time.slice(0, 13)}:00`));

export function initConditions() {
  for (const root of document.querySelectorAll<HTMLElement>("[data-conditions]:not([data-ready])")) {
    root.dataset.ready = "";
    void load(root);
  }
}

async function load(root: HTMLElement) {
  const labels: Labels = JSON.parse(root.dataset.labels!);
  const rows = [...root.querySelectorAll<HTMLElement>("[data-cond]")];
  const status = root.querySelector<HTMLElement>("[data-cond-status]")!;
  // Adding zero turns the -0 that Math.round gives for -0.3 into a plain 0.
  const round = (value: number) => (Math.round(value) + 0).toLocaleString(labels.locale);
  try {
    const forecasts = await fetchForecasts(rows);
    rows.forEach((row, i) => {
      const forecast = forecasts[i];
      const { current } = forecast;
      const set = (field: string, text: string) => {
        const cell = row.querySelector(`[data-f="${field}"]`);
        if (cell) cell.textContent = text;
      };
      set("sky", labels.sky[skyGroup(current.weather_code)]);
      set("temp", `${round(current.temperature_2m)} °C`);
      set("feels", `${round(current.apparent_temperature)} °C`);
      set("wind", `${round(current.wind_speed_10m)} / ${round(current.wind_gusts_10m)} km/h`);
      const level = forecast.hourly.freezing_level_height[hourIndex(forecast)];
      if (level == null) set("freezing", "—");
      else {
        const margin = level - Number(row.dataset.ele);
        set("freezing", `${round(level)} m (${round(Math.abs(margin))} m ${margin >= 0 ? labels.above : labels.below})`);
      }
    });
    const time = new Date(`${forecasts[0].current.time}:00Z`);
    status.textContent = `${labels.updated} ${new Intl.DateTimeFormat(labels.locale, { hour: "2-digit", minute: "2-digit", timeZone: "UTC" }).format(time)}. `;

    const host = root.querySelector<HTMLElement>("[data-cond-chart]");
    if (host) {
      const index = Math.max(0, rows.findIndex((row) => row.dataset.id === CHART_SUMMIT));
      // Only the summits this air mass is over: the Andes chart has nothing to say about Bavaria.
      const refs = rows
        .filter((row) => Math.abs(Number(row.dataset.lat) - Number(rows[index].dataset.lat)) < 3)
        .map((row) => ({ name: row.dataset.name!, ele: Number(row.dataset.ele) }));
      const draw = () => drawChart(host, forecasts[index], refs, labels);
      draw();
      new ResizeObserver(draw).observe(host);
    }
  } catch {
    status.textContent = `${labels.error} `;
    for (const cell of root.querySelectorAll("[data-f]")) cell.textContent = "—";
  }
}

/** The freezing level for the next three days, against the height of my summits. */
function drawChart(host: HTMLElement, forecast: Forecast, refs: { name: string; ele: number }[], labels: Labels) {
  const width = host.clientWidth;
  if (width === 0) return;
  const H = 280;
  const margin = { left: 46, right: width < 520 ? 92 : 128, top: 12, bottom: 26 };
  const times = forecast.hourly.time;
  const levels = forecast.hourly.freezing_level_height;
  const known = levels.filter((value): value is number => value != null);
  if (!known.length) {
    host.textContent = "—";
    return;
  }
  const low = Math.floor(Math.min(...known, 1500) / 500) * 500;
  const high = Math.ceil(Math.max(...known, 4200) / 500) * 500;
  const x = (i: number) => margin.left + (i / (times.length - 1)) * (width - margin.left - margin.right);
  const y = (value: number) => margin.top + (1 - (value - low) / (high - low)) * (H - margin.top - margin.bottom);
  const metres = (value: number) => Math.round(value).toLocaleString(labels.locale);
  const utc = (time: string) => new Date(`${time}:00Z`);
  const day = new Intl.DateTimeFormat(labels.locale, { weekday: "short", day: "numeric", timeZone: "UTC" });
  const stamp = new Intl.DateTimeFormat(labels.locale, { weekday: "short", hour: "2-digit", minute: "2-digit", timeZone: "UTC" });

  let grid = "";
  for (let value = Math.ceil(low / 1000) * 1000; value <= high; value += 1000) {
    grid += `<line class="cond-grid" x1="${margin.left}" x2="${width - margin.right}" y1="${y(value)}" y2="${y(value)}"/><text class="cond-tick" x="${margin.left - 6}" y="${y(value) + 4}" text-anchor="end">${metres(value)}</text>`;
  }
  times.forEach((time, i) => {
    if (!time.endsWith("T00:00")) return;
    grid += `<line class="cond-grid" x1="${x(i)}" x2="${x(i)}" y1="${margin.top}" y2="${H - margin.bottom}"/><text class="cond-tick" x="${x(i) + 4}" y="${H - 8}">${day.format(utc(time))}</text>`;
  });

  // Highest first, skipping any that would print over the one above.
  let lastY = -Infinity;
  let summits = "";
  for (const ref of [...refs].sort((a, b) => b.ele - a.ele)) {
    if (ref.ele < low || ref.ele > high || y(ref.ele) - lastY < 15) continue;
    lastY = y(ref.ele);
    summits += `<line class="cond-ref" x1="${margin.left}" x2="${width - margin.right}" y1="${lastY}" y2="${lastY}"/><text class="cond-tick" x="${width - margin.right + 6}" y="${lastY + 4}">${ref.name}</text>`;
  }

  const path = levels.map((value, i) => (value == null ? "" : `${i && levels[i - 1] != null ? "L" : "M"}${x(i).toFixed(1)} ${y(value).toFixed(1)}`)).join("");
  const now = hourIndex(forecast);
  host.innerHTML = `<svg width="${width}" height="${H}" viewBox="0 0 ${width} ${H}">
    ${grid}${summits}
    <line class="cond-now" x1="${x(now)}" x2="${x(now)}" y1="${margin.top}" y2="${H - margin.bottom}"/>
    <text class="cond-tick" x="${x(now) + 4}" y="${margin.top + 11}">${labels.now}</text>
    <path class="cond-line" d="${path}"/>
    <g data-hover hidden><line class="cond-cross" y1="${margin.top}" y2="${H - margin.bottom}"/><circle class="cond-dot" r="5"/></g>
    <rect x="${margin.left}" y="${margin.top}" width="${width - margin.left - margin.right}" height="${H - margin.top - margin.bottom}" fill="transparent" data-hit/>
  </svg><div class="cond-tip" data-tip-box hidden></div>`;

  const hover = host.querySelector<SVGGElement>("[data-hover]")!;
  const tip = host.querySelector<HTMLElement>("[data-tip-box]")!;
  const hit = host.querySelector<SVGRectElement>("[data-hit]")!;
  hit.addEventListener("pointermove", (event) => {
    const box = host.getBoundingClientRect();
    const ratio = (event.clientX - box.left - margin.left) / (width - margin.left - margin.right);
    const i = Math.max(0, Math.min(times.length - 1, Math.round(ratio * (times.length - 1))));
    const value = levels[i];
    if (value == null) return;
    hover.removeAttribute("hidden");
    hover.querySelector("line")!.setAttribute("x1", String(x(i)));
    hover.querySelector("line")!.setAttribute("x2", String(x(i)));
    hover.querySelector("circle")!.setAttribute("cx", String(x(i)));
    hover.querySelector("circle")!.setAttribute("cy", String(y(value)));
    tip.hidden = false;
    tip.textContent = `${stamp.format(utc(times[i]))} · ${labels.chartLevel} ${metres(value)} m`;
    tip.style.left = `${Math.min(x(i) + 10, width - 190)}px`;
    tip.style.top = `${Math.max(y(value) - 34, 0)}px`;
  });
  hit.addEventListener("pointerleave", () => {
    hover.setAttribute("hidden", "");
    tip.hidden = true;
  });
}
