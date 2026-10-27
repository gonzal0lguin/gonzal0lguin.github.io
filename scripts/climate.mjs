// The weather on the hill, 1991 to today, for the charts in the post: what a normal year
// looks like, how 2025 — our first planting year — compared, and when the water runs out.
//
// Source: ERA5 reanalysis through the Open-Meteo historical API (free, no key). ERA5 is a
// global model on a ~28 km grid, not a rain gauge on the hill: it is good at "drier or wetter
// than usual" and at the shape of the year, and known to run wet in the foothills of central
// Chile, so the post leans on its comparisons, not its absolute millimetres. Where the
// Dirección Meteorológica de Chile published station numbers, the post quotes those instead.
//
// Run with: npm run climate   (writes src/data/climate.json)

import { writeFileSync } from "node:fs";

const PLACE = { lat: -32.8589, lon: -70.6594 };
const FIRST = "1991-01-01";
const NORMAL = [1991, 2020];

const until = new Date(Date.now() - 3 * 86_400_000).toISOString().slice(0, 10);
const url =
  `https://archive-api.open-meteo.com/v1/archive?latitude=${PLACE.lat}&longitude=${PLACE.lon}` +
  `&start_date=${FIRST}&end_date=${until}&models=era5&timezone=America%2FSantiago` +
  `&daily=precipitation_sum,temperature_2m_max,temperature_2m_min,et0_fao_evapotranspiration`;

const response = await fetch(url);
if (!response.ok) throw new Error(`Open-Meteo: HTTP ${response.status}`);
const { daily, elevation } = await response.json();
const days = daily.time.map((date, i) => ({
  date,
  year: Number(date.slice(0, 4)),
  month: Number(date.slice(5, 7)),
  rain: daily.precipitation_sum[i],
  tmax: daily.temperature_2m_max[i],
  tmin: daily.temperature_2m_min[i],
  et0: daily.et0_fao_evapotranspiration[i],
}));

const round = (value, digits = 1) => (value === null || value === undefined ? null : Number(value.toFixed(digits)));
const inNormal = (d) => d.year >= NORMAL[0] && d.year <= NORMAL[1];
const normalYears = NORMAL[1] - NORMAL[0] + 1;

// Monthly normals: totals for rain and evaporation, means for temperature.
const months = Array.from({ length: 12 }, (_, m) => {
  const all = days.filter((d) => inNormal(d) && d.month === m + 1);
  const mean = (key) => all.reduce((t, d) => t + (d[key] ?? 0), 0) / all.length;
  return {
    month: m + 1,
    rain: round(all.reduce((t, d) => t + (d.rain ?? 0), 0) / normalYears),
    et0: round(all.reduce((t, d) => t + (d.et0 ?? 0), 0) / normalYears),
    tmax: round(mean("tmax")),
    tmin: round(mean("tmin")),
  };
});

// Every year's rain, and how warm each October was.
const years = [];
for (let year = Number(FIRST.slice(0, 4)); year <= Number(until.slice(0, 4)); year++) {
  const all = days.filter((d) => d.year === year);
  const october = all.filter((d) => d.month === 10 && d.tmax !== null);
  years.push({
    year,
    rain: round(all.reduce((t, d) => t + (d.rain ?? 0), 0), 0),
    complete: all.length >= 365,
    octoberTmax: october.length === 31 ? round(october.reduce((t, d) => t + d.tmax, 0) / 31) : null,
  });
}

// Each year's monthly rain.
const monthly = (year) =>
  Array.from({ length: 12 }, (_, m) => {
    const all = days.filter((d) => d.year === year && d.month === m + 1 && d.rain !== null);
    return all.length ? round(all.reduce((t, d) => t + d.rain, 0)) : null;
  });

// Daily normal maximum temperature: the 1991–2020 mean for each calendar day, smoothed over
// a fortnight so it reads as the season rather than as noise.
const byDay = new Map();
for (const d of days.filter(inNormal)) {
  const key = d.date.slice(5);
  if (key === "02-29" || d.tmax === null) continue;
  byDay.set(key, [...(byDay.get(key) ?? []), d.tmax]);
}
const calendar = [...byDay.keys()].sort();
const rawNormal = calendar.map((key) => byDay.get(key).reduce((a, b) => a + b, 0) / byDay.get(key).length);
const normalTmax = new Map(
  calendar.map((key, i) => {
    let sum = 0;
    for (let k = -7; k <= 7; k++) sum += rawNormal[(i + k + rawNormal.length) % rawNormal.length];
    return [key, round(sum / 15)];
  }),
);

// The first season: from the month before the first planting to the end of that summer.
const season = days
  .filter((d) => d.date >= "2025-05-01" && d.date <= "2026-02-28")
  .map((d) => ({ date: d.date, rain: round(d.rain), tmax: round(d.tmax), normal: normalTmax.get(d.date.slice(5)) ?? null }));

writeFileSync(
  new URL("../src/data/climate.json", import.meta.url),
  `${JSON.stringify(
    {
      source: "ERA5 reanalysis (Copernicus Climate Change Service, ECMWF) via the Open-Meteo historical weather API",
      generated: new Date().toISOString().slice(0, 10),
      place: { ...PLACE, modelElevation: elevation },
      normal: NORMAL,
      months,
      years,
      monthly: { 2025: monthly(2025), 2026: monthly(2026) },
      season,
    },
    null,
    1,
  )}\n`,
);

const y2025 = years.find((y) => y.year === 2025);
const normalRain = months.reduce((t, m) => t + m.rain, 0);
const octobers = years.filter((y) => y.octoberTmax !== null).sort((a, b) => b.octoberTmax - a.octoberTmax);
console.log(
  `normal ${normalRain.toFixed(0)} mm/yr; 2025 ${y2025.rain} mm (${(((y2025.rain - normalRain) / normalRain) * 100).toFixed(0)}%); ` +
    `October 2025 Tmax ${y2025.octoberTmax} °C, #${octobers.findIndex((y) => y.year === 2025) + 1} of ${octobers.length}`,
);
