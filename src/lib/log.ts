// The log as the mountaineering pages list it: every outing with its summits (src/lib/summits.ts)
// and, when it has a GPS track, the route's numbers and a light elevation profile.

import { getLocalizedCollection, type LocalizedEntry } from "../i18n/content";
import type { Lang } from "../i18n/ui";
import { loadRoute } from "./gpx";
import { ascentSummitIds, getSummits, type Summit } from "./summits";

export interface Outing {
  key: string;
  item: LocalizedEntry<"ascents">;
  summits: Summit[];
  attempt: boolean;
  route: {
    distance: number;
    gain: number | null;
    maxEle: number | null;
    movingTime: number | null;
    totalTime: number | null;
    days: number;
    /** [distance m, elevation m] every ~1/80 of the way, for sparklines. */
    profile: [number, number][];
  } | null;
}

export async function getOutings(lang: Lang): Promise<Outing[]> {
  const [items, summits] = await Promise.all([getLocalizedCollection("ascents", lang), getSummits(lang)]);
  return Promise.all(
    items.map(async (item) => {
      const ids = await ascentSummitIds(item.key);
      const mine = ids.flatMap((id) => summits.filter((summit) => summit.id === id));
      const attempt = !!mine[0]?.ascents.find((ascent) => ascent.key === item.key)?.attempt;
      let route: Outing["route"] = null;
      if (item.entry.data.route) {
        const data = loadRoute(item.entry.data.route);
        const step = Math.max(1, Math.floor(data.points.length / 80));
        route = {
          distance: data.stats.distance,
          gain: data.stats.gain,
          maxEle: data.stats.maxEle,
          movingTime: data.stats.movingTime,
          totalTime: data.stats.totalTime,
          days: Math.max(1, data.days.length),
          profile: data.hasElevation
            ? data.points.filter((_, i, all) => i % step === 0 || i === all.length - 1).map((p) => [p[3], p[2] ?? 0] as [number, number])
            : [],
        };
      }
      return { key: item.key, item, summits: mine, attempt, route };
    }),
  );
}
