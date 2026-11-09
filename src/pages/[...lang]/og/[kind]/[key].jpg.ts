// One share picture per page worth sharing, in each language: /og/ascents/<entry>.jpg and so on,
// plus /og/site/home.jpg for everything else. The drawing is in src/lib/og.ts.
import type { APIRoute } from "astro";
import { getLocalizedCollection } from "../../../../i18n/content";
import { getLang, langParams, languages, useTranslations, type Lang } from "../../../../i18n/ui";
import { getOutings } from "../../../../lib/log";
import { formatDistance, formatElevation } from "../../../../lib/route-format";
import { renderOgCard, type OgCard } from "../../../../lib/og";
import { getSummits, skylineProfile, type PanoramaData } from "../../../../lib/summits";

const skyline = (panorama?: PanoramaData | null) => (panorama ? skylineProfile(panorama, 240) : null);
const clip = (text: string, max: number) => (text.length <= max ? text : `${text.slice(0, max).replace(/\s+\S*$/, "")}…`);

async function cards(lang: Lang) {
  const t = useTranslations(lang);
  const locale = languages[lang].locale;
  const list: { kind: string; key: string; card: OgCard }[] = [];
  const summits = await getSummits(lang);

  list.push({
    kind: "site",
    key: "home",
    card: { eyebrow: t("og.site.eyebrow"), title: "Gonzalo Olguín Moncada", detail: t("og.site.detail"), skyline: skyline(summits.find((s) => s.id === "provincia")?.panorama) },
  });

  for (const summit of summits) {
    const { data, panorama } = summit;
    const place = [`${data.elevation.toLocaleString(locale)} m`, data.region].filter(Boolean).join(" · ");
    list.push({ kind: "summits", key: summit.id, card: { eyebrow: t("nav.mountaineering"), title: data.name, detail: place, skyline: skyline(panorama) } });
  }

  for (const { key, item, summits: own, route } of await getOutings(lang)) {
    const { title, heroImage } = item.entry.data;
    const numbers = [
      own[0] && `${own[0].data.elevation.toLocaleString(locale)} m`,
      route && formatDistance(route.distance, locale),
      route?.gain != null && `+${formatElevation(route.gain, locale)}`,
    ].filter(Boolean);
    list.push({
      kind: "ascents",
      key,
      card: { eyebrow: t("nav.mountaineering"), title, detail: numbers.join(" · ") || undefined, photo: heroImage, skyline: skyline(own.find((s) => s.panorama)?.panorama) },
    });
  }

  for (const [collection, eyebrow] of [["projects", t("nav.projects")], ["blog", t("nav.news")]] as const) {
    for (const { key, entry } of await getLocalizedCollection(collection, lang)) {
      list.push({ kind: collection, key, card: { eyebrow, title: entry.data.title, detail: clip(entry.data.description, 120), photo: entry.data.heroImage } });
    }
  }
  return list;
}

export async function getStaticPaths() {
  const perLang = await Promise.all(
    langParams.map(async (lang) => (await cards(getLang(lang))).map(({ kind, key, card }) => ({ params: { lang, kind, key }, props: { card } }))),
  );
  return perLang.flat();
}

export const GET: APIRoute = async ({ props }) => {
  return new Response(await renderOgCard(props.card as OgCard), { headers: { "Content-Type": "image/jpeg" } });
};
