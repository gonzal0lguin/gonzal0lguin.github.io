// The route of each mountaineering post as KML, for "Open in Google Earth": /routes/<entry>.kml
// (and /en/routes/<entry>.kml with English labels).
import type { APIRoute } from "astro";
import { getLocalizedCollection } from "../../../i18n/content";
import { getLang, langParams } from "../../../i18n/ui";
import { loadRoute, withPostWaypoints } from "../../../lib/gpx";
import { toKml } from "../../../lib/kml";
import { placePhotos } from "../../../lib/photos";

export async function getStaticPaths() {
  const perLang = await Promise.all(
    langParams.map(async (lang) =>
      (await getLocalizedCollection("ascents", getLang(lang)))
        .filter((item) => item.entry.data.route)
        .map((item) => ({ params: { lang, slug: item.key }, props: { item } })),
    ),
  );
  return perLang.flat();
}

export const GET: APIRoute = async ({ props, params, site }) => {
  const { entry } = props.item as Awaited<ReturnType<typeof getLocalizedCollection<"ascents">>>[number];
  const { route: files, waypoints, mountain, title, timezone = "America/Santiago" } = entry.data;
  const route = withPostWaypoints(loadRoute(files!), waypoints);
  // The post's photos with a known location, with absolute URLs so Google Earth can load them.
  const { remarkPluginFrontmatter } = await entry.render();
  const photos = placePhotos(remarkPluginFrontmatter.photos, route, timezone).map((photo) => ({
    ...photo,
    src: new URL(photo.src, site).href,
  }));
  const kml = toKml(route, { lang: getLang(params.lang), title: mountain ?? title, photos });
  return new Response(kml, { headers: { "Content-Type": "application/vnd.google-earth.kml+xml" } });
};
