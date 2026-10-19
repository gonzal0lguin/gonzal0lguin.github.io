// Stitched GPX download for posts whose route was recorded in several files: /routes/<entry>.gpx
import type { APIRoute } from "astro";
import { getCollection } from "astro:content";
import { entryKey } from "../../i18n/content";
import { loadTrack, routeFileList } from "../../lib/gpx";
import { toGpx } from "../../lib/gpx-core";

export async function getStaticPaths() {
  const paths = new Map<string, { params: { slug: string }; props: { files: string[] } }>();
  for (const entry of await getCollection("ascents")) {
    const files = entry.data.route ? routeFileList(entry.data.route) : [];
    const slug = entryKey(entry);
    // The Spanish entry wins if a translation lists different files.
    if (files.length > 1 && !(paths.has(slug) && entry.slug.startsWith("en/"))) {
      paths.set(slug, { params: { slug }, props: { files } });
    }
  }
  return [...paths.values()];
}

export const GET: APIRoute = ({ props }) =>
  new Response(toGpx(loadTrack(props.files)), { headers: { "Content-Type": "application/gpx+xml" } });
