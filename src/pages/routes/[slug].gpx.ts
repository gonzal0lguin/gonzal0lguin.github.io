// GPX download for posts whose route was recorded in several files (stitched into one) or that
// mark points in their frontmatter (added as waypoints): /routes/<entry>.gpx
import type { APIRoute } from "astro";
import { getCollection } from "astro:content";
import { entryKey } from "../../i18n/content";
import { generatedDownload, loadTrack, routeFileList, withPostWaypoints, type PostWaypoint } from "../../lib/gpx";
import { toGpx } from "../../lib/gpx-core";

export async function getStaticPaths() {
  const paths = new Map<string, { params: { slug: string }; props: { files: string[]; waypoints?: PostWaypoint[] } }>();
  for (const entry of await getCollection("ascents")) {
    const files = entry.data.route ? routeFileList(entry.data.route) : [];
    const slug = entryKey(entry);
    const { waypoints } = entry.data;
    // The Spanish entry wins if a translation lists different files.
    if (files.length && generatedDownload(files, waypoints) && !(paths.has(slug) && entry.slug.startsWith("en/"))) {
      paths.set(slug, { params: { slug }, props: { files, waypoints } });
    }
  }
  return [...paths.values()];
}

export const GET: APIRoute = ({ props }) =>
  new Response(toGpx(withPostWaypoints(loadTrack(props.files), props.waypoints)), { headers: { "Content-Type": "application/gpx+xml" } });
