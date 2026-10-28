import { z, defineCollection, reference } from "astro:content";
import { WAYPOINT_KIND_IDS, type WaypointKind } from "../lib/waypoint-kinds";

const tags = z.array(z.string()).refine(items => new Set(items).size === items.length, {
    message: 'tags must be unique',
}).optional();

// Set by `npm run translate` on English files it generates: a hash of the Spanish
// source they were translated from. Remove it to keep a hand-edited translation.
const translationHash = z.string().optional();

const projectsSchema = z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.coerce.date(),
    updatedDate: z.coerce.date().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    // Matches the category URL: /projects/<category>
    category: z.enum(["academic-projects", "hobby-projects", "personal"]).optional(),
    tags,
    translationHash,
});

const ascentsSchema = z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.coerce.date(),
    mountain: z.string().optional(),
    elevation: z.number().optional(),
    latitude: z.number().optional(),
    longitude: z.number().optional(),
    // GPS track shown as an interactive map: public path of a .gpx file, e.g. "/assets/lib/union.gpx",
    // or a list of them when the route was recorded in several parts (they're stitched into one)
    route: z.union([z.string(), z.array(z.string()).nonempty()]).optional(),
    // Points to mark on the route map besides the GPX file's own waypoints, anywhere, e.g. a
    // summit that wasn't reached (then the route's highest point shows as "high point"):
    //   - { name: "Cerro Plomo", kind: summit, lat: -33.2385, lon: -70.2156, ele: 5424, note: "..." }
    // kind: camp, summit, view, water, hut, crux, danger, parking or note (default)
    waypoints: z.array(z.object({
        name: z.string(),
        lat: z.number().min(-90).max(90),
        lon: z.number().min(-180).max(180),
        kind: z.enum(WAYPOINT_KIND_IDS as [WaypointKind, ...WaypointKind[]]).default("note"),
        ele: z.number().optional(),
        note: z.string().optional(),
    })).optional(),
    // IANA timezone for the route's clock times (default America/Santiago)
    timezone: z.string().optional(),
    // The summits of this outing, by file name in src/content/summits/, e.g. [provincia] or, for
    // a traverse, [manquehue, manquehuito]. They give the post its summit panorama tab and put
    // it on each summit's page.
    summits: z.array(reference("summits")).optional(),
    // The summit wasn't reached (the summits above are the ones attempted)
    attempt: z.boolean().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    category: z.string().optional(),
    tags,
    translationHash,
});

// One file per summit of the log: where it is, and whether `npm run panoramas` computes the view
// from its top (written to src/data/panoramas/<file name>.json).
const summitsSchema = z.object({
    name: z.string(),
    // Official height, m
    elevation: z.number(),
    lat: z.number().min(-90).max(90),
    lon: z.number().min(-180).max(180),
    // OpenStreetMap node of the summit, to recognise it in other summits' panoramas
    osm: z.number().optional(),
    wikidata: z.string().optional(),
    region: z.string().optional(),
    panorama: z.boolean().default(false),
    timezone: z.string().optional(),
    // Photos taken from the top; `azimuth` (degrees from north, where the camera points) places
    // them on the panorama
    photos: z.array(z.object({
        src: z.string(),
        azimuth: z.number().min(0).max(360).optional(),
        caption: z.string().optional(),
    })).optional(),
});

const blogSchema = z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.coerce.date(),
    updatedDate: z.coerce.date().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    category: z.string().optional(),
    tags,
    translationHash,
});

// Standalone page content (CV, home intro) so it can be written in Spanish and translated.
const pagesSchema = z.object({
    title: z.string(),
    description: z.string().optional(),
    greeting: z.string().optional(),
    subtitle: z.string().optional(),
    translationHash,
});

export type ProjectsSchema = z.infer<typeof projectsSchema>;
export type AscentsSchema = z.infer<typeof ascentsSchema>;
export type BlogSchema = z.infer<typeof blogSchema>;
export type SummitsSchema = z.infer<typeof summitsSchema>;

export const collections = {
    'projects': defineCollection({ schema: projectsSchema }),
    'ascents': defineCollection({ schema: ascentsSchema }),
    'blog': defineCollection({ schema: blogSchema }),
    'pages': defineCollection({ schema: pagesSchema }),
    'summits': defineCollection({ type: 'data', schema: summitsSchema }),
}
