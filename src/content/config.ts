import { z, defineCollection } from "astro:content";

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
    // GPS track shown as an interactive map: public path of a .gpx file, e.g. "/assets/tracks/union.gpx"
    route: z.string().optional(),
    // IANA timezone for the route's clock times (default America/Santiago)
    timezone: z.string().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    category: z.string().optional(),
    tags,
    translationHash,
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

export const collections = {
    'projects': defineCollection({ schema: projectsSchema }),
    'ascents': defineCollection({ schema: ascentsSchema }),
    'blog': defineCollection({ schema: blogSchema }),
    'pages': defineCollection({ schema: pagesSchema }),
}
