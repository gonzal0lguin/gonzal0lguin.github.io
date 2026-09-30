import { z, defineCollection } from "astro:content";

const projectsSchema = z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.coerce.date(),
    updatedDate: z.string().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    category: z.string().optional(),
    tags: z.array(z.string()).refine(items => new Set(items).size === items.length, {
        message: 'tags must be unique',
    }).optional(),
});

const ascentsSchema = z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.coerce.date(),
    mountain: z.string().optional(),
    elevation: z.number().optional(),
    latitude: z.number().optional(),
    longitude: z.number().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    category: z.string().optional(),
    tags: z.array(z.string()).refine(items => new Set(items).size === items.length, {
        message: 'tags must be unique',
    }).optional(),
});

const blogSchema = z.object({
    title: z.string(),
    description: z.string(),
    pubDate: z.coerce.date(),
    updatedDate: z.string().optional(),
    heroImage: z.string().optional(),
    badge: z.string().optional(),
    category: z.string().optional(),
    tags: z.array(z.string()).refine(items => new Set(items).size === items.length, {
        message: 'tags must be unique',
    }).optional(),
});

export type ProjectsSchema = z.infer<typeof projectsSchema>;
export type AscentsSchema = z.infer<typeof ascentsSchema>;
export type BlogSchema = z.infer<typeof blogSchema>;

const projectsCollection = defineCollection({ schema: projectsSchema });
const ascentsCollection = defineCollection({ schema: ascentsSchema });
const blogCollection = defineCollection({ schema: blogSchema });

export const collections = {
    'projects': projectsCollection,
    'ascents': ascentsCollection,
    'blog': blogCollection
}
