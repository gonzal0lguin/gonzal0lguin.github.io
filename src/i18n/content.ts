import { getCollection, type CollectionEntry } from "astro:content";
import type { Lang } from "./ui";

// Content is written in Spanish at src/content/<collection>/<file>.md(x).
// English versions live at src/content/<collection>/en/<file>.md(x) — written by hand
// or generated with `npm run translate`. When an English version is missing, the
// Spanish entry is shown instead and `fallback` is set so pages can say so.

type Collection = "projects" | "ascents" | "blog" | "pages";

export interface LocalizedEntry<C extends Collection> {
  /** Language-neutral identifier, used in URLs: "volcan-osorno". */
  key: string;
  entry: CollectionEntry<C>;
  /** True when an English page is showing the Spanish original. */
  fallback: boolean;
}

const EN_PREFIX = "en/";

export function entryKey(entry: { slug: string }): string {
  return entry.slug.startsWith(EN_PREFIX) ? entry.slug.slice(EN_PREFIX.length) : entry.slug;
}

/** All entries of a collection in `lang`, newest first when entries have a pubDate. */
export async function getLocalizedCollection<C extends Collection>(
  collection: C,
  lang: Lang,
): Promise<LocalizedEntry<C>[]> {
  const all = (await getCollection(collection)) as CollectionEntry<C>[];
  const english = new Map(all.filter((e) => e.slug.startsWith(EN_PREFIX)).map((e) => [entryKey(e), e]));

  const items = all
    .filter((e) => !e.slug.startsWith(EN_PREFIX))
    .map((source) => {
      const key = entryKey(source);
      const translated = lang === "en" ? english.get(key) : undefined;
      return { key, entry: translated ?? source, fallback: lang === "en" && !translated };
    });

  return items.sort((a, b) => pubDate(b.entry) - pubDate(a.entry));
}

export async function getLocalizedEntry<C extends Collection>(collection: C, lang: Lang, key: string) {
  const items = await getLocalizedCollection(collection, lang);
  const item = items.find((i) => i.key === key);
  if (!item) throw new Error(`No "${key}" entry in the "${collection}" collection`);
  return item;
}

function pubDate(entry: CollectionEntry<Collection>): number {
  return "pubDate" in entry.data ? entry.data.pubDate.valueOf() : 0;
}
