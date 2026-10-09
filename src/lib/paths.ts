import type { PaginateFunction } from "astro";
import { getLocalizedCollection, type LocalizedEntry } from "../i18n/content";
import { getLang, langParams } from "../i18n/ui";

// getStaticPaths helpers for pages under src/pages/[...lang]/. Each one generates the
// Spanish routes (lang param undefined → no prefix) and the English ones (/en/...).

type ListCollection = "projects" | "ascents" | "blog";
type Item<C extends ListCollection> = LocalizedEntry<C>;

const PAGE_SIZE = 10;

export const langStaticPaths = () => langParams.map((lang) => ({ params: { lang } }));

/** One page per entry: /<collection>/<key> */
export async function entryStaticPaths<C extends ListCollection>(collection: C) {
  const perLang = await Promise.all(
    langParams.map(async (lang) =>
      (await getLocalizedCollection(collection, getLang(lang))).map((item) => ({
        params: { lang, slug: item.key },
        props: { item },
      })),
    ),
  );
  return perLang.flat();
}

/** Paginated list of a collection, optionally filtered. */
export async function listStaticPaths<C extends ListCollection>(
  collection: C,
  paginate: PaginateFunction,
  filter: (item: Item<C>) => boolean = () => true,
) {
  const perLang = await Promise.all(
    langParams.map(async (lang) => {
      const items = (await getLocalizedCollection(collection, getLang(lang))).filter(filter);
      return paginate(items, { params: { lang }, pageSize: PAGE_SIZE });
    }),
  );
  return perLang.flat();
}

/** Paginated list per tag: /<collection>/tag/<tag> */
export async function tagStaticPaths<C extends ListCollection>(collection: C, paginate: PaginateFunction) {
  const perLang = await Promise.all(
    langParams.map(async (lang) => {
      const items = await getLocalizedCollection(collection, getLang(lang));
      const tags = [...new Set(items.flatMap((item) => item.entry.data.tags ?? []))];
      return tags.flatMap((tag) =>
        paginate(
          items.filter((item) => item.entry.data.tags?.includes(tag)),
          { params: { lang, tag }, pageSize: PAGE_SIZE, props: { tag } },
        ),
      );
    }),
  );
  return perLang.flat();
}
