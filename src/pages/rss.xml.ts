import rss from "@astrojs/rss";
import type { APIContext } from "astro";
import { getLocalizedCollection } from "../i18n/content";
import { useTranslations } from "../i18n/ui";

// Feed of all Spanish entries (news, projects and mountaineering), newest first.
export async function GET(context: APIContext) {
  const t = useTranslations("es");
  const collections = [
    { name: "blog", path: "/blog" },
    { name: "projects", path: "/projects" },
    { name: "ascents", path: "/ascents" },
  ] as const;

  const items = (
    await Promise.all(
      collections.map(async ({ name, path }) =>
        (await getLocalizedCollection(name, "es")).map(({ key, entry }) => ({
          title: entry.data.title,
          pubDate: entry.data.pubDate,
          description: entry.data.description,
          link: `${path}/${key}/`,
        })),
      ),
    )
  )
    .flat()
    .sort((a, b) => b.pubDate.valueOf() - a.pubDate.valueOf());

  return rss({
    title: t("site.title"),
    description: t("site.description"),
    site: context.site ?? "https://gonzal0lguin.github.io",
    customData: "<language>es-CL</language>",
    items,
  });
}
