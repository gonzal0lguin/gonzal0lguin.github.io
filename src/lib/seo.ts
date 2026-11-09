// What search engines and link previews read about a page: which share picture it has and the
// schema.org description of who wrote it. The <head> markup itself is src/components/BaseHead.astro.

import { LINKS } from "../config";
import { getLangFromUrl, localizePath, stripLang, type Lang } from "../i18n/ui";

/** The share picture of the page at `url`: /og/<kind>/<key>.jpg, as the URL's own path says. */
export function ogImagePath(url: URL) {
  const parts = stripLang(url.pathname).split("/").filter(Boolean);
  const key = parts.at(-1);
  const kind = parts[1] === "summits" ? "summits" : parts[0];
  return localizePath(getLangFromUrl(url), `/og/${kind}/${key}.jpg`);
}

const AUTHOR = { "@type": "Person", name: "Gonzalo Olguín Moncada", url: "https://gonzal0lguin.github.io/" };

export function articleJsonLd(args: { title: string; description: string; published: Date; modified?: Date; image: string; url: URL; site?: URL; lang: Lang }) {
  const { title, description, published, modified, image, url, site, lang } = args;
  return {
    "@context": "https://schema.org",
    "@type": "BlogPosting",
    headline: title,
    description,
    inLanguage: lang,
    datePublished: published.toISOString(),
    ...(modified ? { dateModified: modified.toISOString() } : {}),
    image: new URL(image, site ?? url).href,
    mainEntityOfPage: url.href,
    author: AUTHOR,
  };
}

export function personJsonLd(args: { jobTitle: string; description: string; image: string; site?: URL; url: URL }) {
  const { jobTitle, description, image, site, url } = args;
  return [
    {
      "@context": "https://schema.org",
      ...AUTHOR,
      jobTitle,
      description,
      image: new URL(image, site ?? url).href,
      worksFor: { "@type": "Organization", name: "Advanced Mining Technology Center (AMTC)", url: "https://www.amtc.cl/" },
      alumniOf: { "@type": "CollegeOrUniversity", name: "Universidad de Chile" },
      // The Scholar link stays out until it points at the profile itself (see LINKS).
      sameAs: [LINKS.github, LINKS.linkedin],
    },
    { "@context": "https://schema.org", "@type": "WebSite", name: "Gonzalo Olguín", url: "https://gonzal0lguin.github.io/" },
  ];
}
