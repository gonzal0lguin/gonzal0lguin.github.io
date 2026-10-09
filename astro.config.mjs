import { defineConfig } from 'astro/config';
import mdx from '@astrojs/mdx';
import tailwind from "@astrojs/tailwind";
import sitemap from "@astrojs/sitemap";
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

// https://astro.build/config
export default defineConfig({
  site: 'https://gonzal0lguin.github.io',
  integrations: [
    mdx(),
    tailwind(),
    // Spanish pages live at the root, English ones under /en/ (see src/i18n/ui.ts).
    sitemap({
      i18n: { defaultLocale: 'es', locales: { es: 'es-CL', en: 'en-GB' } },
    }),
  ],
  markdown: {
    remarkPlugins: [remarkMath],
    rehypePlugins: [rehypeKatex],
  },
});
