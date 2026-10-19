import { defineConfig } from 'astro/config';
import mdx from '@astrojs/mdx';
import tailwind from "@astrojs/tailwind";
import sitemap from "@astrojs/sitemap";
import remarkDirective from 'remark-directive';
import remarkMath from 'remark-math';
import remarkMedia from './src/plugins/remark-media.mjs';
import { loadEnv } from 'vite';
import rehypeKatex from 'rehype-katex';

// Photos referenced by a relative path in posts are resolved against this base URL, so the
// photo library can live outside the repo (see src/lib/media.mjs and the README).
// `loadEnv` is needed because .env files are not applied to this file automatically.
const { PUBLIC_MEDIA_BASE_URL } = loadEnv(process.env.NODE_ENV ?? 'production', process.cwd(), '');

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
    remarkPlugins: [remarkMath, remarkDirective, [remarkMedia, { baseUrl: PUBLIC_MEDIA_BASE_URL }]],
    rehypePlugins: [rehypeKatex],
  },
});
