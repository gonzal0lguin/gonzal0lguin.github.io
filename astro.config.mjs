import { defineConfig } from 'astro/config';
import mdx from '@astrojs/mdx';
import tailwind from "@astrojs/tailwind";
import sitemap from "@astrojs/sitemap";
import remarkDirective from 'remark-directive';
import remarkMath from 'remark-math';
import remarkMedia from './src/plugins/remark-media.mjs';
import { loadEnv } from 'vite';
import rehypeKatex from 'rehype-katex';
import editor from './editor/vite-plugin.mjs';

// Photos referenced by a relative path in posts are resolved against this base URL, so the
// photo library can live outside the repo (see src/lib/media.mjs and the README).
// `loadEnv` is needed because .env files are not applied to this file automatically.
const { PUBLIC_MEDIA_BASE_URL } = loadEnv(process.env.NODE_ENV ?? 'production', process.cwd(), '');

// https://astro.build/config
export default defineConfig({
  site: 'https://gonzal0lguin.github.io',
  // The route analyzer moved into the tools section; old links keep working.
  redirects: {
    '/ascents/route-tool': '/ascents/tools/route-analyzer/',
    '/en/ascents/route-tool': '/en/ascents/tools/route-analyzer/',
  },
  integrations: [
    mdx(),
    tailwind(),
    // Spanish pages live at the root, English ones under /en/ (see src/i18n/ui.ts).
    sitemap({
      // /hero/<summit>/ are fragments of the home page, not pages.
      filter: (page) => !page.includes('/hero/'),
      i18n: { defaultLocale: 'es', locales: { es: 'es-CL', en: 'en-GB' } },
    }),
  ],
  vite: {
    // The editor at /edit: a development-only tool (see editing.md). The plugin declares
    // `apply: "serve"`, so it does nothing during a build.
    plugins: [editor()],
    // Astro scans all of src/ for browser dependencies, which reaches sharp through src/lib/og.ts
    // even though only the share-card endpoint runs it. Its native .node binary can't be bundled,
    // and that one failure aborts pre-bundling for the real browser dependencies too.
    optimizeDeps: { exclude: ['sharp'] },
  },
    markdown: {
    remarkPlugins: [remarkMath, remarkDirective, [remarkMedia, { baseUrl: PUBLIC_MEDIA_BASE_URL }]],
    rehypePlugins: [rehypeKatex],
  },
});
