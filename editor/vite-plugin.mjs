// Mounts the editor API on the Astro dev server. `apply: "serve"` keeps it out of every
// build: `npm run build` never loads this file, and the editor's pages are not generated
// either (see src/pages/edit/[...path].astro).
//
// Everything about the editor lives in editor/ (this API) and src/editor/ (the interface);
// the site itself does not import either. See editing.md.
import { loadEnv } from "vite";
import { createEditorApi } from "./server/api.mjs";

export const EDITOR_API_BASE = "/__editor/api";

export default function editorPlugin() {
  return {
    name: "site-editor",
    apply: "serve",
    configureServer(server) {
      const root = server.config.root ?? process.cwd();
      const { PUBLIC_MEDIA_BASE_URL } = loadEnv(server.config.mode ?? "development", root, "");
      server.middlewares.use(EDITOR_API_BASE, createEditorApi({ root, mediaBaseUrl: PUBLIC_MEDIA_BASE_URL || null }));
      // The port is Astro's, not Vite's, so only the path is printed.
      server.config.logger.info(`  \x1b[32m➜\x1b[39m  \x1b[1mEditor\x1b[22m:  /edit  (sólo en desarrollo — ver editing.md)`, { timestamp: false });
    },
  };
}
