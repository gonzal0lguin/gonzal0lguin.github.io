// Entry point of the editor interface, loaded at runtime by src/pages/edit/[...path].astro.
//
// The page imports this with a dynamic `import("/src/editor/boot.ts")` inside an
// `is:inline` script, which Astro deliberately does not analyze: the editor's code and
// styles therefore never enter the production bundle. The dev server resolves the path
// through Vite as usual.
import "../styles/editor.css";
import { startEditor } from "./app";

export async function boot(root: HTMLElement) {
  await startEditor(root);
}
