// The site itself, as opposed to its posts: the standalone pages (home, CV) and the files
// the templates point at — the profile photo, the CV PDF, the favicon.
//
// Those files are referenced by path from .astro components, so the editor never rewrites the
// code: it replaces the file in place, keeping its name and extension.
import * as api from "../api";
import { actionsSlot, type Ctx } from "../app";
import { ICONS, button, confirmDialog, el, fail, fill, formatBytes, icon, toast } from "../dom";
import { entryRoute, entryUrl, runTask } from "./dashboard";

export async function siteView(ctx: Ctx): Promise<Node> {
  const assets = await api.get<SiteAsset[]>("/site/assets");
  const pages = ctx.state.collections.find((collection) => collection.id === "pages");
  ctx.heading("Sitio", "Páginas fijas y archivos a los que apuntan las plantillas");

  fill(actionsSlot(), button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }));

  const assetList = el("div", { class: "ed-picker" });
  const drawAssets = () => {
    fill(
      assetList,
      ...assets.map((asset) =>
        el(
          "div",
          { class: `ed-picker__item ${asset.missing ? "is-unused" : ""}` },
          asset.kind === "image"
            ? el("a", { href: asset.publicPath, target: "_blank", rel: "noreferrer" }, el("img", { src: asset.publicPath, alt: "", loading: "lazy" }))
            : el("a", { class: "ed-filecard", href: asset.publicPath, target: "_blank", rel: "noreferrer" }, el("span", {}, asset.kind.toUpperCase())),
          el("span", { title: asset.path }, asset.name),
          el("small", {}, asset.missing ? "¡no existe!" : formatBytes(asset.size)),
          el("small", { title: asset.usedIn.join("\n") }, asset.usedIn.length ? `usado en ${asset.usedIn.length} archivo(s)` : "sin usos en el código"),
          el("div", { class: "ed-picker__tools" }, replaceButton(ctx, asset, drawAssets)),
        ),
      ),
    );
  };
  drawAssets();

  return el(
    "div",
    { class: "ed-stack" },
    el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, "Páginas fijas")),
      el("p", { class: "ed-note" }, "El texto de la portada y del CV vive en src/content/pages. Se editan como cualquier entrada: bloques, formulario y archivo completo."),
      el(
        "ul",
        { class: "ed-list ed-list--tight" },
        ...(pages?.entries ?? []).map((entry) =>
          el(
            "li",
            { class: "ed-list__row" },
            el(
              "a",
              { class: "ed-list__main", href: `#/${entryRoute(entry.path)}` },
              el("span", { class: "ed-list__title" }, entry.key === "home" ? "Portada" : entry.title),
              el("span", { class: "ed-list__meta" }, entry.path),
            ),
            el("a", { class: "ed-chip", href: entryUrl(pages, entry.key), target: "_blank", rel: "noreferrer" }, "ver ↗"),
            entry.translation
              ? el("a", { class: `ed-chip ${entry.translation.state === "stale" ? "ed-chip--warn" : "ed-chip--ok"}`, href: `#/${entryRoute(entry.translation.path)}` }, "EN")
              : el("span", { class: "ed-chip ed-chip--muted" }, "sin EN"),
            button("Traducir", () => void runTask("translate", [entry.path], `npm run translate -- ${entry.path}`).then(() => ctx.refresh()), { variant: "ghost" }),
          ),
        ),
      ),
    ),
    el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, "Archivos del sitio", el("span", { class: "ed-chip" }, String(assets.length)))),
      el(
        "p",
        { class: "ed-note" },
        "La foto de perfil, el PDF del CV, el favicon y las banderas. Las plantillas los llaman por su ruta, así que ",
        el("strong", {}, "“Reemplazar” mantiene el nombre"),
        " y el archivo anterior queda en la copia de seguridad. Para cambiar la ruta en sí hay que editar el .astro que la usa.",
      ),
      assetList,
    ),
  );
}

function replaceButton(ctx: Ctx, asset: SiteAsset, redraw: () => void) {
  const extension = asset.name.slice(asset.name.lastIndexOf("."));
  const input = el("input", { type: "file", accept: extension, class: "ed-hidden" }) as HTMLInputElement;
  input.addEventListener("change", async () => {
    const file = input.files?.[0];
    input.value = "";
    if (!file) return;
    if (!file.name.toLowerCase().endsWith(extension.toLowerCase())) {
      toast(`El archivo nuevo tiene que ser ${extension} para mantener la ruta ${asset.publicPath}`, "error");
      return;
    }
    const ok = await confirmDialog({
      title: `¿Reemplazar ${asset.name}?`,
      message: `${file.name} (${formatBytes(file.size)}) pasa a ser ${asset.publicPath}. El archivo actual se copia a ${ctx.state.localDir}/backups.`,
      confirm: "Reemplazar",
      danger: true,
    });
    if (!ok) return;
    try {
      const result = await api.replaceFile(asset.path, file);
      toast(`${asset.publicPath} reemplazado (${formatBytes(result.size)})`, "ok");
      asset.size = result.size;
      asset.missing = false;
      redraw();
    } catch (error) {
      fail("No se pudo reemplazar", error);
    }
  });
  return el("label", { class: "ed-btn ed-btn--ghost", title: `Subir otro ${extension} con esta misma ruta` }, icon(ICONS.save), el("span", {}, "Reemplazar"), input);
}

export interface SiteAsset {
  publicPath: string;
  path: string;
  name: string;
  kind: string;
  usedIn: string[];
  size: number;
  mtime: number;
  missing?: boolean;
}
