// The whole photo library: what is in public/assets/img, who uses it, and what can go.
import * as api from "../api";
import { actionsSlot, type Ctx } from "../app";
import { button, confirmDialog, el, fail, fill, formatBytes, icon, ICONS, promptDialog, toast } from "../dom";
import { isRemoteMedia } from "../media-url";
import { runTask } from "./dashboard";

export async function mediaView(ctx: Ctx): Promise<Node> {
  const library = await api.get<api.MediaLibrary>("/media");
  const total = library.posts.reduce((bytes, file) => bytes + file.size, 0);
  ctx.heading("Fotos", `${library.posts.length} en posts · ${library.headers.length} portadas · ${formatBytes(total)}`);

  const root = el("div", { class: "ed-stack" });
  const draw = () => {
    const groups = new Map<string, api.MediaFile[]>();
    for (const file of library.posts) {
      const key = file.folder || "(raíz)";
      groups.set(key, [...(groups.get(key) ?? []), file]);
    }
    fill(root,
      isRemoteMedia() ? el("p", { class: "ed-warn" }, `PUBLIC_MEDIA_BASE_URL = ${library.mediaBaseUrl}. El sitio sirve las fotos desde ahí; lo que subas aquí sólo queda en el repositorio.`) : null,
      ...[...groups.entries()].sort().map(([folder, files]) => section(ctx, `posts/${folder}`, files, draw)),
      section(ctx, "headers (portadas)", library.headers, draw),
    );
  };

  fill(actionsSlot(),
    button("Nueva carpeta con fotos…", () => void uploadToNewFolder(ctx), { variant: "primary", iconPath: ICONS.plus }),
    button("Leer EXIF", () => void runTask("photos", [], "npm run photos"), { title: "npm run photos" }),
    button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }),
  );

  draw();
  return root;
}

function section(ctx: Ctx, title: string, files: api.MediaFile[], redraw: () => void) {
  return el(
    "section",
    { class: "ed-card" },
    el("header", { class: "ed-card__head" }, el("h2", {}, title, el("span", { class: "ed-chip" }, String(files.length)))),
    el(
      "div",
      { class: "ed-picker" },
      ...files.map((file) =>
        el(
          "div",
          { class: `ed-picker__item ${file.usedBy.length === 0 ? "is-unused" : ""}` },
          el("a", { href: `/${file.path.replace(/^public\//, "")}`, target: "_blank", rel: "noreferrer" }, el("img", { src: `/${file.path.replace(/^public\//, "")}`, alt: "", loading: "lazy" })),
          el("span", { title: file.path }, file.name),
          el("small", {}, formatBytes(file.size), file.usedBy.length ? ` · ${file.usedBy.length} uso(s)` : " · sin usar"),
          el(
            "div",
            { class: "ed-picker__tools" },
            file.usedBy.length
              ? el("span", { class: "ed-chip ed-chip--soft", title: file.usedBy.join("\n") }, "usada")
              : button("", () => void remove(ctx, file, redraw), { variant: "ghost", iconPath: ICONS.trash, title: "Mover a la papelera" }),
          ),
        ),
      ),
    ),
  );
}

async function remove(ctx: Ctx, file: api.MediaFile, redraw: () => void) {
  const ok = await confirmDialog({ title: `¿Borrar ${file.name}?`, message: `Se mueve a ${ctx.state.localDir}/trash.`, confirm: "Mover a la papelera", danger: true });
  if (!ok) return;
  try {
    await api.post("/media/delete", { paths: [file.path] });
    toast("A la papelera", "ok");
    await ctx.refresh();
    redraw();
  } catch (error) {
    fail("No se pudo borrar", error);
  }
}

async function uploadToNewFolder(ctx: Ctx) {
  const folder = await promptDialog({ title: "Subir fotos", label: "Carpeta dentro de posts/", hint: "Normalmente el slug de la entrada, p.ej. la-cruz", confirm: "Elegir archivos" });
  if (!folder) return;
  const input = el("input", { type: "file", accept: "image/*", multiple: true }) as HTMLInputElement;
  input.addEventListener("change", async () => {
    let count = 0;
    for (const file of [...(input.files ?? [])]) {
      try {
        await api.uploadFile(file, { kind: "photo", folder });
        count++;
      } catch (error) {
        fail(`No se pudo subir ${file.name}`, error);
      }
    }
    toast(`${count} foto(s) en posts/${folder}/`, "ok");
    await ctx.refresh();
  });
  input.click();
}

