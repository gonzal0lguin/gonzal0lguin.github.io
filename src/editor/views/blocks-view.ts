// The block editor: the body of a post as a list of text paragraphs and photo groups.
// Photos are edited as photos — layout, columns, shape, captions, where they sit on the
// route — and the Markdown is written back in the form src/plugins/remark-media.mjs reads.
import * as api from "../api";
import { type Block, type Photo, type PhotosBlock, type TextBlock, blockLabel, emptyPhotos, emptyText, headingLevel } from "../blocks";
import { button, confirmDialog, el, fail, fill, icon, ICONS, toast } from "../dom";
import { isRemoteMedia, photoUrl } from "../media-url";
import { GEO_MODES, RATIOS } from "../schema";
import type { EntryCtx } from "./entry";

export function blocksView(entry: EntryCtx): Node {
  const list = el("div", { class: "ed-blocks" });

  const draw = () => {
    fill(list,
      ...entry.blocks.flatMap((block, index) => [dropLine(index), blockCard(entry, block, index, draw)]),
      dropLine(entry.blocks.length),
      el(
        "div",
        { class: "ed-blocks__add" },
        button("Texto", () => insert(entry.blocks.length, emptyText()), { iconPath: ICONS.plus }),
        button("Galería", () => insert(entry.blocks.length, emptyPhotos("gallery")), { iconPath: ICONS.image }),
        button("Carrusel", () => insert(entry.blocks.length, emptyPhotos("carousel")), { iconPath: ICONS.image }),
      ),
    );
  };

  const insert = (at: number, block: Block) => {
    entry.blocks.splice(at, 0, block);
    entry.markDirty();
    draw();
  };

  /** The gap between two blocks: where a dragged block or a dropped photo lands. */
  function dropLine(at: number) {
    const line = el("div", { class: "ed-drop", dataset: { at: String(at) } });
    line.addEventListener("dragover", (event) => {
      event.preventDefault();
      line.classList.add("is-over");
    });
    line.addEventListener("dragleave", () => line.classList.remove("is-over"));
    line.addEventListener("drop", async (event) => {
      event.preventDefault();
      line.classList.remove("is-over");
      const moved = event.dataTransfer?.getData("text/ed-block");
      if (moved) {
        const from = entry.blocks.findIndex((b) => b.id === moved);
        if (from === -1) return;
        const [block] = entry.blocks.splice(from, 1);
        entry.blocks.splice(from < at ? at - 1 : at, 0, block);
        entry.markDirty();
        draw();
        return;
      }
      const files = [...(event.dataTransfer?.files ?? [])].filter((file) => file.type.startsWith("image/"));
      if (files.length === 0) return;
      const photos = await uploadPhotos(entry, files);
      if (photos.length) insert(at, emptyPhotos(photos.length > 1 ? "gallery" : "auto", photos));
    });
    return line;
  }

  draw();

  return el(
    "div",
    { class: "ed-pane" },
    el(
      "p",
      { class: "ed-note" },
      "Cada párrafo, lista o bloque de fotos es una tarjeta: arrástrala por el asa para moverla, o suelta imágenes entre dos tarjetas para insertarlas. ",
      isRemoteMedia() ? el("strong", {}, "Las fotos se suben a public/assets/img/posts, pero el sitio las lee de PUBLIC_MEDIA_BASE_URL: súbelas también allí.") : "",
    ),
    list,
  );
}

function blockCard(entry: EntryCtx, block: Block, index: number, redraw: () => void) {
  const move = (to: number) => {
    if (to < 0 || to >= entry.blocks.length) return;
    const [item] = entry.blocks.splice(index, 1);
    entry.blocks.splice(to, 0, item);
    entry.markDirty();
    redraw();
  };
  const remove = async () => {
    const ok = block.type === "text" && block.text.trim().length < 80 ? true : await confirmDialog({ title: "¿Borrar este bloque?", message: blockLabel(block), confirm: "Borrar", danger: true });
    if (!ok) return;
    entry.blocks.splice(index, 1);
    entry.markDirty();
    redraw();
  };

  const handle = el("button", { type: "button", class: "ed-block__handle", title: "Arrastra para mover", draggable: true }, icon(ICONS.drag));
  const card = el("article", { class: `ed-block ed-block--${block.type}` });
  handle.addEventListener("dragstart", (event) => {
    (event as DragEvent).dataTransfer?.setData("text/ed-block", block.id);
    card.classList.add("is-dragging");
  });
  handle.addEventListener("dragend", () => card.classList.remove("is-dragging"));

  card.append(
    el(
      "header",
      { class: "ed-block__bar" },
      handle,
      el("span", { class: `ed-block__label ed-block__label--h${headingLevel(block)}` }, blockLabel(block)),
      el(
        "div",
        { class: "ed-block__tools" },
        button("", () => move(index - 1), { variant: "ghost", iconPath: ICONS.up, title: "Subir" }),
        button("", () => move(index + 1), { variant: "ghost", iconPath: ICONS.down, title: "Bajar" }),
        button("", () => void remove(), { variant: "ghost", iconPath: ICONS.trash, title: "Borrar el bloque" }),
      ),
    ),
    block.type === "text" ? textEditor(entry, block, redraw) : photosEditor(entry, block, redraw),
  );
  return card;
}

// --- Text

function textEditor(entry: EntryCtx, block: TextBlock, redraw: () => void) {
  const area = el("textarea", { class: "ed-textarea", spellcheck: "true", rows: 2 }, block.text);
  const grow = () => {
    area.style.height = "auto";
    area.style.height = `${area.scrollHeight + 2}px`;
  };
  area.addEventListener("input", () => {
    block.text = area.value;
    block.dirty = true;
    entry.markDirty();
    grow();
  });
  // Blur, not input: relabelling on every keystroke would move the cursor around.
  area.addEventListener("blur", () => {
    const label = area.closest(".ed-block")?.querySelector(".ed-block__label");
    if (label) label.textContent = blockLabel(block);
  });
  requestAnimationFrame(grow);

  const wrap = (before: string, after = before) => {
    const { selectionStart: start, selectionEnd: end, value } = area;
    area.value = `${value.slice(0, start)}${before}${value.slice(start, end)}${after}${value.slice(end)}`;
    area.setSelectionRange(start + before.length, end + before.length);
    area.focus();
    area.dispatchEvent(new Event("input"));
  };
  const prefixLines = (prefix: string) => {
    const { selectionStart: start, selectionEnd: end, value } = area;
    const from = value.lastIndexOf("\n", start - 1) + 1;
    const to = value.indexOf("\n", end) === -1 ? value.length : value.indexOf("\n", end);
    const lines = value.slice(from, to).split("\n").map((line) => (line.startsWith(prefix) ? line.slice(prefix.length) : prefix + line));
    area.value = value.slice(0, from) + lines.join("\n") + value.slice(to);
    area.focus();
    area.dispatchEvent(new Event("input"));
  };

  const tool = (label: string, action: () => void, title: string) =>
    el("button", { type: "button", class: "ed-mini", title, onclick: action }, label);

  return el(
    "div",
    { class: "ed-block__body" },
    el(
      "div",
      { class: "ed-mini-bar" },
      tool("H2", () => prefixLines("## "), "Título de sección"),
      tool("H3", () => prefixLines("### "), "Subtítulo"),
      tool("B", () => wrap("**"), "Negrita"),
      tool("I", () => wrap("*"), "Cursiva"),
      tool("•", () => prefixLines("- "), "Lista"),
      tool("1.", () => prefixLines("1. "), "Lista numerada"),
      tool("”", () => prefixLines("> "), "Cita"),
      tool("</>", () => wrap("`"), "Código"),
      tool("∑", () => wrap("$"), "Matemáticas (KaTeX)"),
      tool("🔗", () => wrap("[", "](https://)"), "Enlace"),
      tool("⤵", () => splitBlock(entry, block, area, redraw), "Dividir en dos bloques por el cursor"),
    ),
    area,
  );
}

function splitBlock(entry: EntryCtx, block: TextBlock, area: HTMLTextAreaElement, redraw: () => void) {
  const at = area.selectionStart;
  const rest = area.value.slice(at).replace(/^\n+/, "");
  block.text = area.value.slice(0, at).replace(/\n+$/, "");
  block.dirty = true;
  const index = entry.blocks.indexOf(block);
  entry.blocks.splice(index + 1, 0, emptyText(rest));
  entry.markDirty();
  redraw();
}

// --- Photos

function photosEditor(entry: EntryCtx, block: PhotosBlock, redraw: () => void) {
  const change = () => {
    block.dirty = true;
    entry.markDirty();
  };

  const layout = el(
    "select",
    { class: "ed-select", onchange: () => {
        block.layout = (layout as HTMLSelectElement).value as PhotosBlock["layout"];
        if (block.layout === "gallery") block.cols ??= Math.min(3, Math.max(1, block.photos.length));
        change();
        redraw();
      } },
    el("option", { value: "auto", selected: block.layout === "auto" }, block.photos.length > 1 ? "Automático (una fila)" : "Foto suelta"),
    el("option", { value: "gallery", selected: block.layout === "gallery" }, "Galería (cuadrícula)"),
    el("option", { value: "carousel", selected: block.layout === "carousel" }, "Carrusel"),
  ) as HTMLSelectElement;

  const cols = el("select", { class: "ed-select", disabled: block.layout !== "gallery", onchange: () => {
      block.cols = Number((cols as HTMLSelectElement).value) || null;
      change();
    } },
    ...[1, 2, 3, 4].map((n) => el("option", { value: String(n), selected: (block.cols ?? 3) === n }, `${n} columna${n > 1 ? "s" : ""}`)),
  ) as HTMLSelectElement;

  const ratio = el("select", { class: "ed-select", disabled: block.layout === "auto", onchange: () => {
      block.ratio = (ratio as HTMLSelectElement).value || null;
      change();
    } },
    ...RATIOS.map((value) => el("option", { value, selected: (block.ratio ?? "") === value }, value === "" ? "Proporción por defecto" : value === "auto" ? "Cada foto con la suya" : value)),
  ) as HTMLSelectElement;

  const caption = el("input", { class: "ed-input", placeholder: "Pie de grupo (opcional)", value: block.caption, disabled: block.layout === "auto" });
  caption.addEventListener("input", () => {
    block.caption = (caption as HTMLInputElement).value;
    change();
  });

  const grid = el("div", { class: "ed-photos" });
  const drawPhotos = () => {
    fill(grid,
      ...block.photos.map((photo, i) => photoCard(entry, block, photo, i, () => {
          drawPhotos();
          const label = grid.closest(".ed-block")?.querySelector(".ed-block__label");
          if (label) label.textContent = blockLabel(block);
        })),
      dropZone(entry, block, drawPhotos),
    );
  };
  drawPhotos();

  return el(
    "div",
    { class: "ed-block__body" },
    el("div", { class: "ed-row" }, layout, block.layout === "gallery" ? cols : null, ratio, caption),
    grid,
  );
}

function photoCard(entry: EntryCtx, block: PhotosBlock, photo: Photo, index: number, redraw: () => void) {
  const change = () => {
    block.dirty = true;
    entry.markDirty();
  };

  const alt = el("input", { class: "ed-input ed-input--sm", placeholder: "Texto alternativo", value: photo.alt });
  alt.addEventListener("input", () => {
    photo.alt = (alt as HTMLInputElement).value;
    change();
  });
  const caption = el("input", { class: "ed-input ed-input--sm", placeholder: "Pie de foto", value: photo.caption });
  caption.addEventListener("input", () => {
    photo.caption = (caption as HTMLInputElement).value;
    change();
  });

  const card = el(
    "figure",
    { class: "ed-photo", draggable: true },
    el("img", { class: "ed-photo__img", src: photoUrl(photo.path), alt: "", loading: "lazy" }),
    el(
      "figcaption",
      { class: "ed-photo__body" },
      el("code", { class: "ed-photo__path", title: photo.path }, photo.path),
      alt,
      caption,
      geoControl(photo, change),
      el(
        "div",
        { class: "ed-photo__tools" },
        button("", () => {
            if (index > 0) {
              [block.photos[index - 1], block.photos[index]] = [block.photos[index], block.photos[index - 1]];
              change();
              redraw();
            }
          }, { variant: "ghost", iconPath: ICONS.left, title: "Mover antes" }),
        button("", () => {
            if (index < block.photos.length - 1) {
              [block.photos[index + 1], block.photos[index]] = [block.photos[index], block.photos[index + 1]];
              change();
              redraw();
            }
          }, { variant: "ghost", iconPath: ICONS.right, title: "Mover después" }),
        button("", () => {
            block.photos.splice(index, 1);
            change();
            redraw();
          }, { variant: "ghost", iconPath: ICONS.trash, title: "Quitar del bloque (el archivo se queda)" }),
      ),
    ),
  );

  card.addEventListener("dragstart", (event) => {
    (event as DragEvent).dataTransfer?.setData("text/ed-photo", String(index));
    card.classList.add("is-dragging");
  });
  card.addEventListener("dragend", () => card.classList.remove("is-dragging"));
  card.addEventListener("dragover", (event) => {
    if ((event as DragEvent).dataTransfer?.types.includes("text/ed-photo")) {
      event.preventDefault();
      card.classList.add("is-over");
    }
  });
  card.addEventListener("dragleave", () => card.classList.remove("is-over"));
  card.addEventListener("drop", (event) => {
    card.classList.remove("is-over");
    const from = Number((event as DragEvent).dataTransfer?.getData("text/ed-photo"));
    if (!Number.isInteger(from) || from === index) return;
    event.preventDefault();
    const [moved] = block.photos.splice(from, 1);
    block.photos.splice(index, 0, moved);
    change();
    redraw();
  });
  return card;
}

/** Where the photo sits on the route map: the "#" fragment of its path. */
function geoControl(photo: Photo, change: () => void) {
  const parse = (geo: string) => {
    if (geo === "off") return { mode: "off", value: "" };
    if (geo.startsWith("km=")) return { mode: "km", value: geo.slice(3) };
    if (geo.startsWith("time=")) return { mode: "time", value: geo.slice(5) };
    if (/^-?\d/.test(geo)) return { mode: "coords", value: geo };
    return { mode: "", value: "" };
  };
  const current = parse(photo.geo);

  const value = el("input", { class: "ed-input ed-input--sm", value: current.value, placeholder: current.mode === "coords" ? "-33.75,-70.10" : current.mode === "time" ? "12:41" : "5.8", hidden: !current.mode || current.mode === "off" });
  const mode = el(
    "select",
    { class: "ed-select ed-select--sm", title: "Dónde aparece la foto en el mapa de la ruta" },
    ...GEO_MODES.map((option) => el("option", { value: option.id, selected: current.mode === option.id }, option.label)),
  ) as HTMLSelectElement;

  const apply = () => {
    const selected = mode.value;
    value.hidden = !selected || selected === "off";
    const text = (value as HTMLInputElement).value.trim();
    photo.geo = selected === "off" ? "off" : selected === "km" ? (text ? `km=${text}` : "") : selected === "time" ? (text ? `time=${text}` : "") : selected === "coords" ? text : "";
    change();
  };
  mode.addEventListener("change", apply);
  value.addEventListener("input", apply);
  return el("div", { class: "ed-row ed-row--tight" }, mode, value);
}

function dropZone(entry: EntryCtx, block: PhotosBlock, redraw: () => void) {
  const input = el("input", { type: "file", accept: "image/*", multiple: true, class: "ed-hidden" }) as HTMLInputElement;
  const add = async (files: File[]) => {
    const photos = await uploadPhotos(entry, files);
    if (!photos.length) return;
    block.photos.push(...photos);
    block.dirty = true;
    entry.markDirty();
    redraw();
  };
  input.addEventListener("change", () => {
    void add([...(input.files ?? [])]);
    input.value = "";
  });
  const zone = el(
    "label",
    { class: "ed-photo-drop" },
    icon(ICONS.plus, 22),
    el("span", {}, "Añadir fotos"),
    el("small", {}, `a public/assets/img/posts/${entry.folder}/`),
    input,
  );
  zone.addEventListener("dragover", (event) => {
    event.preventDefault();
    zone.classList.add("is-over");
  });
  zone.addEventListener("dragleave", () => zone.classList.remove("is-over"));
  zone.addEventListener("drop", (event) => {
    event.preventDefault();
    zone.classList.remove("is-over");
    void add([...((event as DragEvent).dataTransfer?.files ?? [])].filter((file) => file.type.startsWith("image/")));
  });
  return zone;
}

/** Upload dropped files into this entry's photo folder and turn them into photo entries. */
export async function uploadPhotos(entry: EntryCtx, files: File[]): Promise<Photo[]> {
  const photos: Photo[] = [];
  for (const file of files) {
    try {
      const result = await api.uploadFile(file, { kind: "photo", folder: entry.folder });
      photos.push({ path: result.relPath ?? result.publicPath, alt: "", caption: "", geo: "" });
    } catch (error) {
      fail(`No se pudo subir ${file.name}`, error);
    }
  }
  if (photos.length) toast(`${photos.length} foto(s) a public/assets/img/posts/${entry.folder}/`, "ok");
  return photos;
}
