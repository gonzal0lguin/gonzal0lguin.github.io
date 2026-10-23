// The photos of one entry: the folder they live in, which ones the post actually uses, and
// where each one sits on the route map.
import * as api from "../api";
import { parseGpx, stitchTracks } from "../../lib/gpx-core";
import { type Photo, emptyPhotos } from "../blocks";
import { button, confirmDialog, dialog, el, fail, fill, formatBytes, icon, ICONS, toast } from "../dom";
import { mapPicker } from "../map-picker";
import { isRemoteMedia, photoUrl } from "../media-url";
import { runTask } from "./dashboard";
import type { EntryCtx } from "./entry";

export async function photosView(entry: EntryCtx): Promise<Node> {
  const library = await api.get<api.MediaLibrary>("/media");
  const folder = entry.folder;
  const inFolder = library.posts.filter((file) => file.folder === folder);

  const used = () => {
    const list: { photo: Photo; blockIndex: number }[] = [];
    entry.blocks.forEach((block, blockIndex) => {
      if (block.type === "photos") for (const photo of block.photos) list.push({ photo, blockIndex });
    });
    return list;
  };

  /** The route's points, loaded once, so a photo can be placed against the track. */
  let track: [number, number][] | null = null;
  const loadTrack = async () => {
    if (track) return track;
    const files = entry.fm.list("route");
    if (!files.length) return null;
    try {
      const parts = await Promise.all(files.map((path) => api.get<{ xml: string }>("/gpx", { path })));
      const stitched = stitchTracks(parts.map((part) => parseGpx(part.xml)));
      track = stitched.points.filter((_, i) => i % 5 === 0).map((point) => [point.lat, point.lon] as [number, number]);
    } catch (error) {
      fail("No se pudo leer la ruta", error);
      return null;
    }
    return track;
  };

  const usedList = el("div", { class: "ed-stack ed-stack--tight" });
  const drawUsed = () => {
    const list = used();
    fill(usedList,
      list.length === 0 ? el("p", { class: "ed-note" }, "El contenido todavía no usa ninguna foto.") : null,
      ...list.map(({ photo, blockIndex }) =>
        el(
          "div",
          { class: "ed-used" },
          el("img", { class: "ed-thumb", src: photoUrl(photo.path), alt: "", loading: "lazy" }),
          el(
            "div",
            { class: "ed-used__body" },
            el("code", {}, photo.path),
            el("span", { class: "ed-used__meta" }, `bloque ${blockIndex + 1}`, photo.caption ? ` · “${photo.caption}”` : "", photo.geo ? ` · #${photo.geo}` : " · sin ubicación propia"),
          ),
          button("Ubicar…", () => void placeOnMap(entry, photo, loadTrack, drawUsed), { variant: "ghost", iconPath: ICONS.map, title: "Elegir en el mapa dónde se tomó" }),
          button("Ir al bloque", () => entry.openTab("content"), { variant: "ghost", iconPath: ICONS.back }),
        ),
      ),
    );
  };
  drawUsed();

  const grid = el("div", { class: "ed-picker" });
  const drawFolder = () => {
    const usedPaths = new Set(used().map((item) => item.photo.path));
    fill(grid,
      ...inFolder.map((file) =>
        el(
          "div",
          { class: `ed-picker__item ${usedPaths.has(file.relPath ?? "") ? "is-used" : ""}` },
          el("img", { src: `/${file.path.replace(/^public\//, "")}`, alt: "", loading: "lazy" }),
          el("span", {}, file.name),
          el("small", {}, formatBytes(file.size), usedPaths.has(file.relPath ?? "") ? " · en el post" : ""),
          el(
            "div",
            { class: "ed-picker__tools" },
            button("Insertar", () => {
                const photo: Photo = { path: file.relPath ?? "", alt: "", caption: "", geo: "" };
                entry.blocks.push(emptyPhotos("auto", [photo]));
                entry.markDirty();
                drawUsed();
                drawFolder();
                toast("Añadida al final del contenido", "ok");
              }, { variant: "ghost", iconPath: ICONS.plus }),
            button("", () => void removeFile(entry, file, () => {
                  inFolder.splice(inFolder.indexOf(file), 1);
                  drawFolder();
                }), { variant: "ghost", iconPath: ICONS.trash, title: "Mover el archivo a la papelera del editor" }),
          ),
        ),
      ),
      uploadZone(entry, (added) => {
        inFolder.push(...added);
        drawFolder();
      }),
    );
  };
  drawFolder();

  return el(
    "div",
    { class: "ed-pane ed-stack" },
    el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, "Fotos usadas en el contenido"), button("Leer EXIF", () => void runTask("photos", [], "npm run photos"), { variant: "ghost", title: "npm run photos: guarda la ubicación y la hora EXIF de cada foto" })),
      usedList,
    ),
    el(
      "section",
      { class: "ed-card" },
      el(
        "header",
        { class: "ed-card__head" },
        el("h2", {}, `Carpeta public/assets/img/posts/${folder}/`, el("span", { class: "ed-chip" }, String(inFolder.length))),
      ),
      isRemoteMedia() ? el("p", { class: "ed-warn" }, `PUBLIC_MEDIA_BASE_URL apunta a ${library.mediaBaseUrl}: el sitio lee las fotos de allí, así que súbelas también a ese servidor.`) : null,
      grid,
    ),
  );
}

function uploadZone(entry: EntryCtx, onAdded: (files: api.MediaFile[]) => void) {
  const input = el("input", { type: "file", accept: "image/*", multiple: true, class: "ed-hidden" }) as HTMLInputElement;
  const add = async (files: File[]) => {
    const added: api.MediaFile[] = [];
    for (const file of files) {
      try {
        const result = await api.uploadFile(file, { kind: "photo", folder: entry.folder });
        added.push({ path: result.path, name: result.path.split("/").pop()!, relPath: result.relPath, folder: entry.folder, size: result.size, mtime: Date.now(), usedBy: [] });
      } catch (error) {
        fail(`No se pudo subir ${file.name}`, error);
      }
    }
    if (added.length) {
      toast(`${added.length} archivo(s) subido(s)`, "ok");
      onAdded(added);
    }
  };
  input.addEventListener("change", () => {
    void add([...(input.files ?? [])]);
    input.value = "";
  });
  const zone = el("label", { class: "ed-photo-drop" }, icon(ICONS.plus, 22), el("span", {}, "Subir fotos"), input);
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

async function removeFile(entry: EntryCtx, file: api.MediaFile, done: () => void) {
  const ok = await confirmDialog({
    title: `¿Borrar ${file.name}?`,
    message: file.usedBy.length ? `Lo mencionan: ${file.usedBy.join(", ")}. El archivo se mueve a ${entry.app.state.localDir}/trash.` : `El archivo se mueve a ${entry.app.state.localDir}/trash.`,
    confirm: "Mover a la papelera",
    danger: true,
  });
  if (!ok) return;
  try {
    await api.post("/media/delete", { paths: [file.path] });
    toast(`${file.name} a la papelera`, "ok");
    done();
  } catch (error) {
    fail("No se pudo borrar", error);
  }
}

/** Pick on a map where a photo was taken, writing it as the "#lat,lon" fragment. */
async function placeOnMap(entry: EntryCtx, photo: Photo, loadTrack: () => Promise<[number, number][] | null>, done: () => void) {
  const track = await loadTrack();
  const coords = photo.geo.match(/^(-?\d+(?:\.\d+)?),\s*(-?\d+(?:\.\d+)?)$/);
  await dialog((close) => {
    const picked = { lat: coords ? Number(coords[1]) : null, lon: coords ? Number(coords[2]) : null };
    const readout = el("code", {}, coords ? photo.geo : "pincha en el mapa");
    const map = mapPicker({
      lat: picked.lat,
      lon: picked.lon,
      track: track ?? undefined,
      onPick: (lat, lon) => {
        picked.lat = lat;
        picked.lon = lon;
        readout.textContent = `${lat},${lon}`;
      },
    });
    return el(
      "div",
      { class: "ed-dialog__body" },
      el("h2", { class: "ed-dialog__title" }, "Ubicar la foto"),
      el("p", { class: "ed-dialog__text" }, el("img", { class: "ed-thumb", src: photoUrl(photo.path), alt: "" }), " ", el("code", {}, photo.path)),
      map,
      el("p", { class: "ed-note" }, "Posición: ", readout, track ? " · la línea naranja es la ruta de la entrada" : ""),
      el(
        "div",
        { class: "ed-dialog__actions" },
        button("Quitar la ubicación", () => {
            photo.geo = "";
            markPhotoDirty(entry, photo);
            close(true);
            done();
          }, { variant: "ghost" }),
        button("Cancelar", () => close(false)),
        button("Guardar", () => {
            if (picked.lat != null && picked.lon != null) {
              photo.geo = `${picked.lat},${picked.lon}`;
              markPhotoDirty(entry, photo);
            }
            close(true);
            done();
          }, { variant: "primary" }),
      ),
    );
  }, { width: "min(60rem, 94vw)" });
}

function markPhotoDirty(entry: EntryCtx, photo: Photo) {
  for (const block of entry.blocks) {
    if (block.type === "photos" && block.photos.includes(photo)) block.dirty = true;
  }
  entry.markDirty();
}
