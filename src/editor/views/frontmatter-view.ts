// The frontmatter as a form. Every field maps to a key of the collection's zod schema
// (src/content/config.ts); keys the form does not know are listed at the bottom and kept
// exactly as they are in the file.
import * as api from "../api";
import { button, dialog, el, fail, field, fill, formatBytes, icon, ICONS, toast } from "../dom";
import { mapPicker } from "../map-picker";
import { photoUrl } from "../media-url";
import { FIELDS, TIMEZONES, type Field } from "../schema";
import type { EntryCtx } from "./entry";

export function frontmatterView(entry: EntryCtx): Node {
  const fields = FIELDS[entry.collection.id] ?? FIELDS.blog;
  const known = new Set([...fields.map((f) => f.key), "translationHash"]);
  const extra = entry.fm.keys().filter((key) => !known.has(key));

  return el(
    "div",
    { class: "ed-pane ed-form" },
    ...fields.map((spec) => renderField(entry, spec)),
    extra.length
      ? el(
          "div",
          { class: "ed-note" },
          "Otras claves del archivo, que el formulario no toca: ",
          ...extra.map((key) => el("code", { class: "ed-chip ed-chip--soft" }, key)),
        )
      : null,
    el(
      "div",
      { class: "ed-form__footer" },
      el("p", { class: "ed-note" }, "Archivo: ", el("code", {}, entry.path), " · slug: ", el("code", {}, entry.key)),
      entry.fm.has("translationHash")
        ? el(
            "p",
            { class: "ed-note" },
            `translationHash: ${entry.fm.string("translationHash")} `,
            button("Quitar", () => {
                entry.fm.remove("translationHash");
                entry.markDirty();
                toast("Se quitará al guardar: npm run translate dejará de sobrescribir este archivo", "ok");
              }, { variant: "ghost" }),
          )
        : null,
    ),
  );
}

function renderField(entry: EntryCtx, spec: Field): Node {
  const set = (value: unknown) => {
    entry.fm.set(spec.key, value as never);
    entry.markDirty();
  };

  switch (spec.type) {
    case "textarea": {
      const area = el("textarea", { class: "ed-textarea ed-textarea--short", rows: 3 }, entry.fm.string(spec.key));
      area.addEventListener("input", () => set(area.value));
      return field(spec.label, area, spec.hint);
    }
    case "number": {
      const input = el("input", { class: "ed-input", type: "number", step: spec.step ?? 1, value: entry.fm.number(spec.key) ?? "" });
      input.addEventListener("input", () => set(input.value === "" ? null : Number(input.value)));
      return field(spec.label, input, spec.hint);
    }
    case "date": {
      const input = el("input", { class: "ed-input", type: "date", value: entry.fm.string(spec.key).slice(0, 10) });
      input.addEventListener("input", () => set(input.value || null));
      return field(spec.label, input, spec.hint);
    }
    case "select": {
      const current = entry.fm.string(spec.key);
      const select = el("select", { class: "ed-select" }, ...(spec.options ?? []).map((option) => el("option", { value: option, selected: option === current }, option || "(ninguna)")));
      select.addEventListener("change", () => set((select as HTMLSelectElement).value || null));
      return field(spec.label, select, spec.hint);
    }
    case "timezone": {
      const current = entry.fm.string(spec.key);
      const options = TIMEZONES.includes(current) ? TIMEZONES : [...TIMEZONES, current];
      const select = el("select", { class: "ed-select" }, ...options.map((zone) => el("option", { value: zone, selected: zone === current }, zone || "America/Santiago (por defecto)")));
      select.addEventListener("change", () => set((select as HTMLSelectElement).value || null));
      return field(spec.label, select, spec.hint);
    }
    case "checkbox": {
      const input = el("input", { type: "checkbox", class: "ed-check__box", checked: entry.fm.get(spec.key) === true }) as HTMLInputElement;
      // An unticked box removes the key rather than writing `false`, except where the file
      // states it either way (a summit's `panorama`, which the scripts read as a switch).
      const explicit = spec.key === "panorama";
      input.addEventListener("change", () => set(input.checked ? true : explicit ? false : null));
      return el(
        "div",
        { class: "ed-field" },
        el("label", { class: "ed-check" }, input, el("span", { class: "ed-field__label" }, spec.label)),
        spec.hint && el("span", { class: "ed-field__hint" }, spec.hint),
      );
    }
    case "summits":
      return field(spec.label, summitsField(entry, spec.key), spec.hint);
    case "photos":
      return field(spec.label, photosField(entry, spec.key), spec.hint);
    case "tags":
      return field(spec.label, tagsEditor(entry, spec.key), spec.hint);
    case "image":
      return field(spec.label, imageField(entry, spec.key), spec.hint);
    case "coords":
      return coordsField(entry, spec);
    case "routes":
      return field(spec.label, routesField(entry), spec.hint);
    case "waypoints":
      return field(spec.label, waypointsSummary(entry), spec.hint);
    default: {
      const input = el("input", { class: "ed-input", value: entry.fm.string(spec.key), placeholder: spec.placeholder ?? "" });
      input.addEventListener("input", () => set(input.value || null));
      return field(spec.label + (spec.required ? " *" : ""), input, spec.hint);
    }
  }
}

// --- Tags

function tagsEditor(entry: EntryCtx, key: string) {
  const all = [...new Set(entry.app.state.collections.flatMap((c) => c.entries.flatMap((e) => e.tags)))].sort();
  const wrap = el("div", { class: "ed-tags" });
  const input = el("input", { class: "ed-input ed-input--sm", placeholder: "Añadir tag + Enter", list: "ed-taglist" });

  const draw = () => {
    const tags = entry.fm.list(key);
    fill(wrap,
      ...tags.map((tag, i) =>
        el(
          "span",
          { class: "ed-tag" },
          tag,
          el("button", { type: "button", class: "ed-tag__x", title: `Quitar ${tag}`, onclick: () => {
                const next = [...tags];
                next.splice(i, 1);
                entry.fm.set(key, next);
                entry.markDirty();
                draw();
              } }, "×"),
        ),
      ),
      input,
      el("datalist", { id: "ed-taglist" }, ...all.map((tag) => el("option", { value: tag }))),
    );
    input.focus({ preventScroll: true });
  };

  input.addEventListener("keydown", (event) => {
    const key2 = (event as KeyboardEvent).key;
    if (key2 !== "Enter" && key2 !== ",") return;
    event.preventDefault();
    const value = input.value.trim();
    input.value = "";
    if (!value) return;
    const tags = entry.fm.list(key);
    if (tags.includes(value)) return;
    entry.fm.set(key, [...tags, value]);
    entry.markDirty();
    draw();
  });
  draw();
  // Don't steal the focus on the first draw.
  input.blur();
  return wrap;
}

// --- Hero image

function imageField(entry: EntryCtx, key: string) {
  const row = el("div", { class: "ed-row" });
  const draw = () => {
    const value = entry.fm.string(key);
    const input = el("input", { class: "ed-input", value, placeholder: "/assets/img/headers/…" });
    input.addEventListener("input", () => {
      entry.fm.set(key, input.value || null);
      entry.markDirty();
    });
    fill(row,
      value ? el("img", { class: "ed-thumb", src: value.startsWith("/") || /^https?:/.test(value) ? value : photoUrl(value), alt: "" }) : null,
      input,
      button("Elegir…", () => void chooseImage(entry).then((picked) => {
            if (!picked) return;
            entry.fm.set(key, picked);
            entry.markDirty();
            draw();
          }), { iconPath: ICONS.image }),
      value ? button("", () => {
            entry.fm.remove(key);
            entry.markDirty();
            draw();
          }, { variant: "ghost", iconPath: ICONS.trash, title: "Quitar" }) : null,
    );
  };
  draw();
  return row;
}

/**
 * Pick an existing image, or upload one. It always returns a public path
 * ("/assets/img/..."), which is what `heroImage` is: the layouts use it as the src as is.
 */
export async function chooseImage(entry: EntryCtx): Promise<string | null> {
  const library = await api.get<api.MediaLibrary>("/media");
  return (await dialog((close) => {
    const grid = el("div", { class: "ed-picker" });
    const show = (files: api.MediaFile[]) => {
      fill(grid,
        ...files.map((file) =>
          el(
            "button",
            { type: "button", class: "ed-picker__item", onclick: () => close(`/${file.path.replace(/^public\//, "")}`) },
            el("img", { src: `/${file.path.replace(/^public\//, "")}`, alt: "", loading: "lazy" }),
            el("span", {}, file.relPath ?? file.name),
            el("small", {}, formatBytes(file.size)),
          ),
        ),
      );
    };
    show(library.headers);

    const input = el("input", { type: "file", accept: "image/*", class: "ed-hidden" }) as HTMLInputElement;
    input.addEventListener("change", async () => {
      const file = input.files?.[0];
      if (!file) return;
      try {
        const kind = (tabs.querySelector("button.is-active") as HTMLElement)?.dataset.kind === "photo" ? "photo" : "header";
        const result = await api.uploadFile(file, { kind, folder: kind === "photo" ? entry.folder : undefined });
        close(result.publicPath);
      } catch (error) {
        fail("No se pudo subir la imagen", error);
      }
    });

    const tabs = el("div", { class: "ed-tabs ed-tabs--inline" });
    const tab = (label: string, kind: "header" | "photo", files: api.MediaFile[], active = false) => {
      const node = el("button", { type: "button", class: `ed-tab ${active ? "is-active" : ""}`, dataset: { kind }, onclick: () => {
            for (const other of tabs.querySelectorAll("button")) other.classList.remove("is-active");
            node.classList.add("is-active");
            show(files);
          } }, label);
      return node;
    };
    tabs.append(tab("Portadas", "header", library.headers, true), tab("Fotos de posts", "photo", library.posts));

    return el(
      "div",
      { class: "ed-dialog__body" },
      el("h2", { class: "ed-dialog__title" }, "Elegir imagen"),
      tabs,
      grid,
      el(
        "div",
        { class: "ed-dialog__actions" },
        el("label", { class: "ed-btn" }, icon(ICONS.plus), el("span", {}, "Subir una…"), input),
        button("Cancelar", () => close(null)),
      ),
    );
  }, { width: "min(70rem, 94vw)" })) as string | null;
}

// --- Coordinates

function coordsField(entry: EntryCtx, spec: Field) {
  const [latKey, lonKey] = spec.keys ?? ["latitude", "longitude"];
  const lat = el("input", { class: "ed-input", type: "number", step: 0.00001, value: entry.fm.number(latKey) ?? "", placeholder: latKey });
  const lon = el("input", { class: "ed-input", type: "number", step: 0.00001, value: entry.fm.number(lonKey) ?? "", placeholder: lonKey });
  const sync = () => {
    entry.fm.set(latKey, lat.value === "" ? null : Number(lat.value));
    entry.fm.set(lonKey, lon.value === "" ? null : Number(lon.value));
    entry.markDirty();
  };
  lat.addEventListener("input", sync);
  lon.addEventListener("input", sync);

  const holder = el("div", { class: "ed-map-holder", hidden: true });
  const toggle = button("Mapa", () => {
      holder.hidden = !holder.hidden;
      if (!holder.hidden && holder.childElementCount === 0) {
        holder.appendChild(
          mapPicker({
            lat: entry.fm.number(latKey),
            lon: entry.fm.number(lonKey),
            onPick: (a, b) => {
              lat.value = String(a);
              lon.value = String(b);
              sync();
            },
          }),
        );
      }
    }, { iconPath: ICONS.map, title: "Elegir en el mapa" });

  return el("div", { class: "ed-field" }, el("span", { class: "ed-field__label" }, spec.label), el("div", { class: "ed-row" }, lat, lon, toggle), spec.hint && el("span", { class: "ed-field__hint" }, spec.hint), holder);
}

// --- route: the GPX files of an ascent

function routesField(entry: EntryCtx) {
  const wrap = el("div", { class: "ed-stack ed-stack--tight" });
  const draw = () => {
    const files = entry.fm.list("route");
    const update = (next: string[]) => {
      entry.fm.set("route", next);
      entry.markDirty();
      draw();
    };
    fill(wrap,
      ...files.map((file, i) =>
        el(
          "div",
          { class: "ed-row ed-row--tight" },
          el("span", { class: "ed-chip" }, String(i + 1)),
          el("code", { class: "ed-grow" }, file),
          button("", () => {
              const next = [...files];
              [next[i - 1], next[i]] = [next[i], next[i - 1]];
              if (i > 0) update(next);
            }, { variant: "ghost", iconPath: ICONS.up, title: "Subir" }),
          button("", () => {
              const next = [...files];
              if (i < files.length - 1) {
                [next[i + 1], next[i]] = [next[i], next[i + 1]];
                update(next);
              }
            }, { variant: "ghost", iconPath: ICONS.down, title: "Bajar" }),
          button("", () => update(files.filter((_, j) => j !== i)), { variant: "ghost", iconPath: ICONS.trash, title: "Quitar de la entrada" }),
        ),
      ),
      el(
        "div",
        { class: "ed-row ed-row--tight" },
        gpxPicker(entry, (picked) => update([...files, picked])),
        button("Abrir en Ruta", () => entry.openTab("route"), { variant: "ghost", iconPath: ICONS.map }),
      ),
      files.length > 1 ? el("p", { class: "ed-note" }, "Los archivos se unen en orden cronológico, no en el de esta lista.") : null,
    );
  };
  draw();
  return wrap;
}

/** A select of the GPX files in public/, plus uploading a new one. */
export function gpxPicker(entry: EntryCtx, onPick: (publicPath: string) => void) {
  const select = el(
    "select",
    { class: "ed-select" },
    el("option", { value: "" }, "Añadir un .gpx de public/…"),
    ...entry.app.state.gpx.map((file) => el("option", { value: file.publicPath }, `${file.publicPath} (${formatBytes(file.size)})`)),
  ) as HTMLSelectElement;
  select.addEventListener("change", () => {
    if (select.value) onPick(select.value);
    select.value = "";
  });

  const input = el("input", { type: "file", accept: ".gpx", class: "ed-hidden", multiple: true }) as HTMLInputElement;
  input.addEventListener("change", async () => {
    for (const file of [...(input.files ?? [])]) {
      try {
        const result = await api.uploadFile(file, { kind: "gpx" });
        toast(`Subido ${result.publicPath}`, "ok");
        onPick(result.publicPath);
      } catch (error) {
        fail(`No se pudo subir ${file.name}`, error);
      }
    }
    input.value = "";
    await entry.app.refresh();
  });

  return el("div", { class: "ed-row ed-row--tight" }, select, el("label", { class: "ed-btn" }, icon(ICONS.plus), el("span", {}, "Subir .gpx"), input));
}

// --- waypoints written in the frontmatter

function waypointsSummary(entry: EntryCtx) {
  const wrap = el("div", { class: "ed-stack ed-stack--tight" });
  const draw = () => {
    const points = entry.fm.maps("waypoints");
    fill(wrap,
      points.length === 0 ? el("p", { class: "ed-note" }, "Ninguno. Se añaden marcando puntos en la pestaña Ruta.") : null,
      ...points.map((point, i) =>
        el(
          "div",
          { class: "ed-row ed-row--tight" },
          el("span", { class: "ed-chip ed-chip--soft" }, String(point.kind ?? "note")),
          el("span", { class: "ed-grow" }, String(point.name ?? "")),
          el("code", {}, `${point.lat}, ${point.lon}`),
          button("", () => {
              const next = points.filter((_, j) => j !== i);
              entry.fm.set("waypoints", next);
              entry.markDirty();
              draw();
            }, { variant: "ghost", iconPath: ICONS.trash, title: "Quitar el punto" }),
        ),
      ),
      points.length ? button("Editar en el mapa", () => entry.openTab("route"), { variant: "ghost", iconPath: ICONS.map }) : null,
    );
  };
  draw();
  return wrap;
}

// --- summits: which tops of the registry an ascent reached

/**
 * The `summits` of an ascent, chosen from the registry rather than typed: the id has to match a
 * file in src/content/summits or the build fails, and the list is short. An id the registry does
 * not have is still shown, so a typo (or a summit deleted from under the post) is visible.
 */
function summitsField(entry: EntryCtx, key: string) {
  const registry = entry.app.state.summits;
  const wrap = el("div", { class: "ed-stack ed-stack--tight" });

  const draw = () => {
    const chosen = entry.fm.list(key);
    const update = (next: string[]) => {
      entry.fm.set(key, next);
      entry.markDirty();
      draw();
    };
    const missing = chosen.filter((id) => !registry.some((summit) => summit.id === id));
    const options = registry.filter((summit) => !chosen.includes(summit.id));

    fill(
      wrap,
      ...chosen.map((id) => {
        const summit = registry.find((item) => item.id === id);
        return el(
          "div",
          { class: "ed-row ed-row--tight" },
          el("a", { class: "ed-chip", href: `#/summits/${id}`, title: "Abrir la cumbre en el registro" }, id),
          summit
            ? el("span", { class: "ed-grow" }, `${summit.name} · ${summit.elevation?.toLocaleString("es-CL") ?? "?"} m`)
            : el("span", { class: "ed-grow ed-warn" }, "no existe en src/content/summits"),
          summit?.panorama ? el("span", { class: "ed-chip ed-chip--ok", title: "Tiene panorama calculado" }, "panorama") : null,
          summit?.wants && !summit.panorama ? el("span", { class: "ed-chip ed-chip--warn", title: "panorama: true, pero todavía sin calcular" }, "sin calcular") : null,
          button("", () => update(chosen.filter((item) => item !== id)), { variant: "ghost", iconPath: ICONS.trash, title: "Quitar del post" }),
        );
      }),
      options.length
        ? (() => {
            const select = el(
              "select",
              { class: "ed-select" },
              el("option", { value: "" }, "Añadir una cumbre del registro…"),
              ...options.map((summit) => el("option", { value: summit.id }, `${summit.name} — ${summit.elevation ?? "?"} m (${summit.id})`)),
            ) as HTMLSelectElement;
            select.addEventListener("change", () => {
              if (select.value) update([...chosen, select.value]);
            });
            return select;
          })()
        : el("p", { class: "ed-note" }, "Todas las cumbres del registro ya están en este post."),
      chosen.length > 1 ? el("p", { class: "ed-note" }, "Varias cumbres: una travesía. Se muestran en este orden.") : null,
      missing.length ? el("p", { class: "ed-note ed-warn" }, `Sin archivo en el registro: ${missing.join(", ")}. El build falla hasta crearlas o quitarlas.`) : null,
      entry.lang === "en"
        ? el("p", { class: "ed-note" }, "El sitio lee las cumbres del post en español; lo que se ponga aquí no se usa.")
        : el("p", { class: "ed-note" }, "¿Falta una cumbre? ", el("a", { href: "#/summits" }, "Créala en el registro")),
    );
  };
  draw();
  return wrap;
}

// --- photos: the ones taken from a summit, placed on its panorama by bearing

function photosField(entry: EntryCtx, key: string) {
  const wrap = el("div", { class: "ed-stack ed-stack--tight" });

  const draw = () => {
    const photos = entry.fm.maps(key);
    const update = (next: Record<string, string | number | boolean>[]) => {
      entry.fm.set(key, next);
      entry.markDirty();
      draw();
    };
    const edit = (i: number, field2: string, value: string | number | null) => {
      const next = photos.map((photo) => ({ ...photo }));
      if (value == null || value === "") delete next[i][field2];
      else next[i][field2] = value;
      entry.fm.set(key, next);
      entry.markDirty();
    };

    fill(
      wrap,
      photos.length === 0 ? el("p", { class: "ed-note" }, "Ninguna. Una foto con azimut queda marcada en la regla del panorama.") : null,
      ...photos.map((photo, i) => {
        const src = String(photo.src ?? "");
        const path = el("input", { class: "ed-input ed-grow", value: src, placeholder: "/assets/img/cumbres/…" });
        path.addEventListener("input", () => edit(i, "src", path.value));
        const azimuth = el("input", { class: "ed-input ed-input--sm", type: "number", min: 0, max: 360, step: 1, value: photo.azimuth ?? "", placeholder: "azimut" });
        azimuth.addEventListener("input", () => edit(i, "azimuth", azimuth.value === "" ? null : Number(azimuth.value)));
        const caption = el("input", { class: "ed-input", value: String(photo.caption ?? ""), placeholder: "Pie de foto" });
        caption.addEventListener("input", () => edit(i, "caption", caption.value));

        return el(
          "div",
          { class: "ed-photo ed-photo--summit" },
          src ? el("img", { class: "ed-thumb", src, alt: "", loading: "lazy" }) : null,
          el(
            "div",
            { class: "ed-stack ed-stack--tight ed-grow" },
            el("div", { class: "ed-row ed-row--tight" }, path, button("Elegir…", () => void chooseImage(entry).then((picked) => picked && update(photos.map((item, j) => (j === i ? { ...item, src: picked } : item)))), { iconPath: ICONS.image })),
            el("div", { class: "ed-row ed-row--tight" }, el("span", { class: "ed-field__label" }, "Azimut °"), azimuth, caption),
          ),
          el(
            "div",
            { class: "ed-photo__tools" },
            button("", () => {
                if (i === 0) return;
                const next = photos.map((item) => ({ ...item }));
                [next[i - 1], next[i]] = [next[i], next[i - 1]];
                update(next);
              }, { variant: "ghost", iconPath: ICONS.up, title: "Subir" }),
            button("", () => update(photos.filter((_, j) => j !== i)), { variant: "ghost", iconPath: ICONS.trash, title: "Quitar la foto" }),
          ),
        );
      }),
      button("Añadir una foto", () => update([...photos.map((photo) => ({ ...photo })), { src: "" }]), { iconPath: ICONS.plus }),
    );
  };
  draw();
  return wrap;
}
