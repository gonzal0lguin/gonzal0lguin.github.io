// The handful of DOM helpers the editor is built from. No framework: the views are plain
// functions that return elements.

type Child = Node | string | number | null | undefined | false | Child[];

export function el<K extends keyof HTMLElementTagNameMap>(
  tag: K,
  attrs: Record<string, unknown> = {},
  ...children: Child[]
): HTMLElementTagNameMap[K] {
  const node = document.createElement(tag);
  for (const [name, value] of Object.entries(attrs)) {
    if (value == null || value === false) continue;
    if (name === "class") node.className = String(value);
    else if (name === "dataset") Object.assign(node.dataset, value as Record<string, string>);
    else if (name === "style" && typeof value === "object") Object.assign(node.style, value as object);
    else if (name === "html") node.innerHTML = String(value);
    else if (name.startsWith("on") && typeof value === "function") node.addEventListener(name.slice(2).toLowerCase(), value as EventListener);
    else if (name in node && typeof value !== "string") {
      // Some DOM properties are read-only (input.list), so fall back to the attribute.
      try {
        (node as Record<string, unknown>)[name] = value;
      } catch {
        node.setAttribute(name, String(value));
      }
    } else node.setAttribute(name, String(value));
  }
  append(node, children);
  return node;
}

export function append(parent: Node, children: Child[]) {
  for (const child of children.flat(4)) {
    if (child == null || child === false) continue;
    parent.appendChild(child instanceof Node ? child : document.createTextNode(String(child)));
  }
}

export const clear = (node: Element) => void (node.textContent = "");

/** Replace a node's children. Unlike replaceChildren(), null/false children are dropped. */
export function fill<T extends Element>(parent: T, ...children: Child[]) {
  parent.textContent = "";
  append(parent, children);
  return parent;
}

/** A labelled form row, the shape every field in the editor uses. */
export function field(label: string, control: Node, hint?: string) {
  return el("label", { class: "ed-field" }, el("span", { class: "ed-field__label" }, label), control, hint && el("span", { class: "ed-field__hint" }, hint));
}

export function icon(path: string, size = 16) {
  const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
  svg.setAttribute("viewBox", "0 0 24 24");
  svg.setAttribute("width", String(size));
  svg.setAttribute("height", String(size));
  svg.setAttribute("fill", "none");
  svg.setAttribute("stroke", "currentColor");
  svg.setAttribute("stroke-width", "2");
  svg.setAttribute("stroke-linecap", "round");
  svg.setAttribute("stroke-linejoin", "round");
  svg.setAttribute("aria-hidden", "true");
  svg.innerHTML = `<path d="${path}"/>`;
  return svg;
}

export const ICONS = {
  save: "M5 4h11l3 3v13H5zM9 4v5h6V4M8 20v-5h8v5",
  plus: "M12 5v14M5 12h14",
  trash: "M4 7h16M9 7V4h6v3M6 7l1 13h10l1-13",
  up: "M12 19V5M5 12l7-7 7 7",
  down: "M12 5v14M5 12l7 7 7-7",
  close: "M6 6l12 12M18 6 6 18",
  image: "M4 5h16v14H4zM4 15l5-5 4 4 3-3 4 4",
  map: "M9 4 3 6v14l6-2 6 2 6-2V4l-6 2z M9 4v14M15 6v14",
  check: "M5 13l4 4L19 7",
  back: "M15 5l-7 7 7 7",
  external: "M14 4h6v6M20 4l-9 9M18 14v6H4V6h6",
  refresh: "M20 11a8 8 0 1 0-2.3 5.7M20 5v6h-6",
  eye: "M2 12s3.6-7 10-7 10 7 10 7-3.6 7-10 7S2 12 2 12zM12 9a3 3 0 1 0 0 6 3 3 0 0 0 0-6z",
  drag: "M9 6h.01M9 12h.01M9 18h.01M15 6h.01M15 12h.01M15 18h.01",
  rename: "M4 20h4L20 8l-4-4L4 16zM14 6l4 4",
  home: "M3 11l9-8 9 8M5 10v10h14V10M10 20v-6h4v6",
  mountain: "M2 18l5-7 3 4 4-7 8 10zM2 21h20",
  left: "M19 12H5m7-7-7 7 7 7",
  right: "M5 12h14m-7-7 7 7-7 7",
};

export function button(label: string, onClick: () => void, options: { variant?: string; iconPath?: string; title?: string } = {}) {
  const node = el(
    "button",
    { type: "button", class: `ed-btn ${options.variant ? `ed-btn--${options.variant}` : ""}`, title: options.title ?? label, onclick: onClick },
    options.iconPath && icon(options.iconPath),
    label && el("span", {}, label),
  );
  return node;
}

// --- Toasts

let toastHost: HTMLElement | null = null;

export function toast(message: string, kind: "info" | "ok" | "error" = "info", timeout = kind === "error" ? 9000 : 3500) {
  toastHost ??= document.body.appendChild(el("div", { class: "ed-toasts" }));
  const node = el("div", { class: `ed-toast ed-toast--${kind}` }, el("span", {}, message), el("button", { type: "button", class: "ed-toast__x", onclick: () => node.remove(), "aria-label": "Cerrar" }, "×"));
  toastHost.appendChild(node);
  if (timeout) setTimeout(() => node.remove(), timeout);
  return node;
}

/** Report a failed action without losing the detail: a toast plus the console. */
export function fail(context: string, error: unknown) {
  const message = error instanceof Error ? error.message : String(error);
  console.error(`[editor] ${context}`, error);
  toast(`${context}: ${message}`, "error");
}

// --- Dialogs

export function dialog(content: (close: (value?: unknown) => void) => Node, { width = "min(46rem, 92vw)" } = {}) {
  return new Promise<unknown>((resolve) => {
    const node = el("dialog", { class: "ed-dialog", style: { width } });
    const close = (value?: unknown) => {
      node.close();
      node.remove();
      resolve(value);
    };
    node.appendChild(content(close));
    node.addEventListener("cancel", (event) => {
      event.preventDefault();
      close(undefined);
    });
    document.body.appendChild(node);
    node.showModal();
  });
}

export function confirmDialog(options: { title: string; message?: string; confirm?: string; danger?: boolean }) {
  return dialog(
    (close) =>
      el(
        "div",
        { class: "ed-dialog__body" },
        el("h2", { class: "ed-dialog__title" }, options.title),
        options.message && el("p", { class: "ed-dialog__text" }, options.message),
        el(
          "div",
          { class: "ed-dialog__actions" },
          button("Cancelar", () => close(false)),
          button(options.confirm ?? "Confirmar", () => close(true), { variant: options.danger ? "danger" : "primary" }),
        ),
      ),
    { width: "min(30rem, 92vw)" },
  ) as Promise<boolean>;
}

export function promptDialog(options: { title: string; label: string; value?: string; confirm?: string; hint?: string }) {
  return dialog((close) => {
    const input = el("input", { class: "ed-input", value: options.value ?? "" });
    const form = el(
      "form",
      {
        class: "ed-dialog__body",
        onsubmit: (event: Event) => {
          event.preventDefault();
          close(input.value.trim() || null);
        },
      },
      el("h2", { class: "ed-dialog__title" }, options.title),
      field(options.label, input, options.hint),
      el("div", { class: "ed-dialog__actions" }, button("Cancelar", () => close(null)), el("button", { type: "submit", class: "ed-btn ed-btn--primary" }, options.confirm ?? "Aceptar")),
    );
    setTimeout(() => input.focus(), 30);
    return form;
  }, { width: "min(32rem, 92vw)" }) as Promise<string | null>;
}

/** A long-running task's output, streamed into a dialog. */
export function outputDialog(title: string) {
  const pre = el("pre", { class: "ed-output" });
  let closeFn: (() => void) | null = null;
  const done = dialog((close) => {
    closeFn = () => close(true);
    return el(
      "div",
      { class: "ed-dialog__body" },
      el("h2", { class: "ed-dialog__title" }, title),
      pre,
      el("div", { class: "ed-dialog__actions" }, button("Cerrar", () => close(true), { variant: "primary" })),
    );
  }, { width: "min(60rem, 94vw)" });
  return {
    write(text: string) {
      pre.textContent += text;
      pre.scrollTop = pre.scrollHeight;
    },
    close: () => closeFn?.(),
    done,
  };
}

export const debounce = <T extends (...args: never[]) => void>(fn: T, ms = 250) => {
  let timer: number | undefined;
  return (...args: Parameters<T>) => {
    clearTimeout(timer);
    timer = setTimeout(() => fn(...args), ms) as unknown as number;
  };
};

export const slugify = (text: string) =>
  text
    .normalize("NFD")
    .replace(/[̀-ͯ]/g, "")
    .toLowerCase()
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");

export const formatBytes = (bytes: number) => (bytes > 1024 * 1024 ? `${(bytes / 1024 / 1024).toFixed(1)} MB` : `${Math.max(1, Math.round(bytes / 1024))} kB`);

export const today = () => new Date().toISOString().slice(0, 10);
