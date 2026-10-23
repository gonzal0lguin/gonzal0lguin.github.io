// What changed in the working tree, what each change looks like, and committing it.
// No pushing: that stays a deliberate step in the terminal.
import * as api from "../api";
import { actionsSlot, type Ctx } from "../app";
import { button, confirmDialog, el, fail, fill, ICONS, toast } from "../dom";
import { entryRoute } from "./dashboard";

export async function gitView(ctx: Ctx): Promise<Node> {
  const status = await api.get<api.GitStatus>("/git/status");
  ctx.heading("Cambios", `rama ${status.branch ?? "?"} · ${status.files.length} archivo(s)`);

  const selected = new Set(status.files.map((file) => file.path));
  const detail = el("pre", { class: "ed-diff" }, "Elige un archivo para ver el diff.");
  const message = el("input", { class: "ed-input", placeholder: "Mensaje del commit" });

  const rows = status.files.map((file) => {
    const check = el("input", { type: "checkbox", checked: true }) as HTMLInputElement;
    check.addEventListener("change", () => (check.checked ? selected.add(file.path) : selected.delete(file.path)));
    return el(
      "li",
      { class: "ed-list__row" },
      check,
      el("span", { class: `ed-chip ${file.untracked ? "ed-chip--soft" : file.unstaged === "deleted" ? "ed-chip--warn" : "ed-chip--ok"}` }, file.untracked ? "nuevo" : file.unstaged ?? file.staged ?? "?"),
      el("button", { type: "button", class: "ed-list__main", onclick: () => void showDiff(file.path) }, el("code", {}, file.path)),
      /^src\/content\/.*\.mdx?$/.test(file.path) ? el("a", { class: "ed-btn ed-btn--ghost", href: `#/${entryRoute(file.path)}` }, "Editar") : null,
      button("", () => void revert(file.path), { variant: "ghost", iconPath: ICONS.trash, title: file.untracked ? "Mover el archivo nuevo a la papelera" : "Descartar los cambios de este archivo" }),
    );
  });

  async function showDiff(path: string) {
    detail.textContent = "…";
    try {
      const result = await api.get<{ diff: string; untracked: boolean }>("/git/diff", { path });
      detail.textContent = result.diff || "(sin cambios de texto)";
    } catch (error) {
      fail("No se pudo leer el diff", error);
    }
  }

  async function revert(path: string) {
    const ok = await confirmDialog({
      title: `¿Descartar los cambios de ${path}?`,
      message: "Vuelve a la última versión commiteada (o a la papelera del editor si el archivo es nuevo). Esto no se puede deshacer desde git.",
      confirm: "Descartar",
      danger: true,
    });
    if (!ok) return;
    try {
      await api.post("/git/revert", { path });
      toast("Descartado", "ok");
      await ctx.refresh();
    } catch (error) {
      fail("No se pudo descartar", error);
    }
  }

  async function doCommit() {
    const paths = [...selected];
    if (!paths.length || !message.value.trim()) {
      toast("Hace falta un mensaje y al menos un archivo", "error");
      return;
    }
    try {
      const result = await api.post<{ output: string }>("/git/commit", { message: message.value, paths });
      toast(result.output.split("\n")[0] || "Commit hecho", "ok");
      message.value = "";
      await ctx.refresh();
    } catch (error) {
      fail("No se pudo hacer el commit", error);
    }
  }

  fill(actionsSlot(), button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }));

  return el(
    "div",
    { class: "ed-stack" },
    status.error ? el("p", { class: "ed-warn" }, status.error) : null,
    el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, "Sin commitear")),
      rows.length ? el("ul", { class: "ed-list ed-list--tight" }, ...rows) : el("p", { class: "ed-empty" }, "El árbol está limpio."),
      rows.length ? el("div", { class: "ed-row" }, message, button("Commit", () => void doCommit(), { variant: "primary", iconPath: ICONS.check })) : null,
      rows.length ? el("p", { class: "ed-note" }, "El editor no hace push: revisa el commit y súbelo tú (`git push`)." ) : null,
    ),
    el("section", { class: "ed-card" }, el("header", { class: "ed-card__head" }, el("h2", {}, "Diff")), detail),
    el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, "Últimos commits")),
      el("ul", { class: "ed-list ed-list--tight" }, ...status.log.map((entry) => el("li", { class: "ed-list__row" }, el("code", {}, entry.hash), el("span", { class: "ed-grow" }, entry.subject), el("span", { class: "ed-list__meta" }, entry.when)))),
    ),
  );
}
