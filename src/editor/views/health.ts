// The checks: broken photo and route paths, missing descriptions, stale translations and
// photos nobody uses — the things that only surface at build time, or never.
import * as api from "../api";
import { actionsSlot, type Ctx } from "../app";
import { button, confirmDialog, el, fail, fill, ICONS, toast } from "../dom";
import { entryRoute, runTask } from "./dashboard";
import { summitRoute } from "./summits";

const LEVELS = { error: "Errores", warn: "Avisos", info: "Sugerencias" } as const;

export async function healthView(ctx: Ctx): Promise<Node> {
  const report = await api.get<api.HealthReport>("/health");
  ctx.heading("Revisión", `${report.counts.entries} entradas · ${report.counts.errors} errores · ${report.counts.warnings} avisos`);

  fill(actionsSlot(),
    button("Build completo", () => void runTask("build", [], "npm run build"), { variant: "primary", title: "La comprobación definitiva: compila el sitio igual que el deploy" }),
    button("Traducir lo pendiente", () => void runTask("translate", [], "npm run translate"), { title: "Traduce todo lo que falte o esté desactualizado" }),
    button("Recargar", () => void ctx.refresh(), { variant: "ghost", iconPath: ICONS.refresh }),
  );

  const groups = (["error", "warn", "info"] as const).map((level) => {
    const issues = report.issues.filter((issue) => issue.level === level);
    return el(
      "section",
      { class: "ed-card" },
      el("header", { class: "ed-card__head" }, el("h2", {}, LEVELS[level], el("span", { class: `ed-chip ${level === "error" ? "ed-chip--warn" : ""}` }, String(issues.length)))),
      issues.length === 0
        ? el("p", { class: "ed-empty" }, "Nada que reportar")
        : el(
            "ul",
            { class: "ed-list ed-list--tight" },
            ...issues.map((issue) =>
              el(
                "li",
                { class: "ed-list__row" },
                el("span", { class: "ed-chip ed-chip--soft" }, issue.kind),
                // A summit is not an entry: its file opens in the registry, not the post editor.
                el(
                  "a",
                  { class: "ed-list__main", href: `#/${issue.collection === "summits" && issue.key ? summitRoute(issue.key) : entryRoute(issue.path)}` },
                  el("span", { class: "ed-list__title" }, issue.title ?? issue.path),
                  el("span", { class: "ed-list__meta" }, issue.message),
                ),
              ),
            ),
          ),
    );
  });

  const unused = el(
    "section",
    { class: "ed-card" },
    el(
      "header",
      { class: "ed-card__head" },
      el("h2", {}, "Fotos sin usar", el("span", { class: "ed-chip" }, String(report.unusedPhotos.length))),
      report.unusedPhotos.length
        ? button("Mover todas a la papelera", () => void removeAll(ctx, report.unusedPhotos), { variant: "danger", iconPath: ICONS.trash })
        : null,
    ),
    report.unusedPhotos.length === 0
      ? el("p", { class: "ed-empty" }, "Ninguna")
      : el("ul", { class: "ed-list ed-list--tight" }, ...report.unusedPhotos.map((path) => el("li", { class: "ed-list__row" }, el("code", { class: "ed-grow" }, path)))),
  );

  return el("div", { class: "ed-stack" }, ...groups, unused);
}

async function removeAll(ctx: Ctx, paths: string[]) {
  const ok = await confirmDialog({
    title: `¿Mover ${paths.length} foto(s) a la papelera?`,
    message: `Ninguna entrada las menciona. Se mueven a ${ctx.state.localDir}/trash, así que se pueden recuperar.`,
    confirm: "Mover a la papelera",
    danger: true,
  });
  if (!ok) return;
  try {
    await api.post("/media/delete", { paths });
    toast(`${paths.length} archivo(s) a la papelera`, "ok");
    await ctx.refresh();
  } catch (error) {
    fail("No se pudieron borrar", error);
  }
}
