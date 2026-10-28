// Search, filter chips and sorting for the lists of the mountaineering section (the log, the
// summits, the routes). Everything is already on the page; this only hides and reorders it.
//
// Markup, inside [data-filter-root]:
//   [data-filter-search]          text input; matched against each item's data-text
//   [data-filter-chip="flag"]     toggle buttons; an item needs every pressed flag in data-flags
//   [data-filter-sort]            <select> with "key:asc|desc" values, sorting by data-<key>
//   [data-filter-item]            the items, inside [data-filter-list] (or [data-filter-group]s,
//                                 which hide when nothing in them is left)
//   [data-filter-count], [data-filter-empty]

import { fold } from "../lib/search";

export function initLogFilter() {
  for (const root of document.querySelectorAll<HTMLElement>("[data-filter-root]:not([data-filter-ready])")) setUp(root);
}

function setUp(root: HTMLElement) {
  root.dataset.filterReady = "";
  const search = root.querySelector<HTMLInputElement>("[data-filter-search]");
  const sort = root.querySelector<HTMLSelectElement>("[data-filter-sort]");
  const chips = [...root.querySelectorAll<HTMLButtonElement>("[data-filter-chip]")];
  const items = [...root.querySelectorAll<HTMLElement>("[data-filter-item]")];
  const groups = [...root.querySelectorAll<HTMLElement>("[data-filter-group]")];
  const count = root.querySelector<HTMLElement>("[data-filter-count]");
  const empty = root.querySelector<HTMLElement>("[data-filter-empty]");

  const apply = () => {
    const words = fold(search?.value ?? "").split(/\s+/).filter(Boolean);
    const flags = chips.filter((chip) => chip.getAttribute("aria-pressed") === "true").map((chip) => chip.dataset.filterChip!);
    let shown = 0;
    for (const item of items) {
      const text = item.dataset.text ?? "";
      const own = (item.dataset.flags ?? "").split(" ");
      const on = words.every((word) => text.includes(word)) && flags.every((flag) => own.includes(flag));
      item.hidden = !on;
      if (on) shown++;
    }
    for (const group of groups) group.hidden = !group.querySelector("[data-filter-item]:not([hidden])");
    if (empty) empty.hidden = shown > 0;
    if (count) count.textContent = (shown === 1 ? count.dataset.labelOne : count.dataset.labelMany)?.replace("{n}", String(shown)) ?? "";
  };

  const reorder = () => {
    if (!sort) return;
    const [key, direction] = sort.value.split(":");
    const sign = direction === "desc" ? -1 : 1;
    for (const list of root.querySelectorAll<HTMLElement>("[data-filter-list]")) {
      const children = [...list.querySelectorAll<HTMLElement>(":scope > [data-filter-item]")];
      children.sort((a, b) => {
        const x = a.dataset[key] ?? "";
        const y = b.dataset[key] ?? "";
        const nx = Number(x), ny = Number(y);
        const order = Number.isFinite(nx) && Number.isFinite(ny) && x !== "" && y !== "" ? nx - ny : x.localeCompare(y, document.documentElement.lang);
        return order * sign;
      });
      list.append(...children);
    }
  };

  search?.addEventListener("input", apply);
  sort?.addEventListener("change", reorder);
  for (const chip of chips) {
    chip.addEventListener("click", () => {
      chip.setAttribute("aria-pressed", String(chip.getAttribute("aria-pressed") !== "true"));
      apply();
    });
  }
  // Sortable table headers: a button per column, pointing at a sort option.
  for (const header of root.querySelectorAll<HTMLButtonElement>("[data-sort-by]")) {
    header.addEventListener("click", () => {
      if (!sort) return;
      const key = header.dataset.sortBy!;
      const [current, direction] = sort.value.split(":");
      sort.value = `${key}:${current === key && direction === "desc" ? "asc" : "desc"}`;
      for (const other of root.querySelectorAll<HTMLElement>("[data-sort-by]")) {
        other.closest("th")?.setAttribute("aria-sort", other === header ? (sort.value.endsWith("desc") ? "descending" : "ascending") : "none");
      }
      reorder();
    });
  }
  reorder();
  apply();
}
