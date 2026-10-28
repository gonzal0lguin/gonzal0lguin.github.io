// What the log's search box matches against: lower case, without accents, so "mocoen" finds
// "Mocoén" and "union" finds "Unión". src/scripts/log-filter.ts folds the query the same way.

export const fold = (text: string) =>
  text
    .normalize("NFD")
    .replace(/[̀-ͯ]/g, "")
    .toLowerCase();

export const searchText = (parts: (string | null | undefined)[]) => fold(parts.filter(Boolean).join(" "));
