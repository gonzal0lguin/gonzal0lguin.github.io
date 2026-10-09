// Project categories: the URL segment (/projects/<category>) and the value of the
// `category` frontmatter field. Labels live in src/i18n/ui.ts as "projects.cat.<category>".
export const PROJECT_CATEGORIES = ["academic-projects", "hobby-projects", "personal"] as const;
export type ProjectCategory = (typeof PROJECT_CATEGORIES)[number];

export const PROJECT_CATEGORY_IMAGES: Record<ProjectCategory, string> = {
  "academic-projects": "/assets/img/headers/about.JPG",
  "hobby-projects": "/assets/img/headers/about.JPG",
  personal: "/assets/img/headers/about.JPG",
};
