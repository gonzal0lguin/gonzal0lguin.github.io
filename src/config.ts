// Place any global data in this file.
// You can import this data from anywhere in your site by using the `import` keyword.
// Translatable text (site title, description, UI labels) lives in src/i18n/ui.ts.

export const TRANSITION_API = true

// DaisyUI themes used by the light/dark toggle. Any theme listed in tailwind.config.cjs works.
export const DARK_THEME = 'dim'
export const LIGHT_THEME = 'light'
export const THEME_STORAGE_KEY = 'theme'

// Where to find me. `scholar` is an author search until the profile URL is known: replace it
// with the https://scholar.google.com/citations?user=... link.
export const LINKS = {
  email: 'golguinm@gmail.com',
  github: 'https://github.com/gonzal0lguin',
  linkedin: 'https://www.linkedin.com/in/gonzalo-olguin',
  scholar: 'https://scholar.google.com/citations?view_op=search_authors&mauthors=Gonzalo+Olguin',
}
