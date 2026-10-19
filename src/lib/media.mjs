// Where the photos used in posts actually live.
//
// Posts reference a photo by a *relative* path ("la-cruz/summit.jpg"). This module turns
// that into a URL, which is the one place that knows whether the files sit in the repo or
// on a CDN. Moving the photo library out of git is therefore a one-line change:
//
//   .env (local) / repository variable (CI):
//     PUBLIC_MEDIA_BASE_URL=https://photos.example.com
//
// Left untouched: absolute paths ("/assets/img/headers/about.JPG"), full URLs, data: URIs,
// and paths explicitly relative to the Markdown file ("./photo.jpg"), which Astro resolves
// and optimizes itself. So existing posts keep working either way.

export const DEFAULT_MEDIA_BASE_URL = "/assets/img/posts";

export function resolveMediaUrl(src, baseUrl = DEFAULT_MEDIA_BASE_URL) {
    if (typeof src !== "string" || src === "") return src;
    if (/^[a-z][a-z0-9+.-]*:/i.test(src) || /^(\/|\/\/|#|\.{1,2}\/)/.test(src)) return src;
    return `${baseUrl.replace(/\/+$/, "")}/${src}`;
}
