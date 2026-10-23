// Where a photo written in a post actually lives, using the site's own rule
// (src/lib/media.mjs) so the editor shows exactly what the page will show.
import { DEFAULT_MEDIA_BASE_URL, resolveMediaUrl } from "../lib/media.mjs";

let base: string = DEFAULT_MEDIA_BASE_URL;

/** Called once the editor knows whether PUBLIC_MEDIA_BASE_URL is set. */
export const setMediaBase = (value: string | null) => void (base = value || DEFAULT_MEDIA_BASE_URL);

export const mediaBase = () => base;

export const photoUrl = (path: string) => resolveMediaUrl(path, base);

/** True when the photo library is somewhere else, so an upload here is only half the job. */
export const isRemoteMedia = () => /^[a-z][a-z0-9+.-]*:|^\/\//i.test(base);
