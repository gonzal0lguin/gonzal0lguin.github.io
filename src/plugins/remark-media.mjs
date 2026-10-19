// Photo layouts for Markdown (and MDX) content, written as plain Markdown.
//
//   A single photo — a normal Markdown image. The optional quoted title is the caption:
//     ![Alt text](la-cruz/ridge.jpg "The last 200 m of the ridge")
//
//   A grid — `cols` is the number of columns (1-4, default 3) and `ratio` the shape of
//   each cell (default 4/3; `ratio=auto` keeps each photo's own proportions):
//     :::gallery{cols=2 ratio=1/1}
//     ![Alt](a.jpg)
//     ![Alt](b.jpg "A caption")
//     :::
//
//   A carousel — one photo at a time, swiped or scrolled sideways:
//     :::carousel{ratio=3/2}
//     ![Alt](a.jpg)
//     ![Alt](b.jpg)
//     :::
//
// Both containers also take `caption="..."` for a line under the whole group.
//
// Where a photo was taken, for posts with a GPS route (see src/lib/photos.ts). Taken from the
// photo's EXIF data (`npm run photos`), or written after a `#` in the path, which wins:
//     ![Cumbre](la-cruz/summit.jpg#km=5.8)               5.8 km along the route
//     ![Cumbre](la-cruz/summit.jpg#time=12:41)           on the route at that local time
//     ![Cumbre](la-cruz/summit.jpg#time=2026-06-13T12:41)
//     ![Cumbre](la-cruz/summit.jpg#-33.7571,-70.1047)    coordinates
//     ![Cumbre](la-cruz/summit.jpg#off)                  never on the map
// Every photo of the file is listed in the frontmatter as `photos` for the layout to place.
//
// This plugin only produces markup and resolves relative paths through src/lib/media.mjs.
// The behaviour (sliding, lightbox) lives in src/scripts/media.ts, the styling in
// src/styles/media.css.
import { visit } from "unist-util-visit";
import { DEFAULT_MEDIA_BASE_URL, resolveMediaUrl } from "../lib/media.mjs";

const MAX_COLS = 4;
const DEFAULT_COLS = 3;
const DEFAULT_RATIO = { gallery: "4/3", carousel: "3/2" };

// The few strings this markup needs. Everything else lives in src/i18n/ui.ts, but a remark
// plugin has no page context, so it reads the language from the file's own path: English
// content lives under src/content/<collection>/en/.
const LABELS = {
    es: { group: "Galería de fotos", prev: "Foto anterior", next: "Foto siguiente", goto: "Ir a la foto" },
    en: { group: "Photo gallery", prev: "Previous photo", next: "Next photo", goto: "Go to photo" },
};

export default function remarkMedia(options = {}) {
    const baseUrl = options.baseUrl || DEFAULT_MEDIA_BASE_URL;

    return (tree, file) => {
        const t = LABELS[/(^|[\\/])en[\\/]/.test(file.path ?? "") ? "en" : "es"];

        // Collect the photos (and strip location fragments) before the layouts consume captions.
        const photos = new Map();
        visit(tree, "image", (image) => {
            const [path, geo] = image.url.split("#");
            image.url = path;
            const known = photos.get(path);
            if (!known) photos.set(path, { path, src: resolveMediaUrl(path, baseUrl), alt: image.alt ?? "", caption: image.title ?? "", geo: geo ?? null });
            else if (geo && !known.geo) known.geo = geo;
        });
        const astro = (file.data.astro ??= {});
        astro.frontmatter = { ...astro.frontmatter, photos: [...photos.values()] };

        // Explicit containers first, so their images are not also treated as loose ones.
        visit(tree, "containerDirective", (node) => {
            if (node.name !== "gallery" && node.name !== "carousel") return;
            const images = [];
            visit(node, "image", (image) => void images.push(image));
            if (images.length === 0) {
                file.message(`Empty :::${node.name} block`, node);
                return;
            }
            Object.assign(node, group(node.name, node.attributes ?? {}, images, t));
        });

        // A paragraph holding nothing but images: one becomes a figure, several a grid.
        // `data.hName` marks the nodes this plugin already built, so they are left alone.
        visit(tree, "paragraph", (node) => {
            if (node.data?.hName || node.children.length === 0) return;
            if (!node.children.every((child) => child.type === "image")) return;
            Object.assign(node, node.children.length === 1
                ? figure(node.children[0], "photo")
                : group("gallery", { cols: String(Math.min(node.children.length, MAX_COLS)) }, [...node.children], t));
        });

        // Finally, point every image at the right URL. `url` itself has to change, not just
        // the rendered attribute, so that Astro's own image handling sees the final path.
        visit(tree, "image", (image) => {
            image.url = resolveMediaUrl(image.url, baseUrl);
            image.data = { ...image.data, hProperties: { ...image.data?.hProperties, loading: "lazy", decoding: "async" } };
        });
    };
}

/** A grid or a carousel of `images`, built from a container directive's attributes. */
function group(name, attributes, images, t) {
    const ratio = attributes.ratio || DEFAULT_RATIO[name];
    const style = ratio === "auto" ? undefined : `--photo-ratio: ${ratio};`;
    const figures = images.map((image) => figure(image, "photo-figure"));
    const caption = attributes.caption
        ? [element("p", { class: "photo-group__caption" }, [{ type: "text", value: attributes.caption }])]
        : [];

    const shared = { "data-lightbox": "", role: "group", "aria-label": t.group };
    if (name === "carousel") {
        return element("div", { class: "photo-group photo-carousel", "data-carousel": "", ...shared }, [
            element("div", { class: "photo-carousel__track", style, tabindex: "0" }, figures),
            element("div", { class: "photo-carousel__controls" }, [
                arrow("prev", t.prev),
                element("div", { class: "photo-carousel__dots" }, images.map((_, i) =>
                    element("button", { type: "button", class: "photo-carousel__dot", "data-slide": String(i), "aria-label": `${t.goto} ${i + 1}` }, []))),
                arrow("next", t.next),
            ]),
            ...caption,
        ]);
    }

    const cols = clamp(parseInt(attributes.cols, 10) || DEFAULT_COLS, 1, MAX_COLS);
    const grid = element("div", { class: "photo-grid", "data-cols": String(cols), style: `${style ?? ""} --photo-cols: ${cols};`.trim(), ...shared }, figures);
    return caption.length ? element("div", { class: "photo-group" }, [grid, ...caption]) : grid;
}

/** A photo with its optional caption: the Markdown image title becomes the caption text. */
function figure(image, className) {
    const caption = image.title;
    image.title = null;
    return element("figure", { class: className }, [
        image,
        ...(caption ? [element("figcaption", {}, [{ type: "text", value: caption }])] : []),
    ]);
}

function arrow(direction, label) {
    const d = direction === "prev" ? "M14.5 5 8 12l6.5 7" : "M9.5 5 16 12l-6.5 7";
    const node = element("button", { type: "button", class: "photo-carousel__arrow", "data-direction": direction, "aria-label": label }, []);
    // Raw hast: an icon has no Markdown equivalent.
    node.data.hChildren = [svg([{ type: "element", tagName: "path", properties: { d, fill: "none", stroke: "currentColor", strokeWidth: 2, strokeLinecap: "round", strokeLinejoin: "round" }, children: [] }])];
    return node;
}

function svg(children) {
    return { type: "element", tagName: "svg", properties: { viewBox: "0 0 24 24", width: 20, height: 20, "aria-hidden": "true", focusable: "false" }, children };
}

/**
 * An mdast node that renders as `<tagName>`: the node type barely matters, because
 * data.hName / data.hProperties replace whatever the default handler produced.
 */
function element(tagName, properties, children) {
    const hProperties = Object.fromEntries(Object.entries(properties).filter(([, value]) => value !== undefined));
    return { type: "paragraph", data: { hName: tagName, hProperties }, children };
}

function clamp(value, min, max) {
    return Math.min(Math.max(value, min), max);
}
