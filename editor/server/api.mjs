// The editor's JSON API. Mounted on the dev server by editor/vite-plugin.mjs and reachable
// from this machine only; it never exists in a production build.
import { createEntry, deleteEntry, listEntries, readEntry, renameEntry, saveEntry } from "./content.mjs";
import * as git from "./git.mjs";
import { health } from "./health.mjs";
import { deleteMedia, listGpx, listMedia, readGpx, replaceFile, saveGpx, siteAssets, upload } from "./media.mjs";
import { runTask } from "./tasks.mjs";
import { HttpError, LOCAL_DIR, bad, readBody, readJson, sendJson } from "./util.mjs";

export function createEditorApi({ root, mediaBaseUrl }) {
  const routes = {
    "GET /ping": () => ({ ok: true, root, mediaBaseUrl: mediaBaseUrl ?? null }),

    "GET /state": async () => ({
      root,
      mediaBaseUrl: mediaBaseUrl ?? null,
      localDir: LOCAL_DIR,
      collections: await listEntries(root),
      gpx: await listGpx(root),
      git: await git.status(root).catch((error) => ({ error: error.message, files: [], log: [] })),
    }),

    "GET /entry": ({ query }) => readEntry(root, query.get("path")),
    "POST /entry/save": async ({ json }) => saveEntry(root, await json()),
    "POST /entry/create": async ({ json }) => createEntry(root, await json()),
    "POST /entry/delete": async ({ json }) => deleteEntry(root, await json()),
    "POST /entry/rename": async ({ json }) => renameEntry(root, await json()),

    "GET /media": () => listMedia(root, mediaBaseUrl),
    "POST /media/upload": async ({ query, body }) =>
      upload(root, { kind: query.get("kind") ?? "photo", folder: query.get("folder") ?? "", name: query.get("name") }, await body()),
    "POST /media/replace": async ({ query, body }) => replaceFile(root, { path: query.get("path") }, await body()),
    "POST /media/delete": async ({ json }) => deleteMedia(root, await json()),

    // The site's own files (profile photo, CV PDF, favicon), which no collection owns.
    "GET /site/assets": () => siteAssets(root),

    "GET /gpx/list": () => listGpx(root),
    "GET /gpx": ({ query }) => readGpx(root, query.get("path")),
    "POST /gpx/save": async ({ json }) => saveGpx(root, await json()),

    "GET /git/status": () => git.status(root),
    "GET /git/diff": ({ query }) => git.diff(root, query.get("path")),
    "POST /git/revert": async ({ json }) => git.revert(root, (await json()).path),
    "POST /git/commit": async ({ json }) => git.commit(root, await json()),

    "GET /health": () => health(root, mediaBaseUrl),
  };

  return async function editorApi(req, res, next) {
    if (!isLocal(req)) {
      // The dev server may be bound to 0.0.0.0 (`astro dev --host`); the editor stays private.
      sendJson(res, 403, { error: "The editor API only answers requests from this machine" });
      return;
    }
    const url = new URL(req.url ?? "/", "http://localhost");
    const key = `${req.method} ${url.pathname.replace(/\/+$/, "") || "/"}`;
    try {
      // Streamed: writes to the response itself instead of returning JSON.
      if (key === "POST /task") {
        runTask(root, await readJson(req), res);
        return;
      }
      const handler = routes[key];
      if (!handler) {
        next?.();
        if (!next) sendJson(res, 404, { error: `No editor API route for ${key}` });
        return;
      }
      const result = await handler({
        query: url.searchParams,
        json: () => readJson(req),
        body: () => readBody(req),
      });
      sendJson(res, 200, result ?? { ok: true });
    } catch (error) {
      const status = error instanceof HttpError ? error.status : 500;
      if (status >= 500) console.error("[editor]", error);
      sendJson(res, status, { error: error.message || String(error) });
    }
  };
}

function isLocal(req) {
  const address = req.socket?.remoteAddress ?? "";
  return address === "127.0.0.1" || address === "::1" || address === "::ffff:127.0.0.1" || address === "";
}

