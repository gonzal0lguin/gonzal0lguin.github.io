// The npm scripts the editor can run, streamed back line by line so the browser can show
// the output while it happens. Only these, and only with arguments it builds itself: either
// content files (checked to be inside the project) or summit ids (a bare name, never a path).
import { spawn } from "node:child_process";
import { bad, safePath } from "./util.mjs";

const ID = /^[a-z0-9][a-z0-9-]*$/;

const TASKS = {
  // EXIF locations of the post photos → src/data/photo-locations.json
  photos: { args: "none", build: () => ["run", "photos"] },
  // Translate Spanish content into English with Claude (needs ANTHROPIC_API_KEY)
  translate: { args: "files", build: (files) => ["run", "translate", "--", ...files] },
  // Mark an English file as up to date after fixing it by hand
  stamp: { args: "files", build: (files) => ["run", "translate", "--", "--stamp", ...files] },
  // The view from the named summits (or every one that has none yet): panorama, terrain mesh
  // and viewshed overlay. Minutes of elevation-model and Overpass work per summit.
  panoramas: { args: "ids", build: (ids) => ["run", "panoramas", "--", ...ids] },
  // Every summit's outline for the to-scale chart → src/data/profiles.json
  profiles: { args: "none", build: () => ["run", "profiles"] },
  // A full production build: the surest check that the content still compiles
  build: { args: "none", build: () => ["run", "build"] },
};

export function runTask(root, { task, files = [] }, res) {
  const spec = TASKS[task];
  if (!spec) throw bad(`Unknown task: ${task}`);
  const args = taskArgs(root, spec.args, files);

  res.statusCode = 200;
  res.setHeader("content-type", "text/plain; charset=utf-8");
  res.setHeader("cache-control", "no-store");
  // Nginx-style buffering off, so the client sees the output as it comes.
  res.setHeader("x-accel-buffering", "no");

  const command = spec.build(args);
  res.write(`$ npm ${command.join(" ")}\n`);
  const child = spawn("npm", command, { cwd: root, env: process.env });
  child.stdout.on("data", (chunk) => res.write(chunk));
  child.stderr.on("data", (chunk) => res.write(chunk));
  child.on("error", (error) => res.end(`\n[editor] could not start npm: ${error.message}\n`));
  child.on("close", (code) => res.end(`\n[editor] exit code ${code}\n`));
  res.on("close", () => child.killed || child.kill());
}

function taskArgs(root, kind, given) {
  if (kind === "none") return [];
  if (kind === "files") {
    return given.map((file) => {
      safePath(root, file, { extensions: [".md", ".mdx"] });
      return file;
    });
  }
  return given.map((id) => {
    // `--all` recomputes every panorama, which is the one flag worth offering.
    if (id !== "--all" && !ID.test(id)) throw bad(`Not a summit: ${id}`);
    return id;
  });
}
