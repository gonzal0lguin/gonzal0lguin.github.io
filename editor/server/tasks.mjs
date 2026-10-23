// The npm scripts the editor can run, streamed back line by line so the browser can show
// the output while it happens. Only these three, and only with arguments it builds itself.
import { spawn } from "node:child_process";
import { bad, safePath } from "./util.mjs";

const TASKS = {
  // EXIF locations of the post photos → src/data/photo-locations.json
  photos: () => ["run", "photos"],
  // Translate Spanish content into English with Claude (needs ANTHROPIC_API_KEY)
  translate: (files) => ["run", "translate", "--", ...files],
  // Mark an English file as up to date after fixing it by hand
  stamp: (files) => ["run", "translate", "--", "--stamp", ...files],
  // A full production build: the surest check that the content still compiles
  build: () => ["run", "build"],
};

export function runTask(root, { task, files = [] }, res) {
  const build = TASKS[task];
  if (!build) throw bad(`Unknown task: ${task}`);
  const paths = files.map((file) => {
    safePath(root, file, { extensions: [".md", ".mdx"] });
    return file;
  });

  res.statusCode = 200;
  res.setHeader("content-type", "text/plain; charset=utf-8");
  res.setHeader("cache-control", "no-store");
  // Nginx-style buffering off, so the client sees the output as it comes.
  res.setHeader("x-accel-buffering", "no");

  const args = build(paths);
  res.write(`$ npm ${args.join(" ")}\n`);
  const child = spawn("npm", args, { cwd: root, env: process.env });
  child.stdout.on("data", (chunk) => res.write(chunk));
  child.stderr.on("data", (chunk) => res.write(chunk));
  child.on("error", (error) => res.end(`\n[editor] could not start npm: ${error.message}\n`));
  child.on("close", (code) => res.end(`\n[editor] exit code ${code}\n`));
  res.on("close", () => child.killed || child.kill());
}
