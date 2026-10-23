// The git side of the editor: what changed, what a change looks like, undo, and commit.
// Every command is run with an argument array (never a shell string) and limited to the repo.
import { execFile } from "node:child_process";
import path from "node:path";
import { bad, exists, relative, safePath, trash } from "./util.mjs";

const run = (root, args, { allowFailure = false } = {}) =>
  new Promise((resolve, reject) => {
    execFile("git", args, { cwd: root, maxBuffer: 32 * 1024 * 1024 }, (error, stdout, stderr) => {
      if (error && !allowFailure) reject(bad(`git ${args[0]}: ${stderr.trim() || error.message}`));
      else resolve({ stdout, stderr, code: error?.code ?? 0 });
    });
  });

const STATUS = {
  M: "modified",
  A: "added",
  D: "deleted",
  R: "renamed",
  C: "copied",
  U: "conflict",
  "?": "untracked",
  "!": "ignored",
};

/** Working-tree changes, newest commits and the current branch. */
export async function status(root) {
  const { stdout } = await run(root, ["status", "--porcelain=v1", "-z", "--untracked-files=normal"]);
  const parts = stdout.split("\0").filter(Boolean);
  const files = [];
  for (let i = 0; i < parts.length; i++) {
    const entry = parts[i];
    const code = entry.slice(0, 2);
    const file = entry.slice(3);
    // A rename comes as "R  new\0old".
    if (code[0] === "R" || code[0] === "C") i++;
    files.push({
      path: file,
      staged: code[0] !== " " && code[0] !== "?" ? STATUS[code[0]] ?? code[0] : null,
      unstaged: code[1] !== " " ? STATUS[code[1]] ?? code[1] : null,
      untracked: code === "??",
    });
  }
  const branch = (await run(root, ["rev-parse", "--abbrev-ref", "HEAD"], { allowFailure: true })).stdout.trim();
  const log = (await run(root, ["log", "-5", "--pretty=format:%h\u001f%ar\u001f%s"], { allowFailure: true })).stdout
    .split("\n")
    .filter(Boolean)
    .map((line) => {
      const [hash, when, subject] = line.split("\u001f");
      return { hash, when, subject };
    });
  return { branch, files, log };
}

/** Unified diff of one file, including a brand-new file (shown as all added). */
export async function diff(root, rel) {
  safePath(root, rel);
  const tracked = (await run(root, ["ls-files", "--error-unmatch", "--", rel], { allowFailure: true })).code === 0;
  if (!tracked) {
    const { stdout } = await run(root, ["diff", "--no-index", "--", "/dev/null", rel], { allowFailure: true });
    return { path: rel, untracked: true, diff: stdout };
  }
  const { stdout } = await run(root, ["diff", "HEAD", "--", rel]);
  return { path: rel, untracked: false, diff: stdout };
}

/** Throw away a file's changes: back to its committed state, or to the trash if it is new. */
export async function revert(root, rel) {
  const full = safePath(root, rel);
  const tracked = (await run(root, ["ls-files", "--error-unmatch", "--", rel], { allowFailure: true })).code === 0;
  if (!tracked) {
    if (!(await exists(full))) throw bad(`${rel} not found`);
    return { path: rel, trashed: await trash(root, full) };
  }
  await run(root, ["checkout", "HEAD", "--", rel]);
  return { path: rel, restored: true };
}

export async function commit(root, { message, paths }) {
  const list = (Array.isArray(paths) ? paths : []).filter(Boolean);
  if (!message?.trim()) throw bad("A commit message is required");
  if (list.length === 0) throw bad("Select at least one file to commit");
  for (const rel of list) safePath(root, rel);
  await run(root, ["add", "--", ...list]);
  const result = await run(root, ["commit", "-m", message.trim()], { allowFailure: true });
  if (result.code !== 0) throw bad(result.stderr.trim() || result.stdout.trim() || "git commit failed");
  return { output: `${result.stdout}${result.stderr}`.trim() };
}

