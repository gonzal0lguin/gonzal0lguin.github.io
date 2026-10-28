// Screenshots pages of the built site, so a visual change can actually be looked at:
//
//   npm run shot -- /en/ascents/union /en/ascents
//
// It builds nothing. Run `npm run build` first, then this: it starts `astro preview` on a
// free port, shoots every path it was given, and shuts the server down again.
//
// Why Playwright's Firefox and not the one in PATH: the system Firefox here is a snap, and a
// snap needs to create a mount namespace to start. Agent/CI shells usually can't (`unshare`
// is blocked), so it hangs forever with no output. A snap also gets a *private* /tmp, so even
// when it does run, `--screenshot /tmp/x.png` writes somewhere nobody else can see. The
// Playwright build is a plain tarball under ~/.cache/ms-playwright with none of that.
//
// Needs node >= 20 (Playwright's floor). The Ubuntu node is 18, so see CLAUDE.md for the
// side-by-side install; this script says so if it's running on the wrong one.
import { spawn } from "node:child_process";
import { mkdirSync, rmSync } from "node:fs";
import path from "node:path";
import { homedir } from "node:os";
import { createServer } from "node:net";

// Playwright looks for its browsers under XDG_CACHE_HOME, and a snap terminal (alacritty,
// here) points that at its own private cache — so the same install is findable from one
// terminal and missing from the next. Pinning the path makes it the same everywhere. It has
// to be set before `playwright` is imported, because that's when it reads it.
process.env.PLAYWRIGHT_BROWSERS_PATH ??= path.join(homedir(), ".cache", "ms-playwright");

const OUTPUT = "tmp/shots";
const VIEWPORT = { width: 1440, height: 900 };

// Content processes use an unprivileged user namespace, which is the same thing the sandbox
// here denies, so Firefox would die on first navigation. The page is ours and the browser is
// thrown away afterwards, so dropping that layer costs nothing.
const ENVIRONMENT = { ...process.env, MOZ_DISABLE_CONTENT_SANDBOX: "1" };

function parse(argv) {
  const options = { paths: [], full: false, out: OUTPUT, ...VIEWPORT };
  for (let i = 0; i < argv.length; i++) {
    const argument = argv[i];
    if (argument === "--full") options.full = true;
    else if (argument === "--width") options.width = Number(argv[++i]);
    else if (argument === "--height") options.height = Number(argv[++i]);
    else if (argument === "--out") options.out = argv[++i];
    else if (argument.startsWith("--")) throw new Error(`unknown flag ${argument}`);
    else options.paths.push(argument);
  }
  if (!options.paths.length) options.paths.push("/");
  return options;
}

/** An unused port, asked of the OS rather than guessed, so parallel runs don't collide. */
function freePort() {
  return new Promise((resolve, reject) => {
    const probe = createServer();
    probe.on("error", reject);
    probe.listen(0, "127.0.0.1", () => {
      const { port } = probe.address();
      probe.close(() => resolve(port));
    });
  });
}

/** Starts `astro preview` and resolves once it answers, so the first shot isn't of nothing. */
async function preview(port) {
  const server = spawn("npx", ["astro", "preview", "--port", String(port)], { stdio: "ignore" });
  server.on("error", (error) => {
    throw error;
  });
  const deadline = Date.now() + 60_000;
  while (Date.now() < deadline) {
    try {
      await fetch(`http://127.0.0.1:${port}/`, { signal: AbortSignal.timeout(2_000) });
      return server;
    } catch {
      await new Promise((resolve) => setTimeout(resolve, 300));
    }
  }
  server.kill();
  throw new Error("astro preview never came up — is dist/ built? run `npm run build`");
}

/** `/en/ascents/union` -> `en-ascents-union.png`, and `/` -> `index.png`. */
function filename(urlPath) {
  const slug = urlPath.replace(/^\/+|\/+$/g, "").replace(/[^a-zA-Z0-9]+/g, "-");
  return `${slug || "index"}.png`;
}

const options = parse(process.argv.slice(2));

let firefox;
try {
  ({ firefox } = await import("playwright"));
} catch (error) {
  const version = Number(process.versions.node.split(".")[0]);
  if (version < 20) throw new Error(`Playwright needs node >= 20, this is ${process.versions.node}. Try: PATH=~/.local/share/node24/bin:$PATH npm run shot -- ...`);
  throw error;
}

const port = await freePort();
const server = await preview(port);
rmSync(options.out, { recursive: true, force: true });
mkdirSync(options.out, { recursive: true });

const browser = await firefox.launch({ env: ENVIRONMENT });
const page = await browser.newPage({ viewport: { width: options.width, height: options.height } });

try {
  for (const urlPath of options.paths) {
    const url = `http://127.0.0.1:${port}${urlPath.startsWith("/") ? urlPath : `/${urlPath}`}`;
    const response = await page.goto(url, { waitUntil: "load", timeout: 30_000 });
    // Maps, the 3D panorama and the charts all draw after load, and a shot taken mid-draw is
    // worse than no shot at all because it looks like a rendering bug.
    await page.waitForTimeout(1_500);
    const file = path.join(options.out, filename(urlPath));
    await page.screenshot({ path: file, fullPage: options.full });
    console.log(`${response?.status() ?? "???"}  ${urlPath}  ->  ${file}`);
  }
} finally {
  await browser.close();
  server.kill();
}
