// A polite Overpass client: a real User-Agent (the public servers reject anonymous ones),
// GET requests, exponential backoff when every worker is busy, and a cache on disk so a
// region is only ever asked for once. Delete the cached file to refresh it.

import { existsSync, mkdirSync, readFileSync, writeFileSync } from "node:fs";

const SERVERS = ["https://overpass-api.de/api/interpreter", "https://overpass.private.coffee/api/interpreter"];
const AGENT = "gonzalo-olguin-web/1.0 (+https://github.com/gonzal0lguin; build script)";

/** Runs an Overpass QL query (without the [out:json] header) and caches the parsed result at `file` (a URL). */
export async function overpass(query, file) {
  if (existsSync(file)) return JSON.parse(readFileSync(file, "utf8"));
  const body = `[out:json][timeout:300];${query}`;
  let wait = 5;
  for (let attempt = 0; attempt < 10; attempt++) {
    const server = SERVERS[attempt % SERVERS.length];
    try {
      const response = await fetch(`${server}?data=${encodeURIComponent(body)}`, { headers: { "User-Agent": AGENT, Accept: "application/json" }, signal: AbortSignal.timeout(330_000) });
      const text = await response.text();
      if (response.ok && text.startsWith("{")) {
        const data = JSON.parse(text);
        mkdirSync(new URL(".", file), { recursive: true });
        writeFileSync(file, JSON.stringify({ query, fetched: new Date().toISOString().slice(0, 10), elements: data.elements }));
        return { query, elements: data.elements };
      }
      process.stderr.write(`overpass ${server}: HTTP ${response.status}, retrying in ${wait}s\n`);
    } catch (error) {
      process.stderr.write(`overpass ${server}: ${error.message}, retrying in ${wait}s\n`);
    }
    await new Promise((resolve) => setTimeout(resolve, wait * 1000));
    wait = Math.min(wait * 2, 120);
  }
  throw new Error("Overpass: no server answered");
}
