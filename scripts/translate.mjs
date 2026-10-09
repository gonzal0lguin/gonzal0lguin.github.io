#!/usr/bin/env node
// Translate Spanish content into English with Claude.
//
// Spanish sources:  src/content/<collection>/<name>.md(x)
// English output:   src/content/<collection>/en/<name>.md(x)
//
// Each generated English file gets a `translationHash` frontmatter field: a hash of the
// Spanish file it was translated from. A file is (re)translated when its English version
// is missing or its hash no longer matches the Spanish source. English files *without*
// a translationHash are treated as hand-written and never overwritten (unless --force).
//
// Usage:
//   npm run translate                      translate everything that is missing or stale
//   npm run translate -- --dry-run         only list what would be translated
//   npm run translate -- --force           retranslate all files (including hand-written ones)
//   npm run translate -- --stamp           mark existing English files as up to date
//                                          (no API calls; use after fixing a translation by hand)
//   npm run translate -- src/content/ascents/union.md   limit to specific Spanish files
//
// Needs Anthropic API credentials: ANTHROPIC_API_KEY, or a profile from `ant auth login`.

import { createHash } from "node:crypto";
import { existsSync, readdirSync, readFileSync, writeFileSync, mkdirSync } from "node:fs";
import path from "node:path";
import Anthropic from "@anthropic-ai/sdk";

const CONTENT_DIR = "src/content";
const EN_DIR = "en";
const MODEL = "claude-opus-5-0";
// Frontmatter keys whose values are translated; every other line is copied from the Spanish file.
const TRANSLATABLE_KEYS = new Set(["title", "description", "badge", "greeting", "subtitle", "mountain"]);

const SYSTEM_PROMPT = `You translate pages of a personal website (a robotics engineering portfolio and a mountaineering log) from Chilean Spanish into natural, fluent English.

You will receive one Markdown or MDX file. Reply with the complete translated file and nothing else: no preamble, no explanation, and no code fence around the whole file.

Rules:
- Keep the YAML frontmatter block. Translate only the values of these keys: ${[...TRANSLATABLE_KEYS].join(", ")}. Copy every other frontmatter line unchanged.
- Preserve the Markdown structure exactly: headings, lists, tables, links, images, raw HTML and MDX/JSX components. Translate the human-readable text inside them, including string attribute values such as title, subtitle and alt. Never translate component or attribute names, URLs, file paths, CSS or HTML class names.
- Leave code blocks, inline code and LaTeX math ($...$ and $$...$$) unchanged.
- Keep proper nouns as written (people, mountains, places, institutions, course names such as "Cerro La Cruz" or "Universidad de Chile") unless there is a well-established English name.
- Keep the author's personal, informal first-person voice. Keep drafting placeholders and notes-to-self (such as "blabla", "X km" or "(referencia a fotos)") in place, translating any words in them.
- If a passage is already in English, keep it as it is.`;

const args = process.argv.slice(2);
const flags = new Set(args.filter((a) => a.startsWith("--")));
const only = args.filter((a) => !a.startsWith("--")).map((p) => path.resolve(p));
const dryRun = flags.has("--dry-run");
const force = flags.has("--force");
const stamp = flags.has("--stamp");

function sourceFiles() {
  const files = [];
  for (const collection of readdirSync(CONTENT_DIR, { withFileTypes: true })) {
    if (!collection.isDirectory()) continue;
    const dir = path.join(CONTENT_DIR, collection.name);
    for (const file of readdirSync(dir, { withFileTypes: true })) {
      if (file.isFile() && /\.mdx?$/.test(file.name)) files.push(path.join(dir, file.name));
    }
  }
  return files.filter((f) => only.length === 0 || only.includes(path.resolve(f))).sort();
}

const englishPath = (source) => path.join(path.dirname(source), EN_DIR, path.basename(source));
const hashOf = (text) => createHash("sha256").update(text).digest("hex").slice(0, 16);

// --- Frontmatter: split into top-level "key: value" blocks (a key line plus its indented lines).

function splitFrontmatter(text) {
  const match = text.match(/^---\r?\n([\s\S]*?)\r?\n---\r?\n?/);
  if (!match) return { blocks: [], body: text };
  const blocks = [];
  for (const line of match[1].split(/\r?\n/)) {
    const key = line.match(/^([A-Za-z_][\w-]*)\s*:/)?.[1];
    if (key) blocks.push({ key, lines: [line] });
    else if (blocks.length) blocks[blocks.length - 1].lines.push(line);
    else blocks.push({ key: null, lines: [line] });
  }
  return { blocks, body: text.slice(match[0].length) };
}

function joinFrontmatter(blocks, body) {
  return `---\n${blocks.flatMap((b) => b.lines).join("\n")}\n---\n${body}`;
}

function withHash(blocks, hash) {
  const rest = blocks.filter((b) => b.key !== "translationHash");
  return [{ key: "translationHash", lines: [`translationHash: "${hash}"`] }, ...rest];
}

/** Spanish frontmatter, with only the translatable values taken from the translation. */
function mergeFrontmatter(sourceText, translatedText, hash) {
  const source = splitFrontmatter(sourceText);
  const translated = splitFrontmatter(translatedText);
  const translatedByKey = new Map(translated.blocks.map((b) => [b.key, b]));
  const blocks = source.blocks.map((b) =>
    TRANSLATABLE_KEYS.has(b.key) && translatedByKey.has(b.key) ? translatedByKey.get(b.key) : b,
  );
  return joinFrontmatter(withHash(blocks, hash), translated.body);
}

function readHash(text) {
  return splitFrontmatter(text)
    .blocks.find((b) => b.key === "translationHash")
    ?.lines[0].match(/translationHash:\s*["']?([^"'\s]+)/)?.[1];
}

// --- Translation

let client;

async function translate(sourceText, file) {
  const response = await client.beta.messages
    .stream({
      model: MODEL,
      max_tokens: 64000,
      output_config: { effort: "low" },
      // If a request is declined by a safety classifier, retry it on the default fallback model.
      betas: ["server-side-fallback-2026-07-01"],
      fallbacks: "default",
      system: SYSTEM_PROMPT,
      messages: [{ role: "user", content: `File: ${path.basename(file)}\n\n${sourceText}` }],
    })
    .finalMessage();

  if (response.stop_reason === "refusal") throw new Error("the model declined to translate this file");
  if (response.stop_reason === "max_tokens") throw new Error("the translation was cut off (file too long)");

  let text = response.content
    .filter((block) => block.type === "text")
    .map((block) => block.text)
    .join("")
    .trim();
  // Unwrap if the whole file came back inside a code fence anyway.
  text = text.replace(/^```(?:mdx?|markdown)?\r?\n([\s\S]*?)\r?\n```$/, "$1");
  if (sourceText.startsWith("---") && !text.startsWith("---")) {
    throw new Error("the translation came back without its frontmatter");
  }
  return `${text}\n`;
}

// --- Main

const plan = [];
for (const source of sourceFiles()) {
  const target = englishPath(source);
  const sourceText = readFileSync(source, "utf8");
  const hash = hashOf(sourceText);
  const existing = existsSync(target) ? readFileSync(target, "utf8") : null;
  const existingHash = existing && readHash(existing);

  if (stamp) {
    if (existing && existingHash !== hash) plan.push({ source, target, hash, action: "stamp", existing });
    continue;
  }
  if (!existing) plan.push({ source, target, hash, sourceText, action: "new" });
  else if (force) plan.push({ source, target, hash, sourceText, action: "forced" });
  else if (!existingHash) console.log(`skip   ${target} (hand-written: no translationHash)`);
  else if (existingHash !== hash) plan.push({ source, target, hash, sourceText, action: "stale" });
}

if (plan.length === 0) {
  console.log("All English content is up to date.");
  process.exit(0);
}

if (!dryRun && plan.some((item) => item.action !== "stamp")) {
  try {
    client = new Anthropic();
  } catch (error) {
    console.error(`${error.message}\nSet ANTHROPIC_API_KEY or run \`ant auth login\`.`);
    process.exit(1);
  }
}

let failed = 0;
for (const item of plan) {
  if (dryRun) {
    console.log(`would ${item.action === "stamp" ? "stamp" : "translate"} ${item.source} → ${item.target} (${item.action})`);
    continue;
  }
  if (item.action === "stamp") {
    const { blocks, body } = splitFrontmatter(item.existing);
    writeFileSync(item.target, joinFrontmatter(withHash(blocks, item.hash), body));
    console.log(`stamp  ${item.target}`);
    continue;
  }
  process.stdout.write(`translate ${item.source} (${item.action})… `);
  try {
    const translated = await translate(item.sourceText, item.source);
    mkdirSync(path.dirname(item.target), { recursive: true });
    writeFileSync(item.target, mergeFrontmatter(item.sourceText, translated, item.hash));
    console.log("done");
  } catch (error) {
    failed++;
    console.log("failed");
    if (error instanceof Anthropic.AuthenticationError) {
      console.error("Invalid or missing API credentials. Set ANTHROPIC_API_KEY or run `ant auth login`.");
      process.exit(1);
    }
    console.error(`  ${error instanceof Anthropic.APIError ? `API error ${error.status}: ` : ""}${error.message}`);
  }
}

if (failed) {
  console.error(`${failed} file(s) failed to translate.`);
  process.exit(1);
}
