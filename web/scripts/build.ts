// Builds the page into dist/: src/main.ts bundled (Cesium, Rapier and the rest with it), its CSS
// and fonts, and index.html. `--watch` rebuilds on every change under src/.
import { watch } from "node:fs";
import { cp, mkdir, rm } from "node:fs/promises";

const root = new URL("..", import.meta.url).pathname;
const dist = `${root}dist`;

async function build(): Promise<boolean> {
  const started = performance.now();
  const result = await Bun.build({
    entrypoints: [`${root}src/main.ts`],
    outdir: dist,
    target: "browser",
    format: "esm",
    splitting: true,
    minify: { syntax: true, whitespace: true, identifiers: false },
    sourcemap: "linked",
    naming: {
      entry: "[name].[ext]",
      chunk: "chunks/[name]-[hash].[ext]",
      asset: "assets/[name]-[hash].[ext]",
    },
    define: { "process.env.NODE_ENV": '"production"' },
  });
  for (const log of result.logs) console.error(log);
  if (!result.success) return false;
  await cp(`${root}index.html`, `${dist}/index.html`);
  console.log(`built in ${Math.round(performance.now() - started)} ms`);
  return true;
}

await rm(dist, { recursive: true, force: true });
await mkdir(dist, { recursive: true });
const ok = await build();
if (process.argv.includes("--watch")) {
  let timer: ReturnType<typeof setTimeout> | undefined;
  watch(`${root}src`, { recursive: true }, () => {
    clearTimeout(timer);
    timer = setTimeout(() => void build(), 100);
  });
  console.log("watching src/");
} else if (!ok) {
  process.exit(1);
}
