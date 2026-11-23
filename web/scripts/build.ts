// Builds the page into dist/: src/main.ts bundled (Cesium, Rapier and the rest with it), its CSS
// and fonts, and index.html. `--watch` rebuilds on every change under src/.
//
// `--web` builds the web version instead (DESIGN.md §18), into dist-web/: the same page with
// DUELS_WEB set (src/net/target.ts), the simulator's worker beside it (src/sim/worker.ts), and
// everything the game's server would serve, as files: Cesium's assets, the models, and the
// simulator's module (external/web/sim/sim.wasm, built by scripts/setup-web.sh). Its data
// (dist-web/data/) is written by `uv run duels web`, which runs this.
import { watch } from "node:fs";
import { cp, mkdir, rm } from "node:fs/promises";

const root = new URL("..", import.meta.url).pathname;
const web = process.argv.includes("--web");
const dist = `${root}${web ? "dist-web" : "dist"}`;
const MODULE = `${root}../external/web/sim/sim.wasm`;
/** The web version's analytics (src/net/analytics.ts): GoatCounter, cookieless; not in the page
 * the game's server serves. */
const ANALYTICS = `<script data-goatcounter="https://droneduels.goatcounter.com/count" async src="//gc.zgo.at/count.js"></script>`;

async function bundle(entrypoints: string[], entry: string): Promise<boolean> {
  const result = await Bun.build({
    entrypoints,
    outdir: dist,
    target: "browser",
    format: "esm",
    splitting: true,
    minify: { syntax: true, whitespace: true, identifiers: false },
    sourcemap: "linked",
    naming: {
      entry,
      chunk: "chunks/[name]-[hash].[ext]",
      asset: "assets/[name]-[hash].[ext]",
    },
    define: { "process.env.NODE_ENV": '"production"', DUELS_WEB: web ? "true" : "false" },
  });
  for (const log of result.logs) console.error(log);
  return result.success;
}

async function build(): Promise<boolean> {
  const started = performance.now();
  if (!(await bundle([`${root}src/main.ts`], "[name].[ext]"))) return false;
  const html = await Bun.file(`${root}index.html`).text();
  await Bun.write(
    `${dist}/index.html`,
    web ? html.replace("</head>", `  ${ANALYTICS}\n  </head>`) : html,
  );
  if (web) {
    // The worker is a bundle of its own: nothing of the page's (Cesium, the DOM) is in it.
    if (!(await bundle([`${root}src/sim/worker.ts`], "sim-[name].[ext]"))) return false;
    if (!(await Bun.file(MODULE).exists())) {
      console.error(`no simulator at ${MODULE}: run scripts/setup-web.sh`);
      return false;
    }
    await mkdir(`${dist}/sim`, { recursive: true });
    await cp(MODULE, `${dist}/sim/sim.wasm`);
  }
  console.log(`built in ${Math.round(performance.now() - started)} ms`);
  return true;
}

/** What the game's server serves from elsewhere, copied beside the web version's page. */
async function statics(): Promise<void> {
  const cesium = `${root}node_modules/cesium/Build/Cesium`;
  for (const part of ["Assets", "ThirdParty", "Widgets", "Workers"]) {
    await cp(`${cesium}/${part}`, `${dist}/cesium/${part}`, { recursive: true });
  }
  await cp(`${root}public/models`, `${dist}/models`, { recursive: true });
  // GitHub Pages: serve the files as they are (no Jekyll).
  await Bun.write(`${dist}/.nojekyll`, "");
}

await rm(dist, { recursive: true, force: true });
await mkdir(dist, { recursive: true });
const ok = await build();
if (ok && web) await statics();
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
