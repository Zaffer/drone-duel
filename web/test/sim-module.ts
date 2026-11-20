// The simulator's module for the tests that fly it (external/web/sim/sim.wasm, built by
// scripts/setup-web.sh): compiled once, or null where it has not been built (those tests are
// skipped, as the tests that start real firmware are without it).

const PATH = new URL("../../external/web/sim/sim.wasm", import.meta.url).pathname;
let compiled: Promise<WebAssembly.Module | null> | undefined;

export function simModule(): Promise<WebAssembly.Module | null> {
  compiled ??= (async () => {
    const file = Bun.file(PATH);
    if (!(await file.exists())) return null;
    return WebAssembly.compile(await file.arrayBuffer());
  })();
  return compiled;
}
