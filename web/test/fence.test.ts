// The engine's fences (DESIGN.md §3.2): what costs a render, draws, or runs each frame goes
// through one place, so it is metered there and cannot hide elsewhere.

import { describe, expect, test } from "bun:test";
import { Glob } from "bun";

const SRC = new URL("../src/", import.meta.url).pathname;
const files = [...new Glob("**/*.ts").scanSync(SRC)].filter((f) => !f.endsWith(".d.ts"));
const text = new Map(files.map((f) => [f, Bun.file(SRC + f).text()]));

/** The files (outside `allowed`) whose code (comments aside) matches `pattern`. */
async function offenders(pattern: RegExp, allowed: string[]): Promise<string[]> {
  const out: string[] = [];
  for (const [f, t] of text) {
    if (allowed.includes(f)) continue;
    const code = (await t)
      .split("\n")
      .filter((l) => !l.trimStart().startsWith("//") && !l.trimStart().startsWith("*"))
      .join("\n");
    if (pattern.test(code)) out.push(f);
  }
  return out;
}

describe("the engine's fences", () => {
  test("Cesium's lookups and picks (each a render) are only in world.ts, metered", async () => {
    const lookups =
      /\b(sampleHeight|sampleHeightMostDetailed|sampleTerrain|sampleTerrainMostDetailed|pickPosition|drillPick)\b|\b(scene|globe)\.pick\(/;
    expect(await offenders(lookups, ["world/world.ts"])).toEqual([]);
  });

  test("everything drawn goes through world.add, so no lookup takes it for ground", async () => {
    expect(await offenders(/primitives\.(add|remove)\(/, ["world/world.ts"])).toEqual([]);
  });

  test("every frame's work is in the frame loop, named and timed", async () => {
    expect(await offenders(/requestAnimationFrame\(/, ["engine/loop.ts"])).toEqual([]);
  });

  test("springs are the exact one: no hand-cut substeps", async () => {
    expect(await offenders(/Math\.ceil\(dt \/ /, [])).toEqual([]);
  });
});

// The web version's simulator runs in a worker, with no page around it (DESIGN.md §18).
describe("the simulator's fence", () => {
  test("sim/ takes nothing of the page's: its own modules, and the protocol's types", async () => {
    const out: string[] = [];
    for (const [f, t] of text) {
      if (!f.startsWith("sim/")) continue;
      for (const m of (await t).matchAll(/^import (type )?[^;]*? from "([^"]+)";/gms)) {
        const [, typeOnly, from] = m;
        const own = from!.startsWith("./");
        const types = typeOnly !== undefined && from === "../net/protocol.ts";
        if (!own && !types) out.push(`${f}: ${from}`);
      }
      const code = (await t)
        .split("\n")
        .filter((l) => !l.trimStart().startsWith("//") && !l.trimStart().startsWith("*"));
      if (code.some((l) => /\b(document|window)\./.test(l))) out.push(`${f}: the page's globals`);
    }
    expect(out).toEqual([]);
  });
});
