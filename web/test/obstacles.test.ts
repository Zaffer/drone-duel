import { describe, expect, test } from "bun:test";
import { lookDistances, type OverKind, plan } from "../src/shoot/obstacles.ts";

const walk = new Set<OverKind>(["vault"]);
const lean = new Set<OverKind>(["vault", "push"]);
const jump = new Set<OverKind>(["vault", "climb"]);

/** Heights along the way: flat at 0, with `h` from `from` m to `to` m. */
function ground(far: number, from: number, to: number, h: number, beyond = 0) {
  const ds = lookDistances(far);
  return { ds, hs: ds.map((d) => (d < from ? 0 : d <= to ? h : beyond)) };
}

describe("getting over things", () => {
  test("a kerb is not in the way: it is stepped onto", () => {
    const { ds, hs } = ground(2, 0.5, 9, 0.3);
    expect(plan(0, ds, hs, jump)).toBeNull();
  });

  test("a waist-high wall: walking on into it vaults onto it", () => {
    const { ds, hs } = ground(2, 0.5, 9, 0.9);
    const p = plan(0, ds, hs, walk);
    expect(p?.kind).toBe("vault");
    expect(p?.feet).toBeCloseTo(0.9, 6);
    expect(p?.ground).toBeCloseTo(0.9, 6);
  });

  test("a thin fence: over it, and down the far side", () => {
    const { ds, hs } = ground(2, 0.5, 0.7, 1);
    const p = plan(0, ds, hs, walk);
    expect(p?.kind).toBe("vault");
    expect(p?.feet).toBeCloseTo(1, 6);
    expect(p?.ground).toBeCloseTo(0, 6);
  });

  test("a head-high wall wants a jump to climb; walking into it does not", () => {
    const { ds, hs } = ground(2, 0.5, 9, 2);
    expect(plan(0, ds, hs, walk)).toBeNull();
    const p = plan(0, ds, hs, jump);
    expect(p?.kind).toBe("climb");
    expect(p?.rise).toBeCloseTo(2, 6);
  });

  test("too high to climb, and thick: a wall", () => {
    const { ds, hs } = ground(4.5, 0.5, 9, 3);
    expect(plan(0, ds, hs, jump)).toBeNull();
    expect(plan(0, ds, hs, lean)).toBeNull();
  });

  test("a tall bush with ground beyond: leaning in pushes through to the far side", () => {
    const { ds, hs } = ground(4.5, 0.5, 2.5, 3.5);
    const p = plan(0, ds, hs, lean);
    expect(p?.kind).toBe("push");
    expect(p?.dist).toBeGreaterThan(2.5);
    expect(p?.dist).toBeLessThanOrEqual(4.5);
    expect(p?.feet).toBeCloseTo(0, 6);
  });

  test("a leaning wall: the top is found beyond the first look over a step", () => {
    const ds = lookDistances(2);
    const hs = ds.map((d) => (d < 0.5 ? 0 : d < 0.8 ? 0.8 : 1.9));
    const p = plan(0, ds, hs, jump);
    expect(p?.kind).toBe("climb");
    expect(p?.feet).toBeCloseTo(1.9, 6);
  });

  test("something out of reach ahead is not reached for", () => {
    const { ds, hs } = ground(2.5, 1.5, 9, 0.9);
    expect(plan(0, ds, hs, jump)).toBeNull();
  });
});
