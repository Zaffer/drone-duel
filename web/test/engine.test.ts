import { describe, expect, test } from "bun:test";
import { Loop } from "../src/engine/loop.ts";
import { approach, Spring } from "../src/engine/spring.ts";

/** The same spring, stepped by small Euler steps: the truth to compare with. */
function fine(hz: number, zeta: number, x0: number, v0: number, t: number): [number, number] {
  const w = 2 * Math.PI * hz;
  let x = x0;
  let v = v0;
  const h = 1e-6;
  for (let s = 0; s < t; s += h) {
    v += (-w * w * x - 2 * zeta * w * v) * h;
    x += v * h;
  }
  return [x, v];
}

describe("the spring", () => {
  for (const zeta of [0.3, 1, 2.5]) {
    test(`ζ ${zeta}: one step of 0.15 s lands where small steps do`, () => {
      const s = new Spring(3, zeta);
      s.x = 0.2;
      s.v = -1.5;
      s.step(0.15);
      const [x, v] = fine(3, zeta, 0.2, -1.5, 0.15);
      expect(s.x).toBeCloseTo(x, 4);
      expect(s.v).toBeCloseTo(v, 3);
    });
  }

  test("a stiff spring stepped at 5 fps settles, never runs away", () => {
    const s = new Spring(14, 0.5);
    s.v = 3;
    let most = 0;
    for (let i = 0; i < 50; i++) {
      s.step(0.2);
      most = Math.max(most, Math.abs(s.x));
    }
    expect(most).toBeLessThan(0.05);
    expect(Math.abs(s.x)).toBeLessThan(1e-9);
  });

  test("it settles on its target", () => {
    const s = new Spring(2, 0.8);
    s.target = 1;
    for (let i = 0; i < 100; i++) s.step(0.05);
    expect(s.x).toBeCloseTo(1, 6);
  });

  test("approach: two half steps go as far as one whole", () => {
    const a = approach(0.1, 0.3);
    const b = 1 - (1 - approach(0.05, 0.3)) ** 2;
    expect(a).toBeCloseTo(b, 12);
  });
});

describe("the frame loop", () => {
  test("a fixed-rate system steps by real time, whatever the frame rate", () => {
    for (const frameMs of [7, 16.7, 50, 100]) {
      const loop = new Loop();
      let steps = 0;
      loop.fixed("x", 240, () => steps++);
      for (let t = 0; t <= 2000; t += frameMs) loop.frame(t);
      const span = Math.floor(2000 / frameMs) * frameMs;
      expect(Math.abs(steps - (span / 1000) * 240)).toBeLessThanOrEqual(1);
    }
  });

  test("a stall is dropped, not caught up: the next frame is no dearer", () => {
    const loop = new Loop();
    const perFrame: number[] = [];
    let steps = 0;
    loop.fixed("x", 1000, () => steps++);
    let t = 0;
    for (let i = 0; i < 10; i++) {
      t += i === 5 ? 3000 : 16;
      steps = 0;
      loop.frame(t);
      perFrame.push(steps);
    }
    expect(Math.max(...perFrame)).toBeLessThanOrEqual(101);
  });

  test("phases run in order, and each system is timed by name", () => {
    const loop = new Loop();
    const order: string[] = [];
    loop.each("c", "place", () => order.push("place"));
    loop.each("a", "input", () => order.push("input"));
    loop.each("b", "sim", () => order.push("sim"));
    loop.meter("pick", 1.5);
    loop.frame(0);
    expect(order).toEqual(["input", "sim", "place"]);
    expect(loop.stats().map((s) => s.name)).toEqual(
      expect.arrayContaining(["a", "b", "c", "pick"]),
    );
    expect(loop.stats().find((s) => s.name === "pick")?.p50).toBeCloseTo(1.5, 6);
  });

  test("a system leaves with its signal", () => {
    const loop = new Loop();
    const stop = new AbortController();
    let n = 0;
    loop.each("x", "sim", () => n++, stop.signal);
    loop.frame(0);
    stop.abort();
    loop.frame(16);
    expect(n).toBe(1);
  });
});

describe("the frame loop's times", () => {
  test("a system that stopped drops out of the stats after a while", () => {
    const loop = new Loop();
    const stop = new AbortController();
    loop.each("gone", "sim", () => {}, stop.signal);
    loop.each("stays", "sim", () => {});
    loop.frame(0);
    stop.abort();
    for (let t = 16; t < 3000; t += 16) loop.frame(t);
    const names = loop.stats().map((s) => s.name);
    expect(names).toContain("stays");
    expect(names).not.toContain("gone");
  });
});
