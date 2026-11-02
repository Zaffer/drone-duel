import { describe, expect, test } from "bun:test";
import { fmtTime, gateGeom, Progress, passes } from "../src/modes/race/gates.ts";
import type { GateFrame } from "../src/net/protocol.ts";
import { moveBy } from "../src/world/geo.ts";

const LAT = 45.3196;
const LON = -75.7586;
// A gate 3 × 2.5 m inside facing west (fly through it westwards), its centre 3 m above sea level 0.
const frame: GateFrame = { inside_m: [3, 2.5], outside_m: [4, 3.5], depth_m: 0.26 };
const gate = gateGeom({ at: [LAT, LON], alt_m: 3, ypr_deg: [270, 0, 0] }, frame, 3);
const at = (east: number, north: number, alt: number) => {
  const [lat, lon] = moveBy(LAT, LON, north, east);
  return { lat, lon, alt };
};

describe("gates", () => {
  test("through the middle, forwards, passes", () => {
    expect(passes(gate, at(2, 0, 3), at(-2, 0, 3))).toBe(true);
  });
  test("backwards does not", () => {
    expect(passes(gate, at(-2, 0, 3), at(2, 0, 3))).toBe(false);
  });
  test("beside it, above it or below it does not", () => {
    expect(passes(gate, at(2, 2, 3), at(-2, 2, 3))).toBe(false);
    expect(passes(gate, at(2, 0, 5), at(-2, 0, 5))).toBe(false);
    expect(passes(gate, at(2, 0, 1.3), at(-2, 0, 1.3))).toBe(false);
  });
  test("just inside the edge, at an angle, passes", () => {
    expect(passes(gate, at(1, 1.3, 3.9), at(-1, 1.6, 4.1))).toBe(true);
  });
  test("a pitched gate (a dive gate, facing down)", () => {
    const dive = gateGeom({ at: [LAT, LON], alt_m: 10, ypr_deg: [0, -90, 0] }, frame, 10);
    expect(passes(dive, at(0, 0, 12), at(0, 0, 8))).toBe(true);
    expect(passes(dive, at(0, 0, 8), at(0, 0, 12))).toBe(false);
  });
  test("the standard gate: only its 1.5 m inside counts, not its frame", () => {
    const std: GateFrame = { inside_m: [1.5, 1.5], outside_m: [2.7, 2.7], depth_m: 0.26 };
    const g = gateGeom({ at: [LAT, LON], alt_m: 3, ypr_deg: [270, 0, 0] }, std, 3);
    expect(g.outerWidth).toBe(2.7);
    expect(passes(g, at(2, 0.9, 3), at(-2, 0.9, 3))).toBe(true);
    // Through the frame's bar, not its inside.
    expect(passes(g, at(2, 1.2, 3), at(-2, 1.2, 3))).toBe(false);
  });
  test("progress counts gates and laps", () => {
    const p = new Progress(3, 2);
    expect([p.pass(1), p.pass(2), p.pass(3)]).toEqual([false, false, true]);
    expect(p.lap).toBe(2);
    expect(p.gate).toBe(0);
    p.pass(4);
    p.pass(5);
    p.pass(7);
    expect(p.done).toBe(true);
    expect(p.laps).toEqual([3, 4]);
    expect(p.upcoming()).toEqual([]);
  });
  test("the next three gates, wrapping into the next lap but not past the finish", () => {
    const p = new Progress(4, 2);
    expect(p.upcoming()).toEqual([0, 1, 2]);
    p.pass(1);
    p.pass(2);
    p.pass(3);
    expect(p.upcoming()).toEqual([3, 0, 1]);
    for (let i = 0; i < 3; i++) p.pass(4 + i);
    expect(p.upcoming()).toEqual([2, 3]);
  });
  test("times", () => {
    expect(fmtTime(83.456)).toBe("1:23.46");
    expect(fmtTime(5)).toBe("0:05.00");
  });
});
