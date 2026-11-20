// The page's wind (src/sim/weather.ts) is drone_duels.physics.core.weather's: the same shear,
// the same turbulence by MIL-F-8785C's numbers, the same gusts (tests/test_physics_wing.py holds
// Python's to them; the random numbers differ, so the statistics are what is compared).

import { describe, expect, test } from "bun:test";
import { gustPeak, seeded, Weather } from "../src/sim/weather.ts";

const FOOT = 0.3048;

describe("the wind", () => {
  test("the mean wind is the log profile: none at the ground, the report's at 20 ft", () => {
    const w = new Weather({ speed_ms: 8, from_deg: 270, turbulence: 0 }, 0.26, seeded(1));
    expect(w.shear(0)).toBe(0);
    expect(w.shear(20 * FOOT)).toBeCloseTo(1, 12);
    expect(w.shear(100)).toBeCloseTo(Math.log(100 / (0.15 * FOOT)) / Math.log(20 / 0.15), 12);
    w.step(0.001, 20 * FOOT, 10);
    // From the west: it blows east.
    expect(w.wind[0]).toBeCloseTo(0, 9);
    expect(w.wind[1]).toBeCloseTo(8, 9);
    expect(w.wind[2]).toBe(0);
  });

  test("still air is still: no turbulence without a wind", () => {
    const w = new Weather({}, 0.26, seeded(2));
    for (let i = 0; i < 1000; i++) w.step(0.001, 30, 12);
    expect(w.wind).toEqual([0, 0, 0]);
  });

  test("the turbulence at a point has 8785C's variances, low altitude", () => {
    const speed = 10;
    const height = 30;
    const w = new Weather({ speed_ms: speed, from_deg: 0 }, 0.26, seeded(3));
    const sum = [0, 0, 0];
    const n = 400_000;
    for (let i = 0; i < n; i++) {
      w.step(0.01, height, 15);
      for (let k = 0; k < 3; k++) sum[k]! += w.eddies[k]! ** 2;
    }
    const h = height / FOOT;
    const sigmaW = 0.1 * speed;
    const sigmaU = sigmaW / (0.177 + 0.000823 * h) ** 0.4;
    // From the north: u is along north-south, v east-west, w down.
    expect(Math.sqrt(sum[0]! / n) / sigmaU).toBeCloseTo(1, 1);
    expect(Math.sqrt(sum[1]! / n) / sigmaU).toBeCloseTo(1, 1);
    expect(Math.sqrt(sum[2]! / n) / sigmaW).toBeCloseTo(1, 1);
  });

  test("a gust asked for starts at once, along the wind, and is over in five seconds", () => {
    const w = new Weather({ speed_ms: 5, from_deg: 180, turbulence: 0 }, 0.26, seeded(4));
    w.step(0.01, 20 * FOOT, 10);
    expect(w.gust).toEqual([0, 0, 0]);
    w.update({ speed_ms: 5, from_deg: 180, turbulence: 0 }, true);
    let peak = 0;
    for (let i = 0; i < 500; i++) {
      w.step(0.01, 20 * FOOT, 10);
      // From the south: north, within 30° of it.
      expect(w.gust[0]).toBeGreaterThanOrEqual(0);
      peak = Math.max(peak, Math.hypot(w.gust[0], w.gust[1]));
    }
    // At least half the mean wind, as a gust asked for is.
    expect(peak).toBeCloseTo(2.5, 1);
    expect(w.gust).toEqual([0, 0, 0]);
    expect(gustPeak(10, 1.5)).toBe(5);
    expect(gustPeak(10, 1)).toBe(0);
  });
});
