import { describe, expect, test } from "bun:test";
import {
  dipoleGainDbi,
  fadeStep,
  fitFrame,
  linkGeometry,
  NULL_FLOOR_DBI,
} from "../src/video/link.ts";

describe("the camera feed's frame", () => {
  test("4:3 in a wide window: full height, centred across", () => {
    const f = fitFrame(1920, 1080, 4 / 3);
    expect(f.height).toBe(1080);
    expect(f.width).toBe(1440);
    expect(f.left).toBe(240);
    expect(f.top).toBe(0);
  });

  test("16:9 in a 16:10 window: letterboxed", () => {
    const f = fitFrame(1920, 1200, 16 / 9);
    expect(f.width).toBe(1920);
    expect(f.height).toBe(1080);
    expect(f.top).toBe(60);
  });
});

describe("the video link", () => {
  test("a dipole: 2.15 dBi broadside, its null filled to NULL_FLOOR_DBI along its axis", () => {
    expect(dipoleGainDbi(0)).toBeCloseTo(2.15, 2);
    expect(dipoleGainDbi(1)).toBe(NULL_FLOOR_DBI);
    expect(dipoleGainDbi(0.5)).toBeLessThan(2.15);
  });

  test("level and out to the side: both ends broadside", () => {
    const g = linkGeometry([100, 0, 0], [0, 0, 1]);
    expect(g.range).toBe(100);
    expect(g.txGainDbi).toBeCloseTo(2.15, 2);
    expect(g.rxGainDbi).toBeCloseTo(2.15, 2);
  });

  test("straight overhead: both ends in their nulls", () => {
    const g = linkGeometry([0, 0, 50], [0, 0, 1]);
    expect(g.txGainDbi).toBe(NULL_FLOOR_DBI);
    expect(g.rxGainDbi).toBe(NULL_FLOOR_DBI);
  });

  test("fading decays toward 0 dB with no noise", () => {
    expect(fadeStep(4, 0.06, 0)).toBeCloseTo(4 * 0.9);
    expect(fadeStep(4, 0, 1)).toBe(4);
  });
});
