import { describe, expect, test } from "bun:test";
import {
  airAt,
  cdG7,
  type Load,
  Round,
  setting,
  spinDrift,
  stability,
  zeroAngle,
} from "../src/shoot/ballistics.ts";
import type { Vec3 } from "../src/world/geo.ts";

/** M193 from an 11.5" 1:7 barrel: content/weapons/ar15-sbr.toml. */
const M193: Load = {
  massKg: 0.003564,
  diameterM: 0.0057,
  lengthM: 0.0188,
  bcG7: 0.124,
  muzzleMs: 853,
  twistM: 7 * 0.0254,
};

/** Fly north, level, until `y` m downrange: the round there (interpolated). */
function flyTo(r: Round, y: number): { p: Vec3; t: number; speed: number } {
  for (;;) {
    const before = { p: [...r.p] as Vec3, t: r.t, speed: r.speed };
    r.step();
    if (r.p[1] >= y) {
      const k = (y - before.p[1]) / (r.p[1] - before.p[1]);
      const lerp = (a: number, b: number) => a + k * (b - a);
      return {
        p: [lerp(before.p[0], r.p[0]), y, lerp(before.p[2], r.p[2])],
        t: lerp(before.t, r.t),
        speed: lerp(before.speed, r.speed),
      };
    }
  }
}

describe("the G7 table and the air", () => {
  test("Cd at the table's points and between", () => {
    expect(cdG7(1.0)).toBeCloseTo(0.3803, 6);
    expect(cdG7(2.5)).toBeCloseTo(0.2697, 6);
    expect(cdG7(2.525)).toBeCloseTo((0.2697 + 0.267) / 2, 6);
    expect(cdG7(9)).toBeCloseTo(0.1618, 6);
  });

  test("ICAO standard atmosphere", () => {
    const sea = airAt(0);
    expect(sea.density).toBeCloseTo(1.225, 3);
    expect(sea.sound).toBeCloseTo(340.29, 1);
    // ISA at 1000 m: 1.1117 kg/m³, 336.43 m/s.
    expect(airAt(1000).density).toBeCloseTo(1.1117, 3);
    expect(airAt(1000).sound).toBeCloseTo(336.43, 1);
  });
});

describe("a bullet's flight", () => {
  test("without drag it falls ½gt² (no air)", () => {
    const where = setting(0, 0);
    const vacuum = { ...where, air: { ...where.air, density: 0 } };
    const r = new Round(M193, vacuum, [0, 0, 0], [0, 1, 0]);
    const at = flyTo(r, 500);
    const t = 500 / 853;
    expect(at.t).toBeCloseTo(t, 4);
    expect(at.p[2]).toBeCloseTo(-0.5 * where.g * t * t, 3);
  });

  test("M193 slows as it does on G1 with its published G1 coefficients", () => {
    // The same flight on the G1 table with M193's G1 BC 0.243–0.272 (makers' figures) keeps
    // 729–742 m/s at 100 m and 515–546 at 300 m from 853 m/s.
    const r = new Round(M193, setting(45, 0), [0, 0, 0], [0, 1, 0]);
    const at100 = flyTo(r, 100);
    expect(at100.speed).toBeGreaterThan(720);
    expect(at100.speed).toBeLessThan(750);
    const at300 = flyTo(r, 300);
    expect(at300.speed).toBeGreaterThan(505);
    expect(at300.speed).toBeLessThan(560);
  });

  test("a 50 m zero: on the sight line at 50 m, and again near 160 m", () => {
    // From a 20" barrel the far crossing is near 200 m; at 853 m/s, 157–161 m on G1.
    const where = setting(45, 0);
    const sight = 0.066;
    const angle = zeroAngle(M193, where, sight, 50);
    const calm = { ...where, latDeg: 0 };
    const r = new Round(M193, calm, [0, 0, 0], [0, Math.cos(angle), Math.sin(angle)]);
    expect(flyTo(r, 50).p[2]).toBeCloseTo(sight, 3);
    // The far crossing.
    let last = flyTo(r, 100).p[2] - sight;
    expect(last).toBeGreaterThan(0);
    let far = 0;
    for (let y = 110; y <= 400; y += 10) {
      const h = flyTo(r, y).p[2] - sight;
      if (last > 0 && h <= 0) far = y;
      last = h;
    }
    expect(far).toBeGreaterThan(140);
    expect(far).toBeLessThan(185);
  });

  test("a steady crosswind drifts it by Didion's lag rule, w (t − x/v0)", () => {
    const w = 5;
    const where = setting(0, 0, () => [w, 0, 0]);
    const r = new Round(M193, { ...where, latDeg: 0 }, [0, 0, 0], [0, 1, 0]);
    const at = flyTo(r, 300);
    const lag = w * (at.t - 300 / 853);
    // Spin drift adds to the right (east) too; take the point mass alone.
    expect(r.p[0]).toBeGreaterThan(0);
    expect(at.p[0]).toBeCloseTo(lag, 2);
  });

  test("a wind from the left lifts a right-hand twist (aerodynamic jump)", () => {
    // Firing north, a wind from the west is from the left.
    const fromLeft = setting(0, 0, () => [5, 0, 0]);
    const fromRight = setting(0, 0, () => [-5, 0, 0]);
    const up = flyTo(new Round(M193, fromLeft, [0, 0, 0], [0, 1, 0]), 300).p[2];
    const down = flyTo(new Round(M193, fromRight, [0, 0, 0], [0, 1, 0]), 300).p[2];
    expect(up).toBeGreaterThan(down);
  });

  test("Coriolis: north in the northern hemisphere goes right; east rises (Eötvös)", () => {
    const north = new Round(M193, setting(45, 0), [0, 0, 0], [0, 1, 0]);
    const at = flyTo(north, 500);
    expect(north.p[0]).toBeGreaterThan(0);
    expect(north.p[0]).toBeLessThan(0.05);
    const east = new Round(M193, setting(45, 0), [0, 0, 0], [1, 0, 0]);
    const west = new Round(M193, setting(45, 0), [0, 0, 0], [-1, 0, 0]);
    for (let i = 0; i < 1000; i++) {
      east.step();
      west.step();
    }
    expect(east.p[2]).toBeGreaterThan(west.p[2]);
    expect(at.t).toBeGreaterThan(0.7);
  });

  test("spin drift: stable, and an inch or so to the right at 300 m", () => {
    const sg = stability(M193, 853, airAt(0));
    expect(sg).toBeGreaterThan(1.5);
    expect(sg).toBeLessThan(4);
    const r = new Round(M193, setting(0, 0), [0, 0, 0], [0, 1, 0]);
    const at = flyTo(r, 300);
    const sd = spinDrift(r.sg, at.t);
    expect(sd).toBeGreaterThan(0.01);
    expect(sd).toBeLessThan(0.06);
    // to the right of a shot north: east.
    expect(r.at()[0]).toBeGreaterThan(r.p[0]);
  });

  test("energy falls as it flies", () => {
    const r = new Round(M193, setting(45, 70), [0, 0, 0], [0, 1, 0]);
    const e0 = r.energy;
    expect(e0).toBeCloseTo(0.5 * 0.003564 * 853 * 853, 3);
    flyTo(r, 200);
    expect(r.energy).toBeLessThan(e0);
  });
});
