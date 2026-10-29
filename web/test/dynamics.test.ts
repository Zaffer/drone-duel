import { describe, expect, test } from "bun:test";
import {
  asDynamics,
  bodyToEnu,
  fmt,
  nameStrips,
  rampAt,
  type StripDynamics,
} from "../src/dynamics/model.ts";

const strip = (y: number, fin = false): StripDynamics =>
  ({ at: [0, y, 0], up: fin ? [0, 1, 0] : [0, 0, -1] }) as unknown as StripDynamics;

describe("the dynamics overlay's model", () => {
  test("strips named from behind: L tips in, R out, the fin last", () => {
    const named = nameStrips([strip(0.3), strip(-0.1), strip(0.1, true), strip(-0.3), strip(0.1)]);
    expect(named.map((n) => n.name)).toEqual(["L2", "L1", "R1", "R2", "fin"]);
    expect(named.map((n) => n.i)).toEqual([3, 1, 4, 0, 2]);
  });

  test("body to east-north-up: a wing heading east, level", () => {
    const axes = {
      fwd: [1, 0, 0] as [number, number, number],
      right: [0, -1, 0] as [number, number, number],
      up: [0, 0, 1] as [number, number, number],
    };
    expect(bodyToEnu(axes, [1, 0, 0])).toEqual([1, 0, 0]);
    expect(bodyToEnu(axes, [0, 1, 0])).toEqual([0, -1, 0]);
    // Down in the body is down in the world, scaled.
    expect(bodyToEnu(axes, [0, 0, 1], 2)).toEqual([0, 0, -2]);
  });

  test("numbers: no -0, a dash for nothing", () => {
    expect(fmt(-0.0001, 2)).toBe("0.00");
    expect(fmt(-1.234, 1)).toBe("-1.2");
    expect(fmt(null, 1)).toBe("–");
    expect(fmt(Number.NaN, 1)).toBe("–");
  });

  test("ramps: between their stops, held at the ends", () => {
    const stops: [number, string][] = [
      [0, "a"],
      [10, "b"],
      [20, "c"],
    ];
    expect(rampAt(stops, 5)).toEqual({ a: 0, b: 1, t: 0.5 });
    expect(rampAt(stops, 15)).toEqual({ a: 1, b: 2, t: 0.5 });
    expect(rampAt(stops, -5)).toEqual({ a: 0, b: 0, t: 0 });
    expect(rampAt(stops, 50)).toEqual({ a: 2, b: 2, t: 0 });
  });

  test("only the wing's dynamics are taken", () => {
    expect(asDynamics(null)).toBeNull();
    expect(asDynamics({ airspeed: 3 })).toBeNull();
    expect(asDynamics({ airspeed: 3, strips: [], wake: [], coefficients: [] })).not.toBeNull();
  });
});
