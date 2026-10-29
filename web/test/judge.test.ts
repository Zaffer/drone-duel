import { describe, expect, test } from "bun:test";
import { strikes, winner } from "../src/modes/duel/judge.ts";
import { moveBy, quatOf } from "../src/world/geo.ts";

const LAT = 43.63;
const LON = -79.39;
// The truck facing east, on ground 40 m above the ellipsoid.
const truck = { lat: LAT, lon: LON, h: 40, q: quatOf(Math.PI / 2, 0, 0) };
const at = (east: number, north: number, up: number) => {
  const [lat, lon] = moveBy(LAT, LON, north, east);
  return { lat, lon, h: 40 + up };
};

describe("strikes", () => {
  test("on the truck's bonnet and roof", () => {
    expect(strikes(truck, at(2.5, 0, 1))).toBe(true);
    expect(strikes(truck, at(0, 0, 3.2))).toBe(true);
  });
  test("along its length, not across it (it faces east)", () => {
    expect(strikes(truck, at(2.8, 0, 1))).toBe(true);
    expect(strikes(truck, at(0, 2.8, 1))).toBe(false);
  });
  test("well above it, or to the side, misses", () => {
    expect(strikes(truck, at(0, 0, 5))).toBe(false);
    expect(strikes(truck, at(0, 3, 1))).toBe(false);
  });
  test("winner", () => {
    const m = { phase: "over" as const, turn: 3, turns: 4, endsAt: 0, lives: 3, minutes: 2 };
    expect(winner({ ...m, scores: { host: 2, guest: 1 } })).toBe("host");
    expect(winner({ ...m, scores: { host: 1, guest: 1 } })).toBeNull();
  });
});
