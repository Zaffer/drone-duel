import { describe, expect, test } from "bun:test";
import { pilotOffset } from "../src/people/operator.ts";

describe("pilotOffset", () => {
  test("behind the pad and to its left", () => {
    // Facing north: behind is south, left is west.
    const [e0, n0] = pilotOffset(0);
    expect(e0).toBeCloseTo(-1.5);
    expect(n0).toBeCloseTo(-3);
    // Facing south (AreaXO's pads): behind is north, left is east.
    const [e1, n1] = pilotOffset(180);
    expect(e1).toBeCloseTo(1.5);
    expect(n1).toBeCloseTo(3);
    // Facing east: behind is west, left is north.
    const [e2, n2] = pilotOffset(90);
    expect(e2).toBeCloseTo(-3);
    expect(n2).toBeCloseTo(1.5);
  });
});
