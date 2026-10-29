import { describe, expect, test } from "bun:test";
import { drawnRate, PropSpin, QUAD_SPIN, WING_SPIN } from "../src/vehicles/props.ts";

describe("drawn propeller spin", () => {
  test("stopped rotors stand still", () => {
    const p = new PropSpin(QUAD_SPIN);
    for (let i = 0; i < 60; i++) p.step(0, 1 / 60);
    expect(p.angle).toBe(0);
    expect(drawnRate(0, QUAD_SPIN)).toBe(0);
  });

  test("faster rotors are drawn faster, up to the ceiling", () => {
    const idle = drawnRate(300, QUAD_SPIN);
    const hover = drawnRate(1500, QUAD_SPIN);
    const full = drawnRate(3000, QUAD_SPIN);
    expect(idle).toBeGreaterThan(0);
    expect(hover).toBeGreaterThan(idle);
    expect(full).toBeGreaterThan(hover);
    expect(drawnRate(1e9, QUAD_SPIN)).toBeCloseTo(QUAD_SPIN.maxRadS, 6);
    // Either way round is the same speed; nonsense is stopped.
    expect(drawnRate(-1500, QUAD_SPIN)).toBe(hover);
    expect(drawnRate(Number.NaN, QUAD_SPIN)).toBe(0);
  });

  test("a 2000 rad/s rotor never turns more than its step limit in a frame", () => {
    for (const look of [QUAD_SPIN, WING_SPIN]) {
      for (const fps of [144, 60, 30, 10]) {
        const p = new PropSpin(look);
        let prev = 0;
        for (let i = 0; i < 5 * fps; i++) {
          const a = p.step(look === QUAD_SPIN ? 2000 : 4500, 1 / fps);
          const d = (a - prev + 2 * Math.PI) % (2 * Math.PI);
          expect(d).toBeLessThanOrEqual(look.maxStepRad + 1e-12);
          prev = a;
        }
      }
    }
  });

  test("the step limit is under half the blade spacing (no backwards strobe)", () => {
    expect(QUAD_SPIN.maxStepRad).toBeLessThan((2 * Math.PI) / 3 / 2); // tri-blades
    expect(WING_SPIN.maxStepRad).toBeLessThan((2 * Math.PI) / 3 / 2);
  });

  test("the drawn rate settles towards the rotor's", () => {
    const p = new PropSpin(QUAD_SPIN);
    for (let i = 0; i < 120; i++) p.step(1500, 1 / 60);
    expect(p.rate).toBeCloseTo(drawnRate(1500, QUAD_SPIN), 2);
    for (let i = 0; i < 120; i++) p.step(0, 1 / 60);
    expect(p.rate).toBeLessThan(0.01);
  });

  test("the angle stays within one turn", () => {
    const p = new PropSpin(WING_SPIN);
    for (let i = 0; i < 1000; i++) p.step(4500, 1 / 60);
    expect(p.angle).toBeGreaterThanOrEqual(0);
    expect(p.angle).toBeLessThan(2 * Math.PI);
  });
});
