import { describe, expect, test } from "bun:test";
import {
  altNeedleDeg,
  climbNeedleDeg,
  climbText,
  drumPositions,
  headingText,
  horizonDy,
  R,
  scalesFor,
  speedNeedleDeg,
  stickMeters,
} from "../src/hud/dialmath.ts";

describe("the altimeter", () => {
  test("the needle turns once every 100 m", () => {
    expect(altNeedleDeg(0, 100)).toBe(0);
    expect(altNeedleDeg(25, 100)).toBe(90);
    expect(altNeedleDeg(150, 100)).toBe(180);
    expect(altNeedleDeg(-5, 100)).toBe(0);
  });

  test("the drum rolls as an odometer: higher digits turn in the last unit", () => {
    expect(drumPositions(123)).toEqual([1, 2, 3]);
    expect(drumPositions(9.5)).toEqual([0, 0.5, 9.5]);
    const [h, t, u] = drumPositions(99.5);
    expect(h).toBeCloseTo(0.5);
    expect(t).toBeCloseTo(9.5);
    expect(u).toBeCloseTo(9.5);
    expect(drumPositions(1234, 4)).toEqual([1, 2, 3, 4]);
    // Pinned at its top.
    expect(drumPositions(5000)[0]).toBeCloseTo(10, 3);
  });
});

describe("vertical speed and ground speed", () => {
  test("VS: 9 o'clock level, 12 at full climb, 6 at full descent", () => {
    expect(climbNeedleDeg(0, 5)).toBeCloseTo(180);
    expect(climbNeedleDeg(5, 5)).toBeCloseTo(315);
    expect(climbNeedleDeg(-5, 5)).toBeCloseTo(45);
    expect(climbNeedleDeg(50, 5)).toBeCloseTo(315);
  });

  test("GS: 0 at 7:30, full scale at 4:30, clockwise", () => {
    expect(speedNeedleDeg(0, 30)).toBeCloseTo(135);
    expect(speedNeedleDeg(15, 30)).toBeCloseTo(270);
    expect(speedNeedleDeg(30, 30)).toBeCloseTo(405);
    expect(speedNeedleDeg(-3, 30)).toBeCloseTo(135);
  });

  test("scales fit the units and the vehicle", () => {
    expect(scalesFor("metric", "fixedwing").speed.full).toBe(40);
    expect(scalesFor("metric", "multirotor").speed.full).toBe(30);
    const imp = scalesFor("imperial", "multirotor");
    expect(imp.alt.turn).toBe(1000);
    expect(imp.alt.digits).toBe(4);
    expect(imp.climb.full).toBe(1000);
  });

  test("readouts", () => {
    expect(climbText(1.24, 1)).toBe("+1.2");
    expect(climbText(-0.04, 1)).toBe("+0.0");
    expect(climbText(-2.5, 1)).toBe("−2.5");
    expect(headingText(-3)).toBe("357°");
    expect(headingText(359.7)).toBe("000°");
  });
});

describe("attitude and sticks", () => {
  test("the horizon drops with the nose up, to 0.8 R at 45°", () => {
    expect(horizonDy(0)).toBe(0);
    expect(horizonDy(45)).toBeCloseTo(0.8 * R);
    expect(horizonDy(-90)).toBeCloseTo(-0.8 * R);
  });

  test("stick meters: AETR, throttle 0..1, pitch forward drawn up", () => {
    expect(stickMeters([], false)).toBeNull();
    const ap = stickMeters([0.5, -1, -1, 0.2], true);
    expect(ap).toEqual({ roll: 0.5, pitch: 1, throttle: 0, yaw: 0.2 });
    const inav = stickMeters([0, 1, 1, 0], false);
    expect(inav).toEqual({ roll: 0, pitch: 1, throttle: 1, yaw: 0 });
  });
});
