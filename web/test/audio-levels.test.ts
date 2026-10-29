import { describe, expect, test } from "bun:test";
import {
  doppler,
  droneGain,
  hz,
  motorRevs,
  proximityGap,
  QUAD_MAX_RAD_S,
  rotorNorm,
  truckEngine,
} from "../src/audio/levels.ts";

describe("sound levels", () => {
  test("notes", () => {
    expect(hz(69)).toBe(440);
    expect(hz(81)).toBeCloseTo(880, 9);
  });

  test("rotor speeds as motor()'s rpmNorm", () => {
    expect(rotorNorm("multirotor", 0)).toBe(0);
    expect(rotorNorm("multirotor", QUAD_MAX_RAD_S / 2)).toBeCloseTo(0.5, 9);
    expect(rotorNorm("multirotor", -QUAD_MAX_RAD_S)).toBe(1);
    expect(rotorNorm("fixedwing", 1e6)).toBe(1);
  });

  test("motor revs on ArduPilot's thrust curve", () => {
    expect(motorRevs(1000)).toBe(0);
    expect(motorRevs(2000)).toBeCloseTo(1, 9);
    expect(motorRevs(1500)).toBeGreaterThan(0.5);
  });

  test("the truck's diesel climbs through its gears", () => {
    expect(truckEngine(0, 0).rpm).toBe(0);
    expect(truckEngine(5.9, 0).rpm).toBeGreaterThan(truckEngine(6.1, 0).rpm); // a gear change
    expect(truckEngine(10, 1).load).toBe(1);
    expect(truckEngine(-10, -1).rpm).toBe(truckEngine(10, 1).rpm);
    expect(truckEngine(100, 1).rpm).toBe(1);
  });

  test("distance and doppler", () => {
    expect(droneGain(5)).toBe(1);
    expect(droneGain(20)).toBeCloseTo(0.5, 9);
    expect(doppler(0)).toBe(1);
    expect(doppler(30)).toBeGreaterThan(1);
    expect(doppler(-300)).toBeCloseTo(0.85, 9);
    expect(proximityGap(200)).toBeNull();
    expect(proximityGap(0)).toBeCloseTo(0.22, 9);
  });
});
