// The truck's Rapier physics on a flat ground, as the old drive tuned it: it settles on its
// springs, drives, brakes, reverses, and steers the right way.
import { beforeAll, describe, expect, test } from "bun:test";
import RAPIER from "@dimforge/rapier3d-compat";
import {
  Car,
  driveForce,
  STEER_MAX_RAD,
  steerLock,
  TOP_SPEED_MS,
} from "../src/vehicles/truck/car.ts";
import { chassisToNed, headingOfYaw } from "../src/vehicles/truck/frame.ts";

const STEP = 1 / 60;

beforeAll(async () => {
  await RAPIER.init();
});

function setup(yaw = 0) {
  const world = new RAPIER.World({ x: 0, y: 0, z: -9.81 });
  world.timestep = STEP;
  world.createCollider(
    RAPIER.ColliderDesc.cuboid(2000, 2000, 1).setTranslation(0, 0, -1).setFriction(0.9),
  );
  const car = new Car(RAPIER, world);
  car.place({ x: 0, y: 0, z: 0.4 }, yaw);
  const run = (s: number, input = { throttle: 0, steer: 0, handbrake: false }) => {
    for (let i = 0; i < s / STEP; i++) {
      car.drive(STEP, input);
      world.step();
    }
  };
  return { world, car, run };
}

describe("the truck's physics", () => {
  test("throttle against the motion brakes, then reverses", () => {
    expect(driveForce(1, 0).force).toBeGreaterThan(0);
    expect(driveForce(1, -3).force).toBe(0);
    expect(driveForce(1, -3).brake).toBeGreaterThan(driveForce(0, 0).brake);
    expect(driveForce(-1, 5).force).toBe(0);
    expect(driveForce(-1, 0).force).toBeLessThan(0);
    expect(driveForce(1, TOP_SPEED_MS).force).toBe(0);
    expect(steerLock(0)).toBe(STEER_MAX_RAD);
    expect(steerLock(14)).toBeCloseTo(STEER_MAX_RAD / 2, 9);
  });

  test("it settles on its four wheels", () => {
    const { car, run, world } = setup();
    run(2);
    expect(car.wheelsOnGround).toBe(4);
    expect(car.uprightness).toBeGreaterThan(0.999);
    // At rest: Rapier's ray-cast vehicle reports a few cm/s of jitter, but it stays put.
    expect(Math.abs(car.speed)).toBeLessThan(0.1);
    const before = car.body.translation();
    run(3);
    const after = car.body.translation();
    expect(Math.hypot(after.x - before.x, after.y - before.y, after.z - before.z)).toBeLessThan(
      0.01,
    );
    // The chassis' origin is on the ground at rest, give or take the springs.
    expect(Math.abs(car.body.translation().z)).toBeLessThan(0.15);
    world.free();
  });

  test("it drives forward, stops, and reverses", () => {
    const { car, run, world } = setup();
    run(1);
    run(5, { throttle: 1, steer: 0, handbrake: false });
    const fast = car.speed;
    expect(fast).toBeGreaterThan(8);
    expect(fast).toBeLessThan(TOP_SPEED_MS);
    expect(car.body.translation().x).toBeGreaterThan(20); // facing east
    run(4, { throttle: -1, steer: 0, handbrake: false });
    expect(car.speed).toBeLessThan(fast);
    run(6, { throttle: -1, steer: 0, handbrake: false });
    expect(car.speed).toBeLessThan(-1);
    expect(car.speed).toBeGreaterThan(-8.5);
    world.free();
  });

  test("steer + (the chassis' left) turns it left", () => {
    const { car, run, world } = setup(Math.PI / 2); // north
    run(1);
    run(3, { throttle: 0.6, steer: 1, handbrake: false });
    // Heading true goes down from 0 (to the west).
    const h = headingOfYaw(car.yaw);
    expect(h).toBeGreaterThan(180);
    expect(h).toBeLessThan(359);
    expect(car.uprightness).toBeGreaterThan(0.9);
    // And its attitude in north-east-down says the same.
    const [w, , , z] = chassisToNed(car.body.rotation());
    const heading = (Math.atan2(2 * w * z, 1 - 2 * z * z) * 180) / Math.PI;
    expect((heading + 360) % 360).toBeCloseTo(h, 0);
    world.free();
  });
});
