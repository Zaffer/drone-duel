import { describe, expect, test } from "bun:test";
import {
  bodyRates,
  chassisToNed,
  enuToNed,
  headingOfYaw,
  type Quat,
  quatMul,
  quatRotate,
  type XYZW,
  yawOfHeading,
  yawQuat,
} from "../src/vehicles/truck/frame.ts";

const D = Math.PI / 180;

/** geo.eulerOf, here without Cesium: heading, pitch (nose up), roll (right down), rad. */
function eulerOf(q: Quat) {
  const [w, x, y, z] = q;
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    heading: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  };
}

const axisAngle = (axis: [number, number, number], angle: number): XYZW => {
  const s = Math.sin(angle / 2);
  return { x: axis[0] * s, y: axis[1] * s, z: axis[2] * s, w: Math.cos(angle / 2) };
};
const xyzw = (q: Quat): XYZW => ({ w: q[0], x: q[1], y: q[2], z: q[3] });
const asQuat = (q: XYZW): Quat => [q.w, q.x, q.y, q.z];

describe("the truck's pose from its local frame", () => {
  test("headings and the local yaw agree", () => {
    for (const h of [0, 45, 90, 180, 270, 359]) {
      expect(headingOfYaw(yawOfHeading(h))).toBeCloseTo(h, 9);
    }
    expect(yawOfHeading(90)).toBeCloseTo(0, 12); // east
    expect(yawOfHeading(0)).toBeCloseTo(Math.PI / 2, 12); // north
  });

  test("a level truck's attitude has its heading and no pitch or roll", () => {
    for (const h of [0, 30, 90, 135, 200, 300]) {
      const e = eulerOf(chassisToNed(yawQuat(yawOfHeading(h))));
      expect((((e.heading / D) % 360) + 360) % 360).toBeCloseTo(h, 6);
      expect(e.pitch).toBeCloseTo(0, 9);
      expect(e.roll).toBeCloseTo(0, 9);
    }
  });

  test("its body axes land where they should in north-east-down", () => {
    // Facing east: forward is east, right is south, down is down.
    const q = chassisToNed(yawQuat(0));
    const near = (a: number[], b: number[]) => {
      for (const [i, v] of a.entries()) expect(v).toBeCloseTo(b[i]!, 9);
    };
    near(quatRotate(q, [1, 0, 0]), [0, 1, 0]);
    near(quatRotate(q, [0, 1, 0]), [-1, 0, 0]);
    near(quatRotate(q, [0, 0, 1]), [0, 0, 1]);
  });

  test("nose up is positive pitch, right side down is positive roll", () => {
    // Facing north; pitched 10° nose up: about the chassis' left axis (+y) by -10°.
    const north = yawQuat(Math.PI / 2);
    const up = asQuat(north);
    const pitched = xyzw(quatMul(up, asQuat(axisAngle([0, 1, 0], -10 * D))));
    const e1 = eulerOf(chassisToNed(pitched));
    expect(e1.pitch / D).toBeCloseTo(10, 6);
    expect(e1.roll).toBeCloseTo(0, 9);
    expect(e1.heading).toBeCloseTo(0, 9);
    // Rolled right side down 15°: about the chassis' forward axis (+x) by +15°.
    const rolled = xyzw(quatMul(up, asQuat(axisAngle([1, 0, 0], 15 * D))));
    const e2 = eulerOf(chassisToNed(rolled));
    expect(e2.roll / D).toBeCloseTo(15, 6);
    expect(e2.pitch).toBeCloseTo(0, 9);
  });

  test("the attitude is a unit quaternion with w ≥ 0", () => {
    const q = chassisToNed(axisAngle([0.3, -0.5, 0.81], 2.9));
    expect(Math.hypot(...q)).toBeCloseTo(1, 12);
    expect(q[0]).toBeGreaterThanOrEqual(0);
  });

  test("velocity: east-north-up to north-east-down", () => {
    expect(enuToNed({ x: 3, y: 4, z: 5 })).toEqual([4, 3, -5]);
  });

  test("body rates: yawing left is negative r, in any heading", () => {
    for (const yaw of [0, 1, 2.5]) {
      const r = bodyRates(yawQuat(yaw), { x: 0, y: 0, z: 0.5 });
      expect(r[0]).toBeCloseTo(0, 12);
      expect(r[1]).toBeCloseTo(0, 12);
      expect(r[2]).toBeCloseTo(-0.5, 12);
    }
    // Facing east, rolling right side down: turning about east, +p.
    const p = bodyRates(yawQuat(0), { x: 0.2, y: 0, z: 0 });
    expect(p[0]).toBeCloseTo(0.2, 12);
    // Facing east, turning about north (the truck's left) +: the nose goes down, -q.
    const q = bodyRates(yawQuat(0), { x: 0, y: 0.3, z: 0 });
    expect(q[1]).toBeCloseTo(-0.3, 12);
  });
});
