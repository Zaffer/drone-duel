// The truck's frames, as pure maths (no Cesium, no Rapier), for its pose. Rapier runs it in a
// local east-north-up frame at its start; its chassis is (+x forward, +y left, +z up). The
// page's poses are the physics' convention: attitude body (forward, right, down) to
// north-east-down, velocity north-east-down, rates about the body's forward, right, down.
//
// The local frame's axes are taken as the north-east-down axes wherever the truck is: within
// the few kilometres it drives they differ by hundredths of a degree.

export type Quat = [number, number, number, number];
export type Vec3 = [number, number, number];
/** Rapier's quaternions and vectors. */
export interface XYZW {
  x: number;
  y: number;
  z: number;
  w: number;
}
export interface XYZ {
  x: number;
  y: number;
  z: number;
}

const S = Math.SQRT1_2;
/** East-north-up to north-east-down: half a turn about (east + north). */
const ENU_TO_NED: Quat = [0, S, S, 0];
/** Body forward-right-down to the chassis' forward-left-up: half a turn about forward. */
const FRD_TO_CHASSIS: Quat = [0, 1, 0, 0];

/** Hamilton product a ⊗ b, both (w, x, y, z). */
export function quatMul(a: Quat, b: Quat): Quat {
  const [aw, ax, ay, az] = a;
  const [bw, bx, by, bz] = b;
  return [
    aw * bw - ax * bx - ay * by - az * bz,
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
  ];
}

/** Rotate `v` by the unit quaternion `q` (w, x, y, z). */
export function quatRotate(q: Quat, v: Vec3): Vec3 {
  const [w, x, y, z] = q;
  const [vx, vy, vz] = v;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ];
}

function normalize(q: Quat): Quat {
  const n = Math.hypot(...q) || 1;
  // One sign for the same attitude: w ≥ 0.
  const k = q[0] < 0 ? -1 / n : 1 / n;
  return [q[0] * k, q[1] * k, q[2] * k, q[3] * k];
}

/** The body (forward, right, down) to north-east-down attitude of a chassis whose rotation in
 * the local east-north-up frame is `q` (Rapier's). */
export function chassisToNed(q: XYZW): Quat {
  return normalize(quatMul(quatMul(ENU_TO_NED, [q.w, q.x, q.y, q.z]), FRD_TO_CHASSIS));
}

/** An east-north-up vector as north-east-down. */
export const enuToNed = (v: XYZ): Vec3 => [v.y, v.x, -v.z];

/** Body rates (about forward, right, down), rad/s, of a chassis at rotation `q` turning at
 * `angvel` (rad/s, local east-north-up), both Rapier's. */
export function bodyRates(q: XYZW, angvel: XYZ): Vec3 {
  const inverse: Quat = [q.w, -q.x, -q.y, -q.z];
  const [x, y, z] = quatRotate(inverse, [angvel.x, angvel.y, angvel.z]);
  return [x, -y, -z];
}

/** The chassis' rotation in the local frame (Rapier's) for a level truck heading `yaw` radians
 * from east, anticlockwise. */
export function yawQuat(yaw: number): XYZW {
  return { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) };
}

/** Heading true, degrees clockwise from north, to the local frame's yaw (radians from east,
 * anticlockwise), and back. */
export const yawOfHeading = (headingDeg: number): number => ((90 - headingDeg) * Math.PI) / 180;
export const headingOfYaw = (yaw: number): number => {
  const h = 90 - (yaw * 180) / Math.PI;
  return ((h % 360) + 360) % 360;
};

/** The yaw (radians from east, anticlockwise) of a chassis at rotation `q`. */
export function yawOf(q: XYZW): number {
  return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
}

/** The chassis' up axis' vertical component: 1 upright, -1 on its roof. */
export function uprightness(q: XYZW): number {
  return 1 - 2 * (q.x * q.x + q.y * q.y);
}
