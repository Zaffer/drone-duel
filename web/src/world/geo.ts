// Frames: the physics' north-east-down and body forward-right-down, and Cesium's earth-fixed
// frame. Heights: the sim's are above sea level, the map's above the WGS84 ellipsoid; `mapOffset`
// is the difference at the start pad (map ground there less the sim's), so the two grounds meet
// where the vehicle starts (DESIGN.md §3).

import { signal } from "alien-signals";
import * as C from "cesium";

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];

/** Map height (ellipsoid) = sim height (sea level) + mapOffset. */
export const mapOffset = signal(0);

/** Rotate `v` by the unit quaternion `q` (w, x, y, z). */
export function rotate(q: Quat, v: Vec3): Vec3 {
  const [w, x, y, z] = q;
  const [vx, vy, vz] = v;
  // t = 2 q_vec × v; v' = v + w t + q_vec × t
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + (y * tz - z * ty),
    vy + w * ty + (z * tx - x * tz),
    vz + w * tz + (x * ty - y * tx),
  ];
}

export const nedToEnu = (v: Vec3): Vec3 => [v[1], v[0], -v[2]];

/** The body's forward, right and up in east-north-up, from its body→NED attitude. */
export function bodyAxesEnu(q: Quat): { fwd: Vec3; right: Vec3; up: Vec3 } {
  return {
    fwd: nedToEnu(rotate(q, [1, 0, 0])),
    right: nedToEnu(rotate(q, [0, 1, 0])),
    up: nedToEnu(rotate(q, [0, 0, -1])),
  };
}

/** Heading (clockwise from north), pitch (nose up) and roll (right down), radians. */
export function eulerOf(q: Quat): { heading: number; pitch: number; roll: number } {
  const [w, x, y, z] = q;
  return {
    roll: Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    pitch: Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    heading: Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
  };
}

/** The body→NED quaternion of heading, pitch and roll (radians). */
export function quatOf(heading: number, pitch: number, roll: number): Quat {
  const [cy, sy] = [Math.cos(heading / 2), Math.sin(heading / 2)];
  const [cp, sp] = [Math.cos(pitch / 2), Math.sin(pitch / 2)];
  const [cr, sr] = [Math.cos(roll / 2), Math.sin(roll / 2)];
  return [
    cr * cp * cy + sr * sp * sy,
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
  ];
}

/** Earth-fixed position of a point at sim height `alt` (m above sea level). */
export function toFixed(
  lat: number,
  lon: number,
  alt: number,
  result?: C.Cartesian3,
): C.Cartesian3 {
  return C.Cartesian3.fromDegrees(lon, lat, alt + mapOffset(), C.Ellipsoid.WGS84, result);
}

const scratchEnu = new C.Matrix4();
const scratchLocal = new C.Matrix4();

/** A glTF model's matrix for a body at `position` with attitude `q` (body→NED). Cesium turns a
 * model's glTF front (+Z) and up (+Y) onto its +x and +z, its left onto +y. */
export function modelMatrix(position: C.Cartesian3, q: Quat, result = new C.Matrix4()): C.Matrix4 {
  const { fwd, right, up } = bodyAxesEnu(q);
  C.Transforms.eastNorthUpToFixedFrame(position, C.Ellipsoid.WGS84, scratchEnu);
  // biome-ignore format: a matrix
  C.Matrix4.fromRowMajorArray([
    fwd[0], -right[0], up[0], 0,
    fwd[1], -right[1], up[1], 0,
    fwd[2], -right[2], up[2], 0,
    0, 0, 0, 1,
  ], scratchLocal);
  return C.Matrix4.multiply(scratchEnu, scratchLocal, result);
}

/** An east-north-up vector at `position` in the earth-fixed frame. */
export function enuToFixed(
  position: C.Cartesian3,
  v: Vec3,
  result = new C.Cartesian3(),
): C.Cartesian3 {
  C.Transforms.eastNorthUpToFixedFrame(position, C.Ellipsoid.WGS84, scratchEnu);
  return C.Matrix4.multiplyByPointAsVector(scratchEnu, new C.Cartesian3(...v), result);
}

/** A point `d` metres (east, north, up) from `position`. */
export function offsetEnu(
  position: C.Cartesian3,
  d: Vec3,
  result = new C.Cartesian3(),
): C.Cartesian3 {
  C.Transforms.eastNorthUpToFixedFrame(position, C.Ellipsoid.WGS84, scratchEnu);
  return C.Matrix4.multiplyByPoint(scratchEnu, new C.Cartesian3(...d), result);
}

export const add3 = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const scale3 = (a: Vec3, k: number): Vec3 => [a[0] * k, a[1] * k, a[2] * k];
export const len3 = (a: Vec3): number => Math.hypot(a[0], a[1], a[2]);
export const norm3 = (a: Vec3): Vec3 => scale3(a, 1 / (len3(a) || 1));

// Small offsets in lat/lon (a pose carried on along its velocity, a point a few metres off),
// flat earth, fine within kilometres. What moves or collides works in the local frame instead
// (engine/frame.ts).

/** WGS84's equatorial radius, m. */
export const EARTH_R = 6378137;

/** Metres north and east of (lat0, lon0) to (lat, lon). */
export function neFrom(lat0: number, lon0: number, lat: number, lon: number): [number, number] {
  const R = EARTH_R;
  return [
    ((lat - lat0) * Math.PI * R) / 180,
    ((lon - lon0) * Math.PI * R * Math.cos((lat0 * Math.PI) / 180)) / 180,
  ];
}

/** (lat, lon) `north`, `east` metres from (lat0, lon0). */
export function moveBy(lat0: number, lon0: number, north: number, east: number): [number, number] {
  const R = EARTH_R;
  return [
    lat0 + (north / R) * (180 / Math.PI),
    lon0 + (east / (R * Math.cos((lat0 * Math.PI) / 180))) * (180 / Math.PI),
  ];
}
