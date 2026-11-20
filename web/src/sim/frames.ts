// The simulator's frames, as the game's physics has them (drone_duels.physics.core.rigidbody,
// drone_duels.world.geo): the world north-east-down (NED), metres from the start pad; the body
// forward-right-down (FRD); attitude a unit quaternion (w, x, y, z) turning body vectors into
// world ones. Plain arrays, and nothing of the page's (the simulator runs in a worker).

export type Vec = [number, number, number];
export type Quat = [number, number, number, number];

/** Body vector `v` in the world. */
export function rotate(q: Readonly<Quat>, v: Readonly<Vec>): Vec {
  const [w, x, y, z] = q;
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * tx + y * tz - z * ty,
    v[1] + w * ty + z * tx - x * tz,
    v[2] + w * tz + x * ty - y * tx,
  ];
}

/** World vector `v` in the body. */
export function unrotate(q: Readonly<Quat>, v: Readonly<Vec>): Vec {
  return rotate([q[0], -q[1], -q[2], -q[3]], v);
}

/** The heading of `q`, rad: aerospace Z-Y-X Euler angles' yaw. */
export function yawOf(q: Readonly<Quat>): number {
  const [w, x, y, z] = q;
  return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

/** Level, nose on `headingDeg` true. */
export function yawQuat(headingDeg: number): Quat {
  const half = (headingDeg * Math.PI) / 360;
  return [Math.cos(half), 0, 0, Math.sin(half)];
}

/** The angle between the body's down and the world's, rad. */
export function tilt(q: Readonly<Quat>): number {
  return Math.acos(Math.max(-1, Math.min(1, rotate(q, [0, 0, 1])[2])));
}

const EARTH_RADIUS_M = 6378137;
const RAD = Math.PI / 180;

/** A local tangent plane anchored at an origin: flat earth, good to a few kilometres. */
export class LocalFrame {
  private readonly cos: number;

  constructor(
    readonly lat: number,
    readonly lon: number,
  ) {
    this.cos = Math.cos(lat * RAD);
  }

  toLatLon(north: number, east: number): [number, number] {
    return [
      this.lat + north / EARTH_RADIUS_M / RAD,
      this.lon + east / (EARTH_RADIUS_M * this.cos) / RAD,
    ];
  }
}
