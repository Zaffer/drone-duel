// A race course's gates, and when one is passed (DESIGN.md §7): pure maths, tested in
// test/gates.test.ts.
//
// A gate is a rectangle of `size` (width, height, inside the frame) at its centre, facing the
// way to fly through it: yaw from true north, pitch up, roll right, as an aircraft's. It is passed
// when the vehicle's path between two poses crosses its plane forwards inside the rectangle.

import type { Gate } from "../../net/protocol.ts";
import { bodyAxesEnu, neFrom, quatOf, type Vec3 } from "../../world/geo.ts";

export interface GateGeom {
  lat: number;
  lon: number;
  /** The centre's sim height, m above sea level. */
  alt: number;
  fwd: Vec3;
  right: Vec3;
  up: Vec3;
  width: number;
  height: number;
}

/** Margin round a gate's inside that still counts, m (a vehicle's half-size). */
export const MARGIN_M = 0.25;

/** A gate's geometry, its centre `groundAlt` + its alt_m above sea level for an AGL course. */
export function gateGeom(g: Gate, centreAlt: number): GateGeom {
  const d = Math.PI / 180;
  const [yaw, pitch, roll] = g.ypr_deg;
  const { fwd, right, up } = bodyAxesEnu(quatOf(yaw * d, pitch * d, roll * d));
  return {
    lat: g.at[0],
    lon: g.at[1],
    alt: centreAlt,
    fwd,
    right,
    up,
    width: g.size_m[0],
    height: g.size_m[1],
  };
}

const dot = (a: Vec3, b: Vec3) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

/** A point's east, north, up from the gate's centre, m. */
export function relative(g: GateGeom, lat: number, lon: number, alt: number): Vec3 {
  const [n, e] = neFrom(g.lat, g.lon, lat, lon);
  return [e, n, alt - g.alt];
}

/** Whether going from a to b (lat, lon, sim alt) passes through `g` forwards. */
export function passes(
  g: GateGeom,
  a: { lat: number; lon: number; alt: number },
  b: { lat: number; lon: number; alt: number },
): boolean {
  const p0 = relative(g, a.lat, a.lon, a.alt);
  const p1 = relative(g, b.lat, b.lon, b.alt);
  const d0 = dot(p0, g.fwd);
  const d1 = dot(p1, g.fwd);
  if (!(d0 < 0 && d1 >= 0)) return false;
  const s = d0 / (d0 - d1);
  const x: Vec3 = [
    p0[0] + (p1[0] - p0[0]) * s,
    p0[1] + (p1[1] - p0[1]) * s,
    p0[2] + (p1[2] - p0[2]) * s,
  ];
  return (
    Math.abs(dot(x, g.right)) <= g.width / 2 + MARGIN_M &&
    Math.abs(dot(x, g.up)) <= g.height / 2 + MARGIN_M
  );
}

/** The four corners of a gate's inside, as east, north, up offsets from its centre. */
export function corners(g: GateGeom): Vec3[] {
  const w = g.width / 2;
  const h = g.height / 2;
  return [
    [-w, -h],
    [w, -h],
    [w, h],
    [-w, h],
  ].map(([x, y]) => [
    g.right[0] * x! + g.up[0] * y!,
    g.right[1] * x! + g.up[1] * y!,
    g.right[2] * x! + g.up[2] * y!,
  ]);
}

/** Progress through a course of `n` gates × `laps`: which gate is next, and the lap times. */
export class Progress {
  next = 0;
  readonly splits: number[] = [];
  readonly laps: number[] = [];
  private lapStart = 0;

  constructor(
    readonly n: number,
    readonly lapsTotal: number,
  ) {}

  get done(): boolean {
    return this.next >= this.n * this.lapsTotal;
  }

  get lap(): number {
    return Math.min(this.lapsTotal, Math.floor(this.next / this.n) + 1);
  }

  /** The index (0..n-1) of the gate to fly through next. */
  get gate(): number {
    return this.next % this.n;
  }

  /** Gate passed at `t` s since the start. Returns true when that ended a lap. */
  pass(t: number): boolean {
    this.splits.push(t);
    this.next++;
    if (this.next % this.n === 0) {
      this.laps.push(t - this.lapStart);
      this.lapStart = t;
      return true;
    }
    return false;
  }
}

/** m:ss.cc */
export function fmtTime(s: number): string {
  const m = Math.floor(s / 60);
  const rest = s - m * 60;
  return `${m}:${rest.toFixed(2).padStart(5, "0")}`;
}
