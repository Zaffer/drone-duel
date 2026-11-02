// A race course's gates, and when one is passed (DESIGN.md §7): pure maths, tested in
// test/gates.test.ts.
//
// A gate is a square frame (the course's GateFrame) at its centre, facing the way to fly through
// it: yaw from true north, pitch up, roll right, as an aircraft's. It is passed when the
// vehicle's path between two poses crosses its plane forwards inside the frame's inside.

import type { Gate, GateFrame } from "../../net/protocol.ts";
import { bodyAxesEnu, neFrom, quatOf, type Vec3 } from "../../world/geo.ts";

export interface GateGeom {
  lat: number;
  lon: number;
  /** The centre's sim height, m above sea level. */
  alt: number;
  fwd: Vec3;
  right: Vec3;
  up: Vec3;
  /** The way through: width, height, m. */
  width: number;
  height: number;
  /** The whole frame: width, height, thickness along the way through, m. */
  outerWidth: number;
  outerHeight: number;
  depth: number;
}

/** Margin round a gate's inside that still counts, m (a vehicle's half-size). */
export const MARGIN_M = 0.25;

/** A gate's geometry, its frame the course's, its centre at sim height `centreAlt`. */
export function gateGeom(g: Gate, frame: GateFrame, centreAlt: number): GateGeom {
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
    width: frame.inside_m[0],
    height: frame.inside_m[1],
    outerWidth: frame.outside_m[0],
    outerHeight: frame.outside_m[1],
    depth: frame.depth_m,
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

/** A point `across` m right of and `up` m above a gate's centre, as east, north, up from it. */
export function inPlane(g: GateGeom, across: number, up: number): Vec3 {
  return [
    g.right[0] * across + g.up[0] * up,
    g.right[1] * across + g.up[1] * up,
    g.right[2] * across + g.up[2] * up,
  ];
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

  /** The gates (0..n-1) to fly through next, soonest first: up to `count`, none past the finish. */
  upcoming(count = 3): number[] {
    const left = this.n * this.lapsTotal - this.next;
    return Array.from(
      { length: Math.max(0, Math.min(count, left)) },
      (_, i) => (this.next + i) % this.n,
    );
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
