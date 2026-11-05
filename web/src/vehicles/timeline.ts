// A vehicle's pose over time (DESIGN.md §3.1): samples kept by the time they carry (the physics'
// sim time, or a peer's clock), never by when the page handled them. The page handles the
// WebSocket on the thread that draws, so poses arrive in clumps between frames, some 100 ms
// late; drawn by their arrival, the aircraft surged, stalled and stepped back.
//
// The page learns how the samples' clock maps to its own (performance.now()) from the quickest
// deliveries: the least of (arrival - t) over the last WINDOW_MS, which the clumps cannot raise.
// That offset slews at most SLEW of real time, so a change in it never shows as a jump.
//
// `sample(now, delay)` is the pose at the samples' time `now` less `delay`: between two samples,
// the cubic through both with their velocities, and the attitude turned between them (slerp);
// past the newest, on along its velocity and body rates for up to MAX_AHEAD_S, then held. Our own
// aircraft is sampled with no delay; a peer's a jitter buffer behind (`buffer()`).

import type { Quat, Vec3 } from "../world/geo.ts";

export interface Sample {
  /** On the source's clock, s: the physics' sim time, or a peer's. */
  t: number;
  lat: number;
  lon: number;
  /** m, in whatever heights the source uses. */
  alt: number;
  /** Attitude, body (forward, right, down) to north-east-down: w, x, y, z. */
  q: Quat;
  /** North, east, down, m/s. */
  v: Vec3;
  /** Body rates, rad/s: the attitude turns on with them past the newest sample. */
  rates?: Vec3;
  /** Actuators (rotor speeds, elevons), blended between samples. */
  act?: [number, number, number, number];
}

/** The window the clock offset is the least over, ms. */
export const WINDOW_MS = 2000;
/** The offset moves at most this much per ms of real time. */
export const SLEW = 0.05;
/** Every sample this much later than the offset says (ms), for SLIP_FOR_MS, means the source's
 * clock slipped (the physics stalled): the offset jumps to them. One late clump (the page itself
 * stalled, loading tiles) is not a slip. */
export const SLIP_MS = 150;
const SLIP_FOR_MS = 250;
/** Past the newest sample, carried on this long at most, s. */
export const MAX_AHEAD_S = 0.1;
/** Samples kept, s of their clock. */
const KEEP_S = 1;
/** An earlier sample than the newest by this much starts the timeline again (a respawn). */
const RESTART_S = 0.5;
/** Recent lateness kept, for the jitter buffer. */
const LATENESS_KEPT = 90;
/** The jitter buffer: to start with, at least and at most, s; it moves at most BUFFER_SLEW of
 * real time. */
const BUFFER = { start: 0.1, min: 0.06, max: 0.25 };
const BUFFER_SLEW = 0.05;

const R = 6378137;
const DEG = 180 / Math.PI;

export class Timeline<S extends Sample> {
  private samples: S[] = [];
  /** Each a quarter of the window: the least lateness (arrival - t, ms) arrived in it. */
  private buckets: { start: number; least: number }[] = [];
  private offset: number | null = null;
  private slewedAt = 0;
  /** Since when every sample has been SLIP_MS late, ms; null while they are not. */
  private slipSince: number | null = null;
  private late: number[] = [];
  private bufferS = BUFFER.start;
  private bufferAt = 0;

  /** A sample, as it arrives at `arrived` (performance.now(), ms). */
  push(s: S, arrived: number): void {
    const newest = this.samples[this.samples.length - 1];
    if (newest && s.t < newest.t - RESTART_S) this.clear();
    const lateness = arrived - s.t * 1000;
    if (this.offset !== null && lateness - this.offset > SLIP_MS) {
      this.slipSince ??= arrived;
      if (arrived - this.slipSince >= SLIP_FOR_MS) {
        // The source's clock fell behind ours: begin the window again from here.
        this.buckets = [];
        this.offset = lateness;
        this.slipSince = null;
      }
    } else {
      this.slipSince = null;
    }
    this.note(lateness, arrived);
    if (this.offset !== null) {
      this.late.push(lateness - this.offset);
      if (this.late.length > LATENESS_KEPT) this.late.shift();
    }
    this.insert(s);
  }

  clear(): void {
    this.samples = [];
    this.buckets = [];
    this.offset = null;
    this.slipSince = null;
    this.late = [];
  }

  get newest(): S | null {
    return this.samples[this.samples.length - 1] ?? null;
  }

  /** The samples' time at `now` (performance.now(), ms), s; null before the first sample. */
  timeAt(now: number): number | null {
    const offset = this.clock(now);
    return offset === null ? null : (now - offset) / 1000;
  }

  /** The pose at the samples' time `now` less `delay` s; null before the first sample. */
  sample(now: number, delay = 0): S | null {
    const at = this.timeAt(now);
    if (at === null) return null;
    return this.at(at - delay);
  }

  /** How far past the newest sample `sample(now, delay)` carries on, s (≤ 0: between samples). */
  ahead(now: number, delay = 0): number {
    const at = this.timeAt(now);
    const newest = this.newest;
    return at === null || !newest ? 0 : at - delay - newest.t;
  }

  /** A jitter buffer for a peer's poses, s: the 95th percentile of their lateness past the
   * quickest, plus one interval between them, held within BUFFER; it moves slowly, so the other
   * player never visibly speeds up or slows down for it. */
  buffer(now: number): number {
    const dt = this.bufferAt ? Math.max(0, now - this.bufferAt) / 1000 : 0;
    this.bufferAt = now;
    if (this.late.length >= 20) {
      const sorted = [...this.late].sort((a, b) => a - b);
      const p95 = sorted[Math.floor(0.95 * (sorted.length - 1))]! / 1000;
      const target = Math.min(BUFFER.max, Math.max(BUFFER.min, p95 + this.interval()));
      const step = BUFFER_SLEW * dt;
      this.bufferS += Math.max(-step, Math.min(step, target - this.bufferS));
    }
    return this.bufferS;
  }

  // -- inside --------------------------------------------------------------------------------

  private insert(s: S): void {
    const xs = this.samples;
    let i = xs.length;
    while (i > 0 && xs[i - 1]!.t > s.t) i--;
    if (i > 0 && xs[i - 1]!.t === s.t) return; // twice: keep the first
    xs.splice(i, 0, s);
    const newest = xs[xs.length - 1]!;
    while (xs.length > 2 && newest.t - xs[0]!.t > KEEP_S) xs.shift();
  }

  private note(lateness: number, arrived: number): void {
    const size = WINDOW_MS / 4;
    const last = this.buckets[this.buckets.length - 1];
    if (!last || arrived - last.start >= size)
      this.buckets.push({ start: arrived, least: lateness });
    else last.least = Math.min(last.least, lateness);
    while (this.buckets.length > 1 && arrived - this.buckets[0]!.start > WINDOW_MS) {
      this.buckets.shift();
    }
    this.offset ??= lateness;
  }

  /** The offset (arrival - t, ms) at `now`, slewed toward the window's least. */
  private clock(now: number): number | null {
    if (this.offset === null || this.buckets.length === 0) return this.offset;
    const target = Math.min(...this.buckets.map((b) => b.least));
    const dt = Math.max(0, now - this.slewedAt);
    if (now > this.slewedAt) this.slewedAt = now;
    const step = SLEW * dt;
    this.offset += Math.max(-step, Math.min(step, target - this.offset));
    return this.offset;
  }

  private interval(): number {
    const xs = this.samples;
    return xs.length > 1 ? (xs[xs.length - 1]!.t - xs[0]!.t) / (xs.length - 1) : 0;
  }

  /** The pose at time `t` of the samples' clock. */
  private at(t: number): S | null {
    const xs = this.samples;
    const newest = xs[xs.length - 1];
    if (!newest) return null;
    if (t >= newest.t) return carryOn(newest, Math.min(t - newest.t, MAX_AHEAD_S));
    let i = xs.length - 1;
    while (i > 0 && xs[i - 1]!.t > t) i--;
    if (i === 0) return xs[0]!;
    return between(xs[i - 1]!, xs[i]!, t);
  }
}

/** `s` carried on `dt` s along its velocity, its attitude turned by its body rates. */
export function carryOn<S extends Sample>(s: S, dt: number): S {
  if (dt <= 0) return s;
  const [vn, ve, vd] = s.v;
  return {
    ...s,
    t: s.t + dt,
    lat: s.lat + ((vn * dt) / R) * DEG,
    lon: s.lon + ((ve * dt) / (R * Math.cos(s.lat / DEG))) * DEG,
    alt: s.alt - vd * dt,
    q: s.rates ? turn(s.q, s.rates, dt) : s.q,
  };
}

/** Between `a` and `b` at time `t` (a.t ≤ t ≤ b.t): the cubic Hermite through their positions
 * with their velocities, the attitude slerped, the rest blended. */
export function between<S extends Sample>(a: S, b: S, t: number): S {
  const span = b.t - a.t;
  if (span <= 0) return b;
  const k = (t - a.t) / span;
  const k2 = k * k;
  const k3 = k2 * k;
  const h10 = k3 - 2 * k2 + k;
  const h01 = -2 * k3 + 3 * k2;
  const h11 = k3 - k2;
  // In metres north, east, down from a.
  const cos = Math.cos(a.lat / DEG);
  const d: Vec3 = [((b.lat - a.lat) / DEG) * R, ((b.lon - a.lon) / DEG) * R * cos, a.alt - b.alt];
  const p = [0, 1, 2].map((i) => h10 * span * a.v[i]! + h01 * d[i]! + h11 * span * b.v[i]!);
  const near = k < 0.5 ? a : b;
  return {
    ...near,
    t,
    lat: a.lat + (p[0]! / R) * DEG,
    lon: a.lon + (p[1]! / (R * cos)) * DEG,
    alt: a.alt - p[2]!,
    q: slerp(a.q, b.q, k),
    v: lerp3(a.v, b.v, k),
    ...(a.rates && b.rates ? { rates: lerp3(a.rates, b.rates, k) } : {}),
    ...(a.act && b.act
      ? { act: a.act.map((x, i) => x + (b.act![i]! - x) * k) as [number, number, number, number] }
      : {}),
  };
}

const lerp3 = (a: Vec3, b: Vec3, k: number): Vec3 => [
  a[0] + (b[0] - a[0]) * k,
  a[1] + (b[1] - a[1]) * k,
  a[2] + (b[2] - a[2]) * k,
];

/** The unit quaternions' arc from `a` to `b`, `k` of the way, the short way round. */
export function slerp(a: Quat, b: Quat, k: number): Quat {
  let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  const sign = dot < 0 ? -1 : 1;
  dot *= sign;
  let ka: number;
  let kb: number;
  if (dot > 0.9995) {
    ka = 1 - k;
    kb = k;
  } else {
    const theta = Math.acos(dot);
    const s = Math.sin(theta);
    ka = Math.sin((1 - k) * theta) / s;
    kb = Math.sin(k * theta) / s;
  }
  const q: Quat = [
    ka * a[0] + kb * sign * b[0],
    ka * a[1] + kb * sign * b[1],
    ka * a[2] + kb * sign * b[2],
    ka * a[3] + kb * sign * b[3],
  ];
  const n = Math.hypot(...q) || 1;
  return [q[0] / n, q[1] / n, q[2] / n, q[3] / n];
}

/** `q` (body to NED) turned by body rates `w` (rad/s) for `dt` s: q ⊗ exp(w dt / 2). */
export function turn(q: Quat, w: Vec3, dt: number): Quat {
  const angle = Math.hypot(w[0], w[1], w[2]) * dt;
  if (angle < 1e-12) return q;
  const s = Math.sin(angle / 2) / (angle / dt);
  const d: Quat = [Math.cos(angle / 2), w[0] * s, w[1] * s, w[2] * s];
  return [
    q[0] * d[0] - q[1] * d[1] - q[2] * d[2] - q[3] * d[3],
    q[0] * d[1] + q[1] * d[0] + q[2] * d[3] - q[3] * d[2],
    q[0] * d[2] - q[1] * d[3] + q[2] * d[0] + q[3] * d[1],
    q[0] * d[3] + q[1] * d[2] - q[2] * d[1] + q[3] * d[0],
  ];
}
