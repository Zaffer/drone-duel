// The air the quad flies through, in the page: drone_duels.physics.core.weather, line for
// line (its docstring has the model and its sources: MIL-F-8785C's low-altitude wind shear and
// Dryden turbulence, and gusts as a weather report gives them). `step` advances it; then `wind`
// is the air's velocity at the aircraft, north, east, down, m/s.
//
// Left out: the turbulence's rotation (`rates` there), which the wing takes and the quad does
// not (ROADMAP.md). The random numbers are this file's own, so a seed gives another air than
// Python's does: the statistics are what match (web/test/sim-weather.test.ts).

import type { Location } from "../net/protocol.ts";
import type { Vec } from "./frames.ts";

export type Wind = Location["wind"];

const FOOT = 0.3048;
/** z0, m: MIL-F-8785C's for terminal flight phases. */
const GROUND_ROUGHNESS = 0.15 * FOOT;
const REFERENCE_HEIGHT = 20 * FOOT;
/** The turbulence is met at no less than this, m/s (EST, as Python's). */
const MIN_AIRSPEED = 2.0;
/** Each gust's rise and fall, s: from, to. */
const GUST_TIME: readonly [number, number] = [2.0, 5.0];
/** How far off the wind's direction a gust can blow, rad. */
const GUST_SPREAD = (30 * Math.PI) / 180;
/** The weakest gust, as a share of the strongest. */
const GUST_LEAST = 0.4;
/** A gust asked for (G) rises at least this share of the mean wind above it... */
const GUST_NOW_LEAST = 0.5;
/** ...and at least this much, m/s, so a light wind still gusts. */
const GUST_LEAST_MS = 2.0;

/** The game's wind settings for a key left out: still air, 8785C's turbulence. */
export const CALM: Wind = {
  speed_ms: 0,
  from_deg: 0,
  turbulence: 1,
  eddy_scale: 1,
  gust_factor: 1,
  gust_every_s: 10,
};

/** A seeded generator of uniform numbers in [0, 1) (mulberry32). */
export function seeded(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/** The strongest gust above the mean wind, m/s, for a gust factor (the strongest gust over the
 * mean wind, as a weather report's). */
export const gustPeak = (speed: number, gustFactor: number): number =>
  Math.max(0, gustFactor - 1) * speed;

export class Weather {
  speed: number;
  fromDeg: number;
  gusts: number;
  turbulence: number;
  scale: number;
  gustEvery: number;
  /** The air's velocity at the aircraft, NED, m/s. */
  wind: Vec = [0, 0, 0];
  /** The turbulence at a point, 8785C's, NED, m/s. */
  eddies: Vec = [0, 0, 0];
  /** The turbulence over the span, as the airframe feels it, NED, m/s. */
  felt: Vec = [0, 0, 0];
  gust: Vec = [0, 0, 0];
  private u = 0;
  private v: [number, number] = [0, 0];
  private w: [number, number] = [0, 0];
  private gustIn: number;
  /** The gust under way: for how long so far, how long in all, its peak (NED). */
  private under: { t: number; duration: number; peak: Vec } | null = null;
  /** A gust asked for now, m/s above the wind. */
  private gustNext: number | null = null;
  private spare: number | null = null;

  constructor(
    settings: Partial<Wind>,
    /** The turbulence's scale across the airframe, m. */
    readonly span: number,
    private readonly random: () => number = Math.random,
  ) {
    const w = { ...CALM, ...settings };
    this.speed = w.speed_ms;
    this.fromDeg = w.from_deg;
    this.gusts = gustPeak(w.speed_ms, w.gust_factor);
    this.turbulence = w.turbulence;
    this.scale = w.eddy_scale;
    this.gustEvery = w.gust_every_s;
    this.gustIn = this.exponential(this.gustEvery);
  }

  /** Change it live, keeping the turbulence's state; `gustNow` starts a gust at once. */
  update(settings: Partial<Wind>, gustNow = false): void {
    const w = { ...CALM, ...settings };
    this.speed = w.speed_ms;
    this.fromDeg = w.from_deg;
    this.turbulence = w.turbulence;
    this.scale = w.eddy_scale;
    this.gusts = gustPeak(w.speed_ms, w.gust_factor);
    if (w.gust_every_s !== this.gustEvery) {
      this.gustEvery = w.gust_every_s;
      this.gustIn = Math.min(this.gustIn, this.exponential(this.gustEvery));
    }
    if (gustNow) {
      this.under = null;
      this.gustNext = Math.max(this.gusts, GUST_NOW_LEAST * this.speed, GUST_LEAST_MS);
    }
  }

  /** Unit, NED: where the wind blows to. */
  get downwind(): Vec {
    const to = ((this.fromDeg + 180) * Math.PI) / 180;
    return [Math.cos(to), Math.sin(to), 0];
  }

  /** The mean wind at `height` m above the ground over the wind at 20 ft. */
  shear(height: number): number {
    if (height <= GROUND_ROUGHNESS) return 0;
    return Math.log(height / GROUND_ROUGHNESS) / Math.log(REFERENCE_HEIGHT / GROUND_ROUGHNESS);
  }

  /** On by `dt` s, the aircraft `height` m above the ground at `airspeed` m/s. */
  step(dt: number, height: number, airspeed: number): void {
    const along = this.downwind;
    const across: Vec = [-along[1], along[0], 0];
    const mean = this.speed * this.shear(height);
    const sigmaW = 0.1 * this.speed * this.turbulence;
    if (sigmaW > 0) {
      const h = Math.min(1000, Math.max(10, height / FOOT));
      const met = Math.max(airspeed, MIN_AIRSPEED);
      this.eddies = this.turbulent(dt, h, sigmaW, met, along, across);
      const e = Math.exp((-dt * Math.PI * met) / (4 * this.span));
      for (let i = 0; i < 3; i++) this.felt[i] = this.felt[i]! * e + this.eddies[i]! * (1 - e);
    } else {
      this.eddies = [0, 0, 0];
      this.felt = [0, 0, 0];
    }
    this.gust = this.gusting(dt, height);
    this.wind = [
      along[0] * mean + this.felt[0] + this.gust[0],
      along[1] * mean + this.felt[1] + this.gust[1],
      this.felt[2] + this.gust[2],
    ];
  }

  /** The turbulence's velocity, NED, `h` ft above the ground. */
  private turbulent(
    dt: number,
    h: number,
    sigmaW: number,
    airspeed: number,
    along: Vec,
    across: Vec,
  ): Vec {
    const k = 0.177 + 0.000823 * h;
    const sigmaU = sigmaW / k ** 0.4;
    const longScale = (h / k ** 1.2) * FOOT * this.scale;
    this.u = this.lag(this.u, longScale / airspeed, dt);
    this.v = this.lag2(this.v, longScale / airspeed, dt);
    this.w = this.lag2(this.w, (h * FOOT * this.scale) / airspeed, dt);
    const u = sigmaU * this.u;
    const v = sigmaU * shaped(this.v);
    const w = sigmaW * shaped(this.w);
    return [along[0] * u + across[0] * v, along[1] * u + across[1] * v, w];
  }

  /** A unit-variance first-order (Gauss-Markov) process of time constant `tau`, s. */
  private lag(z: number, tau: number, dt: number): number {
    const e = Math.exp(-dt / tau);
    return e * z + Math.sqrt(1 - e * e) * this.gauss();
  }

  /** `lag`, and that again through a first-order lag: Dryden's second-order spectrum's
   * states (`shaped`). */
  private lag2(z: [number, number], tau: number, dt: number): [number, number] {
    const first = this.lag(z[0], tau, dt);
    return [first, z[1] + (1 - Math.exp(-dt / tau)) * (first - z[1])];
  }

  private gusting(dt: number, height: number): Vec {
    if (this.gusts <= 0 && this.under === null && this.gustNext === null) return [0, 0, 0];
    if (this.under === null) {
      this.gustIn -= dt;
      if (this.gustIn > 0 && this.gustNext === null) return [0, 0, 0];
      let heading: number;
      if (this.speed > 0) {
        heading = ((this.fromDeg + 180) * Math.PI) / 180 + this.uniform(-GUST_SPREAD, GUST_SPREAD);
      } else {
        heading = this.uniform(-Math.PI, Math.PI);
      }
      let peak: number;
      if (this.gustNext !== null) {
        peak = this.gustNext;
        this.gustNext = null;
      } else {
        peak = this.gusts * this.uniform(GUST_LEAST, 1);
      }
      this.under = {
        t: 0,
        duration: this.uniform(GUST_TIME[0], GUST_TIME[1]),
        peak: [peak * Math.cos(heading), peak * Math.sin(heading), 0],
      };
      this.gustIn = this.exponential(this.gustEvery);
    }
    const g = this.under;
    g.t += dt;
    if (g.t >= g.duration) {
      this.under = null;
      return [0, 0, 0];
    }
    const k = 0.5 * (1 - Math.cos((2 * Math.PI * g.t) / g.duration)) * this.shear(height);
    return [g.peak[0] * k, g.peak[1] * k, g.peak[2] * k];
  }

  private uniform(a: number, b: number): number {
    return a + (b - a) * this.random();
  }

  private exponential(mean: number): number {
    return -Math.log(1 - this.random()) * mean;
  }

  /** A unit normal draw (Box-Muller, both of each pair used). */
  private gauss(): number {
    if (this.spare !== null) {
      const z = this.spare;
      this.spare = null;
      return z;
    }
    const r = Math.sqrt(-2 * Math.log(1 - this.random()));
    const a = 2 * Math.PI * this.random();
    this.spare = r * Math.sin(a);
    return r * Math.cos(a);
  }
}

/** Dryden's second-order spectrum, (1 + √3 T s) / (1 + T s)², at unit variance, from `lag2`'s
 * states. */
function shaped(z: [number, number]): number {
  return (z[1] + Math.sqrt(3) * (z[0] - z[1])) / Math.SQRT2;
}
