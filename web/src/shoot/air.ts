// The air the shooter's bullets fly through, and the wind meter reads (DESIGN.md §7, Shoot Down):
// physics/core/weather.py's model of the place's wind, made again in the page. The mean wind at
// 20 ft falls off toward the ground on MIL-F-8785C's log profile; its turbulence (σ_w 0.1 W20,
// σ_u = σ_w / (0.177 + 0.000823 h)^0.4) is carried past by the mean wind, so a point on the
// ground feels it change with time L_u / W; and gusts in 8785C's 1 − cos shape blow up to
// `gust_factor` × the mean now and then. The drone's physics draws its own gusts from the same
// settings: the same weather, not the same moment of it.
//
// One value fills the air at each height at a time: over the half second a bullet flies, the
// eddies the wind carries along a few hundred metres of its path are the same few seconds of
// turbulence (EST: the spatial variation across a shot is left out).

import type { Location } from "../net/protocol.ts";
import type { Vec3 } from "../world/geo.ts";

type Wind = Location["wind"];

const FOOT = 0.3048;
const Z0 = 0.15 * FOOT;
const W20_HEIGHT = 20 * FOOT;
/** The turbulence's scale at this height, ft: where the bullets fly over open ground (EST). */
const EDDY_HEIGHT_FT = 30;
/** As weather.py: each gust's rise and fall, s; how far off the wind; the weakest. */
const GUST_TIME: [number, number] = [2, 5];
const GUST_SPREAD = (30 * Math.PI) / 180;
const GUST_LEAST = 0.4;

/** The mean wind at `height` m over the wind at 20 ft (MIL-F-8785C's log profile). */
export function shear(height: number): number {
  return height <= Z0 ? 0 : Math.log(height / Z0) / Math.log(W20_HEIGHT / Z0);
}

function gauss(rand: () => number): number {
  const u = 1 - rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rand());
}

export class Air {
  /** Turbulence, unit variance: along the wind, across it. */
  private u = 0;
  private v = 0;
  private gust: { t: number; dur: number; e: number; n: number } | null = null;
  private gustIn: number;
  /** Seconds since it began. */
  t = 0;

  constructor(
    public wind: Wind,
    private readonly rand: () => number = Math.random,
  ) {
    this.gustIn = this.nextGust();
  }

  private nextGust(): number {
    return -Math.log(1 - this.rand()) * this.wind.gust_every_s;
  }

  /** On by `dt` s. */
  step(dt: number): void {
    this.t += dt;
    const w = this.wind;
    const h = EDDY_HEIGHT_FT;
    const k = 0.177 + 0.000823 * h;
    const scale = (h / k ** 1.2) * FOOT * w.eddy_scale;
    const tau = scale / Math.max(1, w.speed_ms);
    const e = Math.exp(-dt / tau);
    const s = Math.sqrt(1 - e * e);
    this.u = e * this.u + s * gauss(this.rand);
    this.v = e * this.v + s * gauss(this.rand);
    const peak = Math.max(0, w.gust_factor - 1) * w.speed_ms;
    if (this.gust) {
      this.gust.t += dt;
      if (this.gust.t >= this.gust.dur) this.gust = null;
    } else if (peak > 0) {
      this.gustIn -= dt;
      if (this.gustIn <= 0) {
        const to = ((w.from_deg + 180) * Math.PI) / 180 + (this.rand() * 2 - 1) * GUST_SPREAD;
        const size = peak * (GUST_LEAST + (1 - GUST_LEAST) * this.rand());
        const dur = GUST_TIME[0] + (GUST_TIME[1] - GUST_TIME[0]) * this.rand();
        this.gust = { t: 0, dur, e: size * Math.sin(to), n: size * Math.cos(to) };
        this.gustIn = this.nextGust();
      }
    }
  }

  /** The air's velocity, m/s east-north-up, `height` m above the ground. */
  at(height: number): Vec3 {
    const w = this.wind;
    const to = ((w.from_deg + 180) * Math.PI) / 180;
    const along: [number, number] = [Math.sin(to), Math.cos(to)];
    const sh = shear(height);
    const hFt = Math.min(1000, Math.max(10, height / FOOT));
    const sigmaW = 0.1 * w.speed_ms * w.turbulence;
    const sigmaU = sigmaW / (0.177 + 0.000823 * hFt) ** 0.4;
    const mean = w.speed_ms * sh;
    const u = mean + sigmaU * this.u;
    const v = sigmaU * this.v;
    // v across the wind, to its right (weather.py's).
    let e = along[0] * u + along[1] * v;
    let n = along[1] * u - along[0] * v;
    if (this.gust) {
      const k = 0.5 * (1 - Math.cos((2 * Math.PI * this.gust.t) / this.gust.dur)) * sh;
      e += this.gust.e * k;
      n += this.gust.n * k;
    }
    return [e, n, 0];
  }

  /** What a wind meter held at `height` m reads: speed m/s, and where from, degrees true. */
  meter(height = 1.5): { speed: number; fromDeg: number } {
    const [e, n] = this.at(height);
    const speed = Math.hypot(e, n);
    const from = ((Math.atan2(-e, -n) * 180) / Math.PI + 360) % 360;
    return { speed, fromDeg: from };
  }
}
