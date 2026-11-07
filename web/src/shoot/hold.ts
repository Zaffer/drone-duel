// Holding the aim (DESIGN.md §7, Shoot Down): why the sight never sits still. Four things move
// it, each by stance, and all of them grow with the shooter's heart rate:
// - breathing lifts and drops the muzzle, slowly; holding the breath (a key) stills that for a
//   few seconds, after which the shakes grow until they must breathe again;
// - the hold wobbles: the body's sway and the arms' tremor, a slow wander and a faster shiver;
// - each heartbeat nudges it;
// - the trigger: breaking a shot jerks it a little, low and left for a right-handed shooter, more
//   when the shots come fast.
// Walking and running raise the heart rate; standing still lets it fall again.
//
// Sizes are EST, from the hold areas shooting coaches quote for a decent shooter (a wobble of
// some 4–8 MOA standing, 2–4 kneeling, under 1 prone, unsupported, at rest), and breathing and
// heart rates from physiology's round numbers.

import { Spring } from "../engine/spring.ts";
import type { Stance } from "./recoil.ts";

/** rad per MOA. */
const MOA = Math.PI / (180 * 60);

interface HoldSpec {
  /** The wobble's spread, one standard deviation each way, rad: the slow wander and the shiver. */
  wander: number;
  shiver: number;
  /** Breathing's swing up and down, rad (half the peak to peak). */
  breath: number;
  /** A heartbeat's nudge, rad. */
  pulse: number;
}

export const HOLDS: Record<Stance, HoldSpec> = {
  standing: { wander: 2.2 * MOA, shiver: 0.8 * MOA, breath: 5 * MOA, pulse: 0.25 * MOA },
  kneeling: { wander: 1.2 * MOA, shiver: 0.5 * MOA, breath: 3 * MOA, pulse: 0.3 * MOA },
  prone: { wander: 0.35 * MOA, shiver: 0.2 * MOA, breath: 2 * MOA, pulse: 0.4 * MOA },
};

/** Heart rate, beats a minute: at rest, walking, running; how fast it climbs and falls, s. */
const HR_REST = 72;
const HR_WALK = 100;
const HR_RUN = 150;
const HR_RISE_S = 12;
const HR_FALL_S = 35;
/** Breaths a minute at rest; and how they quicken with the heart. */
const BREATHS_REST = 13;
/** Holding the breath: still for this long, then the shakes grow until this, when they must
 * breathe; afterwards they need this long before they can hold it well again (EST). */
const HOLD_STILL_S = 6;
const HOLD_MAX_S = 14;
const HOLD_RECOVER_S = 6;
/** The wander's and the shiver's time scales, s. */
const WANDER_S = 1.2;
const SHIVER_S = 0.12;
/** The trigger's jerk at a slow, careful press, and what fast shooting adds, rad; its way,
 * degrees on from straight down toward the left (EST). */
const JERK = 0.4 * MOA;
const JERK_FAST = 2.5 * MOA;
const JERK_FAST_S = 0.35;
const JERK_WAY_DEG = 35;

function gauss(rand: () => number): number {
  const u = 1 - rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rand());
}

/** A unit-variance first-order random process. */
class Wander {
  x = 0;
  constructor(private readonly tau: number) {}
  step(dt: number, rand: () => number): number {
    const e = Math.exp(-dt / this.tau);
    this.x = e * this.x + Math.sqrt(1 - e * e) * gauss(rand);
    return this.x;
  }
}

export type Effort = "still" | "walk" | "run";

export class Hold {
  stance: Stance = "standing";
  /** Beats a minute. */
  heart = HR_REST;
  /** Seconds the breath has been held, or 0. */
  held = 0;
  /** Breathing's phase, 0..1 a breath. */
  private phase = 0.3;
  private breathAmp = 1;
  private beat = 0;
  /** The heart's nudge, rung down quickly. */
  private pulse = new Spring(4, 0.6);
  private recover = 0;
  private wx = new Wander(WANDER_S);
  private wy = new Wander(WANDER_S);
  private sx = new Wander(SHIVER_S);
  private sy = new Wander(SHIVER_S);
  /** The trigger's jerk, decaying, rad: right, up. */
  private jerk: [number, number] = [0, 0];
  private lastShot = -1e9;
  private t = 0;
  /** The aim's offset now, rad: right, up. */
  yaw = 0;
  pitch = 0;

  constructor(private readonly rand: () => number = Math.random) {}

  /** Whether holding the breath can start now (not straight after a forced breath). */
  get canHold(): boolean {
    return this.recover <= 0;
  }

  /** On by `dt` s: moving with `effort`, the breath held or not. */
  step(dt: number, effort: Effort, holding: boolean): void {
    this.t += dt;
    const target = effort === "run" ? HR_RUN : effort === "walk" ? HR_WALK : HR_REST;
    const tau = target > this.heart ? HR_RISE_S : HR_FALL_S;
    this.heart += (target - this.heart) * (1 - Math.exp(-dt / tau));
    const effortK = 1 + (this.heart - HR_REST) / 60;
    const spec = HOLDS[this.stance];

    // Breathing: a breath held stops it (after the breath out, as shooters do: the muzzle is at
    // the bottom of its swing), until the body forces the next.
    this.recover = Math.max(0, this.recover - dt);
    const wantsHold = holding && effort === "still" && (this.held > 0 || this.canHold);
    if (wantsHold && this.held < HOLD_MAX_S) {
      this.held += dt;
      this.breathAmp += (0 - this.breathAmp) * (1 - Math.exp(-dt / 0.4));
    } else {
      if (this.held > 0) this.recover = this.held >= HOLD_MAX_S ? HOLD_RECOVER_S : 1;
      this.held = 0;
      this.breathAmp += (1 - this.breathAmp) * (1 - Math.exp(-dt / 0.8));
      const rate = (BREATHS_REST * (this.heart / HR_REST)) / 60;
      this.phase = (this.phase + rate * dt) % 1;
    }
    // A breath: in over 40 %, out over 35 %, a pause at the bottom.
    const p = this.phase;
    const lung =
      p < 0.4
        ? 0.5 - 0.5 * Math.cos((Math.PI * p) / 0.4)
        : p < 0.75
          ? 0.5 + 0.5 * Math.cos((Math.PI * (p - 0.4)) / 0.35)
          : 0;
    const breath = spec.breath * effortK * this.breathAmp * (lung * 2 - 1);

    // Holding too long: the shakes.
    const strain =
      this.held > HOLD_STILL_S
        ? 1 + (2 * (this.held - HOLD_STILL_S)) / (HOLD_MAX_S - HOLD_STILL_S)
        : 1;
    const calm = this.held > 0 && this.held <= HOLD_STILL_S ? 0.7 : 1;
    const k = effortK * strain * calm;
    const wobbleX =
      spec.wander * k * this.wx.step(dt, this.rand) + spec.shiver * k * this.sx.step(dt, this.rand);
    const wobbleY =
      spec.wander * k * this.wy.step(dt, this.rand) + spec.shiver * k * this.sy.step(dt, this.rand);

    // The heart: a nudge each beat, rung down quickly.
    this.beat += (this.heart / 60) * dt;
    if (this.beat >= 1) {
      this.beat -= 1;
      this.pulse.v += spec.pulse * 40 * (this.heart / HR_REST);
    }
    this.pulse.step(dt);

    // The trigger's jerk fades over a few tenths.
    const jf = Math.exp(-dt / 0.15);
    this.jerk = [this.jerk[0] * jf, this.jerk[1] * jf];

    this.yaw = wobbleX + this.jerk[0];
    this.pitch = wobbleY + breath + this.pulse.x + this.jerk[1];
  }

  /** The shot breaks: jerk the aim (before the bullet leaves). */
  pull(): void {
    const since = this.t - this.lastShot;
    this.lastShot = this.t;
    const size = (JERK + JERK_FAST * Math.exp(-since / JERK_FAST_S)) * (0.5 + this.rand());
    // Its way, clockwise from straight up: low left is between 180° and 270°.
    const way = ((180 + JERK_WAY_DEG + (this.rand() * 2 - 1) * 40) * Math.PI) / 180;
    this.jerk[0] += size * Math.sin(way);
    this.jerk[1] += size * Math.cos(way);
  }
}
