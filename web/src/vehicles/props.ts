// Propellers as they are drawn: a real rotor turns far faster than a frame can show (a 5" prop
// at 2000 rad/s turns five times a frame at 60 fps), so drawn at its real rate it would strobe,
// standing still or turning backwards. The drawn spin follows the rotor's speed instead, eased
// towards a ceiling slow enough to read as turning forward, and never more than a fraction of
// the blades' spacing in one frame, however slow the frame. As the old COP (fleet.js) and wing
// page (wing.js) did, it says "stopped, idling, flying" rather than the true rpm.
//
// Pure maths, no Cesium: quad.ts and wing.ts move the model's nodes with it.

export interface SpinLook {
  /** The drawn rate the spin eases towards at full speed, rad/s. */
  maxRadS: number;
  /** The rotor speed, rad/s, at which the drawn rate is 63 % of `maxRadS`. */
  refRadS: number;
  /** Seconds for the drawn rate to settle at a new rotor speed. */
  settleS: number;
  /** The most the prop turns in one frame, rad: under half the angle between its blades, so it
   * never reads as turning backwards. */
  maxStepRad: number;
}

/** The 5" quad's tri-blades (fleet.js turned them at most once a second; this follows the
 * rotors): 3 turns/s flat out, about 1.2 at idle (≈300 rad/s), 2.6 in a hover (≈1500 rad/s). */
export const QUAD_SPIN: SpinLook = {
  maxRadS: 3 * 2 * Math.PI,
  refRadS: 800,
  settleS: 0.2,
  maxStepRad: Math.PI / 6,
};

/** The Chupito's two-blade prop: wing.js's 30 rad/s at full throttle (≈4500 rad/s). */
export const WING_SPIN: SpinLook = {
  maxRadS: 30,
  refRadS: 1500,
  settleS: 0.3,
  maxStepRad: Math.PI / 4,
};

/** The drawn spin rate, rad/s, for a rotor turning at `omega` rad/s (either way round). */
export function drawnRate(omega: number, look: SpinLook): number {
  const w = Math.abs(omega);
  if (!Number.isFinite(w)) return 0;
  return look.maxRadS * (1 - Math.exp(-w / look.refRadS));
}

/** One propeller's drawn angle, advanced a frame at a time. */
export class PropSpin {
  /** Radians, 0..2π. */
  angle = 0;
  /** The drawn rate now, rad/s. */
  rate = 0;

  constructor(readonly look: SpinLook) {}

  /** Advance by `dt` seconds with the rotor at `omega` rad/s; the new angle. */
  step(omega: number, dt: number): number {
    const d = Math.min(Math.max(dt, 0), 0.5);
    const target = drawnRate(omega, this.look);
    this.rate += (target - this.rate) * (1 - Math.exp(-d / this.look.settleS));
    const turn = Math.min(this.rate * d, this.look.maxStepRad);
    this.angle = (this.angle + turn) % (2 * Math.PI);
    return this.angle;
  }
}
