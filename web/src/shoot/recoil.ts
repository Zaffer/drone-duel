// Recoil (DESIGN.md §7, Shoot Down): the shot's momentum, and what the shooter's body does with
// it. The bullet and the powder gas leave forward, so the rifle takes the same momentum back:
// J = m_bullet v_muzzle + m_powder v_gas, v_gas ≈ 1.75 v_muzzle (Hatcher's factor for rifle gas,
// as SAAMI's free-recoil sums use). For M193 from this rifle that is 5.5 N·s: free, its 3.2 kg
// would come back at 1.7 m/s with 4.8 J.
//
// The shoulder stops it, and the body moves. Three springy, damped motions carry the kick, each
// set going by its share of the impulse:
// - back: the rifle into the shoulder pocket (soft tissue, stiff) and the shoulder back with the
//   upper body (heavy, held by the stance): two masses on two springs;
// - up: the upper body tipping back about the hips (standing, kneeling) or the elbows (prone),
//   the push at the shoulder's height above that pivot; and the rifle itself tipping about the
//   shoulder, the bore a little above where the butt meets it;
// - sideways: the push lands right of the spine (a right-handed shooter), so the shoulders turn
//   and the muzzle swings right, more or less as the stance squares up to the shot.
// The muscles bring each back to where the shooter was holding, near enough: a little error is
// left each shot. Masses, levers and stiffnesses are EST (anthropometric round numbers; tuned
// only so the times to settle are a shooter's, a few tenths of a second).

export type Stance = "standing" | "kneeling" | "prone";

/** Hatcher's: rifle powder gas leaves at about this times the bullet's speed. */
export const GAS_FACTOR = 1.75;

/** The rifle's momentum back, N·s. */
export function recoilImpulse(c: { bulletKg: number; powderKg: number; muzzleMs: number }): number {
  return c.bulletKg * c.muzzleMs + c.powderKg * c.muzzleMs * GAS_FACTOR;
}

/** A body's part in taking the recoil, by stance (EST). */
interface Body {
  /** The upper body's mass behind the shoulder, kg, and how firmly the stance holds it back, Hz. */
  torsoKg: number;
  torsoHz: number;
  /** Its moment of inertia about the pivot it tips on, kg·m², and the shoulder's height above
   * that pivot, m. */
  pitchI: number;
  pitchLever: number;
  pitchHz: number;
  /** Turning: the shoulders' moment of inertia (arms and rifle out), kg·m², the push's offset
   * right of the spine, m (± a spread, shot to shot), and how stiffly the stance resists. */
  yawI: number;
  yawLever: number;
  yawSpread: number;
  yawHz: number;
  /** What's left off the aim after each shot, rad (one standard deviation). */
  residual: number;
}

export const BODIES: Record<Stance, Body> = {
  standing: {
    torsoKg: 18,
    torsoHz: 2.2,
    pitchI: 6,
    pitchLever: 0.45,
    pitchHz: 2.6,
    yawI: 3,
    yawLever: 0.05,
    yawSpread: 0.03,
    yawHz: 2.4,
    residual: 0.0012,
  },
  kneeling: {
    torsoKg: 22,
    torsoHz: 2.8,
    pitchI: 5,
    pitchLever: 0.35,
    pitchHz: 3.2,
    yawI: 3,
    yawLever: 0.04,
    yawSpread: 0.025,
    yawHz: 3,
    residual: 0.0007,
  },
  prone: {
    torsoKg: 35,
    torsoHz: 4,
    pitchI: 2.5,
    pitchLever: 0.12,
    pitchHz: 4.5,
    yawI: 4,
    yawLever: 0.02,
    yawSpread: 0.015,
    yawHz: 4,
    residual: 0.0003,
  },
};

/** The shoulder pocket: soft tissue over bone, Hz with the rifle's mass on it, and its damping
 * ratio (EST: it gives a centimetre or so under the kick). */
const POCKET_HZ = 14;
const POCKET_ZETA = 0.5;
/** The rifle tipping about the shoulder under the hands: the bore sits this far above where the
 * butt bears (an AR's straight stock: EST), its moment about the shoulder (EST), Hz. */
const BORE_ABOVE_BUTT = 0.025;
const RIFLE_I = 0.55;
const RIFLE_HZ = 6;
/** The muscles' damping ratio bringing each motion back (EST: a little under critical). */
const ZETA = 0.75;

/** How long a shot's leftover aim error takes to fade, s (EST). */
const RESIDUAL_FADE_S = 3;

/** A damped spring's state: where, how fast. */
class Mode {
  x = 0;
  v = 0;
  target = 0;
  constructor(
    public hz: number,
    public zeta: number,
  ) {}
  step(dt: number, force = 0): void {
    const w = 2 * Math.PI * this.hz;
    const a = -w * w * (this.x - this.target) - 2 * this.zeta * w * this.v + force;
    this.v += a * dt;
    this.x += this.v * dt;
  }
}

function gauss(rand: () => number): number {
  const u = 1 - rand();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * rand());
}

export class Recoil {
  /** The rifle back from where it was held (into the shoulder and with it), m. */
  private rifleBack = new Mode(POCKET_HZ, POCKET_ZETA);
  private torsoBack = new Mode(BODIES.standing.torsoHz, ZETA);
  private torsoPitch = new Mode(BODIES.standing.pitchHz, ZETA);
  private riflePitch = new Mode(RIFLE_HZ, ZETA);
  private yaw = new Mode(BODIES.standing.yawHz, ZETA);
  private body: Body = BODIES.standing;

  constructor(
    readonly rifleKg: number,
    private readonly rand: () => number = Math.random,
  ) {}

  set stance(s: Stance) {
    this.body = BODIES[s];
    this.torsoBack.hz = this.body.torsoHz;
    this.torsoPitch.hz = this.body.pitchHz;
    this.yaw.hz = this.body.yawHz;
  }

  /** A shot: `impulse` N·s back along the bore. */
  kick(impulse: number): void {
    const b = this.body;
    // The rifle takes it all at once; the shoulder pocket hands it on to the torso as it gives.
    this.rifleBack.v += impulse / this.rifleKg;
    this.riflePitch.v += (impulse * BORE_ABOVE_BUTT) / RIFLE_I;
    // What the torso takes, it takes over the pocket's few milliseconds: as an impulse here.
    this.torsoPitch.v += (impulse * b.pitchLever) / b.pitchI;
    const lever = b.yawLever + b.yawSpread * gauss(this.rand);
    this.yaw.v += (impulse * lever) / b.yawI;
    this.torsoBack.v += impulse / (b.torsoKg + this.rifleKg);
    // Where each comes back to: near, not exactly, where it was.
    this.torsoPitch.target += b.residual * gauss(this.rand);
    this.yaw.target += b.residual * gauss(this.rand);
  }

  step(dt: number): void {
    // What's left off the aim fades as the shooter finds their aim again (EST: a few seconds).
    const fade = Math.exp(-dt / RESIDUAL_FADE_S);
    this.torsoPitch.target *= fade;
    this.yaw.target *= fade;
    // Small steps: the pocket rings at 14 Hz.
    const n = Math.max(1, Math.ceil(dt / 0.002));
    const h = dt / n;
    for (let i = 0; i < n; i++) {
      this.rifleBack.step(h);
      this.torsoBack.step(h);
      this.torsoPitch.step(h);
      this.riflePitch.step(h);
      this.yaw.step(h);
    }
  }

  /** The aim's offset now: up and right, rad. */
  get pitch(): number {
    return this.torsoPitch.x + this.riflePitch.x;
  }
  get yawRight(): number {
    return this.yaw.x;
  }
  /** The head goes with the torso; the rifle with the torso and into the pocket, m. */
  get headBack(): number {
    return this.torsoBack.x;
  }
  get rifleBackM(): number {
    return this.torsoBack.x + this.rifleBack.x;
  }
}
