// A bullet's flight (DESIGN.md §7, Shoot Down): a point mass under gravity, drag against the
// standard G7 projectile scaled by the load's ballistic coefficient, through the moving air,
// on the turning Earth; with the two effects of its spin a point mass leaves out, spin drift
// and aerodynamic jump, from Litz's fits to measured flights. Everything here is in a local
// east-north-up frame, metres, seconds, with z up from the shooter's ground (sea level + the
// sim's ground there, so the air's density is the place's).
//
// Sources:
// - G7: the US Army BRL standard drag function (Cd against Mach), as every point-mass solver
//   tabulates it; here as py-ballisticcalc 3.1's TableG7.
// - The atmosphere: ICAO's standard atmosphere (ISO 2533) below 11 km.
// - Gravity: the WGS84 Somigliana formula with the free-air gradient.
// - Spin drift: Litz, Applied Ballistics for Long Range Shooting (3rd ed.), §10:
//   SD (in) = 1.25 (Sg + 1.2) t^1.83, to the right for a right-hand twist.
// - Aerodynamic jump: Litz, ibid. §10: Y (MOA per mph of crosswind) = 0.01 Sg − 0.0024 L
//   + 0.032 (L the bullet's length in calibres); a wind from the left lifts the bullet of a
//   right-hand twist, from the right drops it.
// - Gyroscopic stability: Miller's twist rule (Precision Shooting, 2005), with its velocity
//   and atmosphere corrections.

import type { Vec3 } from "../world/geo.ts";

/** Mach, Cd: the G7 standard projectile's drag coefficient. */
const G7: readonly (readonly [number, number])[] = [
  [0.0, 0.1198],
  [0.05, 0.1197],
  [0.1, 0.1196],
  [0.15, 0.1194],
  [0.2, 0.1193],
  [0.25, 0.1194],
  [0.3, 0.1194],
  [0.35, 0.1194],
  [0.4, 0.1193],
  [0.45, 0.1193],
  [0.5, 0.1194],
  [0.55, 0.1193],
  [0.6, 0.1194],
  [0.65, 0.1197],
  [0.7, 0.1202],
  [0.725, 0.1207],
  [0.75, 0.1215],
  [0.775, 0.1226],
  [0.8, 0.1242],
  [0.825, 0.1266],
  [0.85, 0.1306],
  [0.875, 0.1368],
  [0.9, 0.1464],
  [0.925, 0.166],
  [0.95, 0.2054],
  [0.975, 0.2993],
  [1.0, 0.3803],
  [1.025, 0.4015],
  [1.05, 0.4043],
  [1.075, 0.4034],
  [1.1, 0.4014],
  [1.125, 0.3987],
  [1.15, 0.3955],
  [1.2, 0.3884],
  [1.25, 0.381],
  [1.3, 0.3732],
  [1.35, 0.3657],
  [1.4, 0.358],
  [1.5, 0.344],
  [1.55, 0.3376],
  [1.6, 0.3315],
  [1.65, 0.326],
  [1.7, 0.3209],
  [1.75, 0.316],
  [1.8, 0.3117],
  [1.85, 0.3078],
  [1.9, 0.3042],
  [1.95, 0.301],
  [2.0, 0.298],
  [2.05, 0.2951],
  [2.1, 0.2922],
  [2.15, 0.2892],
  [2.2, 0.2864],
  [2.25, 0.2835],
  [2.3, 0.2807],
  [2.35, 0.2779],
  [2.4, 0.2752],
  [2.45, 0.2725],
  [2.5, 0.2697],
  [2.55, 0.267],
  [2.6, 0.2643],
  [2.65, 0.2615],
  [2.7, 0.2588],
  [2.75, 0.2561],
  [2.8, 0.2533],
  [2.85, 0.2506],
  [2.9, 0.2479],
  [2.95, 0.2451],
  [3.0, 0.2424],
  [3.1, 0.2368],
  [3.2, 0.2313],
  [3.3, 0.2258],
  [3.4, 0.2205],
  [3.5, 0.2154],
  [3.6, 0.2106],
  [3.7, 0.206],
  [3.8, 0.2017],
  [3.9, 0.1975],
  [4.0, 0.1935],
  [4.2, 0.1861],
  [4.4, 0.1793],
  [4.6, 0.173],
  [4.8, 0.1672],
  [5.0, 0.1618],
];

/** lb/in² in kg/m²: a ballistic coefficient's units. */
const LB_IN2 = 0.45359237 / 0.0254 ** 2;
const INCH = 0.0254;
const GRAIN = 6.479891e-5;
/** The Earth's turn, rad/s (WGS84). */
const OMEGA_EARTH = 7.292115e-5;

/** The G7 drag coefficient at `mach`, linear between the table's points. */
export function cdG7(mach: number): number {
  if (mach <= G7[0]![0]) return G7[0]![1];
  for (let i = 1; i < G7.length; i++) {
    const [m1, c1] = G7[i]!;
    if (mach <= m1) {
      const [m0, c0] = G7[i - 1]!;
      return c0 + ((c1 - c0) * (mach - m0)) / (m1 - m0);
    }
  }
  return G7[G7.length - 1]![1];
}

export interface Air {
  /** kg/m³ */
  density: number;
  /** The speed of sound, m/s. */
  sound: number;
  kelvin: number;
  pascal: number;
}

/** ICAO's standard atmosphere at `alt` m above sea level, `dT` K warmer than standard. */
export function airAt(alt: number, dT = 0): Air {
  const T0 = 288.15;
  const L = 0.0065;
  const R = 287.05287;
  const g0 = 9.80665;
  const tStd = T0 - L * alt;
  const pascal = 101325 * (tStd / T0) ** (g0 / (R * L));
  const kelvin = tStd + dT;
  return { density: pascal / (R * kelvin), sound: Math.sqrt(1.4 * R * kelvin), kelvin, pascal };
}

/** Gravity at `latDeg` and `alt` m above sea level, m/s² (WGS84's normal gravity). */
export function gravity(latDeg: number, alt: number): number {
  const s2 = Math.sin((latDeg * Math.PI) / 180) ** 2;
  const g = (9.7803253359 * (1 + 0.00193185265241 * s2)) / Math.sqrt(1 - 0.00669437999013 * s2);
  return g - 3.086e-6 * alt;
}

/** A cartridge as the ballistics needs it (from content's Cartridge and Weapon). */
export interface Load {
  massKg: number;
  diameterM: number;
  lengthM: number;
  bcG7: number;
  muzzleMs: number;
  /** The rifling's turn, m per turn (right hand). */
  twistM: number;
}

export function loadOf(w: {
  twist_in: number;
  cartridge: {
    bullet_g: number;
    diameter_mm: number;
    length_mm: number;
    bc_g7: number;
    muzzle_ms: number;
  };
}): Load {
  const c = w.cartridge;
  return {
    massKg: c.bullet_g / 1000,
    diameterM: c.diameter_mm / 1000,
    lengthM: c.length_mm / 1000,
    bcG7: c.bc_g7,
    muzzleMs: c.muzzle_ms,
    twistM: w.twist_in * INCH,
  };
}

/** Drag's deceleration over the square of the airspeed, 1/m, at `mach`: ½ ρ Cd A / m, with
 * the load's Cd A / m = (π/4) Cd_G7 / BC. */
function dragFactor(load: Load, air: Air, mach: number): number {
  return (Math.PI / 8) * air.density * (cdG7(mach) / (load.bcG7 * LB_IN2));
}

/** Gyroscopic stability (Miller), at `speed` m/s in `air`. */
export function stability(load: Load, speed: number, air: Air): number {
  const d = load.diameterM / INCH;
  const l = load.lengthM / load.diameterM;
  const t = load.twistM / load.diameterM;
  const m = load.massKg / GRAIN;
  const sg = (30 * m) / (t * t * d ** 3 * l * (1 + l * l));
  const fps = speed / 0.3048;
  const fahrenheit = ((air.kelvin - 273.15) * 9) / 5 + 32;
  const inHg = air.pascal / 3386.389;
  return sg * (fps / 2800) ** (1 / 3) * ((fahrenheit + 460) / 519) * (29.92 / inHg);
}

/** Spin drift after `t` s of flight, m to the right of the line of fire (Litz). */
export function spinDrift(sg: number, t: number): number {
  return 1.25 * (sg + 1.2) * t ** 1.83 * INCH;
}

/** Aerodynamic jump, rad up, from a crosswind of `fromLeft` m/s (blowing left to right)
 * (Litz). */
export function aeroJump(load: Load, sg: number, fromLeft: number): number {
  const l = load.lengthM / load.diameterM;
  const moaPerMph = 0.01 * sg - 0.0024 * l + 0.032;
  const moa = moaPerMph * (fromLeft / 0.44704);
  return (moa / 60) * (Math.PI / 180);
}

/** The air moving, m/s east-north-up, at a point (height `z` above the ground) and time. */
export type WindAt = (p: Vec3, t: number) => Vec3;
export const still: WindAt = () => [0, 0, 0];

/** Where the flight happens: the place's air, gravity and latitude. */
export interface Setting {
  air: Air;
  g: number;
  latDeg: number;
  wind: WindAt;
}

export function setting(latDeg: number, groundAlt: number, wind: WindAt = still): Setting {
  return { air: airAt(groundAlt), g: gravity(latDeg, groundAlt), latDeg, wind };
}

/** Steps of this length, s: well under a millimetre of error over a kilometre (the tests). */
export const STEP_S = 0.0005;

/** A bullet in flight. `p` and `v` are the point mass's; `at()` adds its spin drift. */
export class Round {
  p: Vec3;
  v: Vec3;
  t = 0;
  /** Gyroscopic stability at the muzzle (it grows as the bullet slows; the drift fit uses the
   * muzzle's). */
  readonly sg: number;
  /** Its right, horizontal, at the muzzle: spin drift's way. */
  private readonly right: Vec3;
  private readonly omega: Vec3;

  constructor(
    readonly load: Load,
    readonly where: Setting,
    muzzle: Vec3,
    dir: Vec3,
    speed = load.muzzleMs,
  ) {
    const [dx, dy, dz] = norm(dir);
    this.sg = stability(load, speed, where.air);
    const h = Math.hypot(dx, dy) || 1;
    this.right = [dy / h, -dx / h, 0];
    // Aerodynamic jump: the crosswind at the muzzle tips the line of departure up or down.
    const w = where.wind(muzzle, 0);
    const fromLeft = w[0] * this.right[0] + w[1] * this.right[1];
    const jump = aeroJump(load, this.sg, fromLeft);
    const up: Vec3 = cross(this.right, [dx, dy, dz]);
    const d = norm([dx + up[0] * jump, dy + up[1] * jump, dz + up[2] * jump]);
    this.p = [...muzzle];
    this.v = [d[0] * speed, d[1] * speed, d[2] * speed];
    const lat = (where.latDeg * Math.PI) / 180;
    this.omega = [0, OMEGA_EARTH * Math.cos(lat), OMEGA_EARTH * Math.sin(lat)];
  }

  get speed(): number {
    return Math.hypot(this.v[0], this.v[1], this.v[2]);
  }

  /** Kinetic energy, J. */
  get energy(): number {
    return 0.5 * this.load.massKg * this.speed ** 2;
  }

  /** Where it is, spin drift included. */
  at(): Vec3 {
    const sd = spinDrift(this.sg, this.t);
    return [this.p[0] + this.right[0] * sd, this.p[1] + this.right[1] * sd, this.p[2]];
  }

  private accel(p: Vec3, v: Vec3, t: number): Vec3 {
    const w = this.where.wind(p, t);
    const r: Vec3 = [v[0] - w[0], v[1] - w[1], v[2] - w[2]];
    const air = Math.hypot(r[0], r[1], r[2]);
    const k = dragFactor(this.load, this.where.air, air / this.where.air.sound) * air;
    const o = this.omega;
    // Coriolis: −2 Ω × v (the centrifugal part is in g).
    const cor: Vec3 = [
      -2 * (o[1] * v[2] - o[2] * v[1]),
      -2 * (o[2] * v[0] - o[0] * v[2]),
      -2 * (o[0] * v[1] - o[1] * v[0]),
    ];
    return [-k * r[0] + cor[0], -k * r[1] + cor[1], -k * r[2] - this.where.g + cor[2]];
  }

  /** On by `dt` s (one RK4 step). */
  step(dt = STEP_S): void {
    const { p, v, t } = this;
    const a1 = this.accel(p, v, t);
    const p2 = add(p, v, dt / 2);
    const v2 = add(v, a1, dt / 2);
    const a2 = this.accel(p2, v2, t + dt / 2);
    const p3 = add(p, v2, dt / 2);
    const v3 = add(v, a2, dt / 2);
    const a3 = this.accel(p3, v3, t + dt / 2);
    const p4 = add(p, v3, dt);
    const v4 = add(v, a3, dt);
    const a4 = this.accel(p4, v4, t + dt);
    for (let i = 0; i < 3; i++) {
      this.p[i] = p[i]! + (dt / 6) * (v[i]! + 2 * v2[i]! + 2 * v3[i]! + v4[i]!);
      this.v[i] = v[i]! + (dt / 6) * (a1[i]! + 2 * a2[i]! + 2 * a3[i]! + a4[i]!);
    }
    this.t += dt;
  }

  /** Slow it by `lost` m/s of its speed (what it left in something it went through). */
  slow(lost: number): void {
    const s = this.speed;
    const k = Math.max(0, s - lost) / (s || 1);
    this.v = [this.v[0] * k, this.v[1] * k, this.v[2] * k];
  }
}

/** The bore's angle above the line of sight, rad, that puts the bullet on the line of sight at
 * `zeroM` m, with the sight `sightHeight` m above the bore, the line of sight level, in still
 * air. (An 11.5" AR zeroed at 50 m crosses it again near 160 m.) */
export function zeroAngle(load: Load, where: Setting, sightHeight: number, zeroM: number): number {
  const calm: Setting = { ...where, wind: still, latDeg: 0 };
  // Height of the bullet over the sight line at the zero range, for a bore angle.
  const miss = (angle: number): number => {
    const r = new Round(load, calm, [0, 0, 0], [0, Math.cos(angle), Math.sin(angle)]);
    while (r.p[1] < zeroM) {
      const before = [...r.p];
      r.step();
      if (r.p[1] >= zeroM) {
        const k = (zeroM - before[1]!) / (r.p[1] - before[1]!);
        return before[2]! + k * (r.p[2] - before[2]!) - sightHeight;
      }
      if (r.t > 5) break;
    }
    return -1e9;
  };
  let lo = 0;
  let hi = 0.02;
  for (let i = 0; i < 60; i++) {
    const mid = (lo + hi) / 2;
    if (miss(mid) < 0) lo = mid;
    else hi = mid;
  }
  return (lo + hi) / 2;
}

function add(a: Vec3, b: Vec3, k: number): Vec3 {
  return [a[0] + b[0] * k, a[1] + b[1] * k, a[2] + b[2] * k];
}

function cross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

function norm(a: Vec3): Vec3 {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
}
