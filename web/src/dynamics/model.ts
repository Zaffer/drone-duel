// What the dynamics channel carries for the Chupito (drone_duels.physics.wing.dynamics: Dynamics.of
// as a dict, plus `motor`, INAV's throttle 0..1), typed for the overlay; and the pure parts of
// reading it. Degrees, m/s, N, N m, Pa, in the body frame (forward, right, down); the wind NED.

export type V3 = [number, number, number];

export interface StripDynamics {
  /** Its quarter chord from the centre of mass, m: where its force acts. */
  at: V3;
  /** Unit: its lift side. */
  up: V3;
  /** Which elevon it carries, 0 left, 1 right; -1 none. */
  elevon: number;
  alpha: number;
  downwash: number;
  seen: number;
  stall: number;
  lead: number;
  ground_effect: number;
  flap: number;
  attached: number;
  cl: number;
  cd: number;
  cm: number;
  q: number;
  force: V3;
  /** Leading edge at either end, then the trailing edge back the other way, m. */
  corners: [V3, V3, V3, V3];
  circulation: number;
  air: V3;
}

export interface Dynamics {
  airspeed: number;
  alpha: number;
  beta: number;
  dynamic_pressure: number;
  /** CL, CD, CY, Cl, Cm, Cn. */
  coefficients: number[];
  wind: V3;
  wind_mean: V3;
  turbulence: V3;
  gust: V3;
  turbulence_rates: V3;
  aero: V3;
  pod_drag: V3;
  prop: V3;
  gravity: V3;
  total: V3;
  load: V3;
  aero_moment: V3;
  munk: V3;
  prop_moment: V3;
  gyroscopic: V3;
  total_moment: V3;
  rates: V3;
  angular_acceleration: V3;
  servos: number[];
  elevons: number[];
  effective: number[];
  flex: number;
  rpm: number;
  advance_ratio: number;
  ct: number;
  cp: number;
  thrust: number;
  motor_torque: number;
  prop_torque: number;
  motor_amps: number;
  pack_volts: number;
  pack_amps: number;
  used_mah: number;
  prop_at: V3;
  strips: StripDynamics[];
  /** Each trailing vortex: where it leaves the wing (m), its strength (m²/s). */
  wake: [V3, number][];
  centre_of_pressure: number | null;
  neutral_point: number | null;
  static_margin: number | null;
  height: number;
  energy_height: number;
  energy_rate: number;
  /** INAV's throttle, 0..1. */
  motor?: number;
}

/** The data as Dynamics, if it looks like it (the wing's; another vehicle sends none). */
export function asDynamics(data: Record<string, unknown> | null): Dynamics | null {
  if (!data) return null;
  const ok =
    Array.isArray(data.strips) &&
    Array.isArray(data.wake) &&
    Array.isArray(data.coefficients) &&
    typeof data.airspeed === "number";
  return ok ? (data as unknown as Dynamics) : null;
}

/** The least attached share of flow over the strips (NaN with none). */
export const leastAttached = (d: Dynamics): number =>
  d.strips.length ? Math.min(...d.strips.map((s) => s.attached)) : Number.NaN;

/** `x` to `digits` decimals; "–" for no value, and no "-0". */
export function fmt(x: number | null | undefined, digits: number): string {
  if (x === null || x === undefined || !Number.isFinite(x)) return "–";
  const text = x.toFixed(digits);
  return /^-0(\.0*)?$/.test(text) ? text.slice(1) : text;
}

/** Body (forward, right, down) to east-north-up for a body whose forward, right and up are
 * `fwd`, `right`, `up` there; times `k`. */
export function bodyToEnu(axes: { fwd: V3; right: V3; up: V3 }, v: readonly number[], k = 1): V3 {
  const { fwd, right, up } = axes;
  const [x = 0, y = 0, z = 0] = v;
  return [
    k * (x * fwd[0] + y * right[0] - z * up[0]),
    k * (x * fwd[1] + y * right[1] - z * up[1]),
    k * (x * fwd[2] + y * right[2] - z * up[2]),
  ];
}

/** The wing's strips named as seen from behind: L6 (left tip) to L1, R1 to R6, then the fin;
 * each with its index in the data. */
export function nameStrips(
  strips: readonly StripDynamics[],
): { s: StripDynamics; i: number; name: string }[] {
  const named = strips.map((s, i) => ({ s, i, fin: Math.abs(s.up[1]) > 0.5, name: "" }));
  const wing = named.filter((n) => !n.fin).sort((a, b) => a.s.at[1] - b.s.at[1]);
  const left = wing.filter((n) => n.s.at[1] < 0);
  left.forEach((n, k) => {
    n.name = `L${left.length - k}`;
  });
  wing
    .filter((n) => n.s.at[1] >= 0)
    .forEach((n, k) => {
      n.name = `R${k + 1}`;
    });
  const fin = named.filter((n) => n.fin).map((n) => ({ ...n, name: "fin" }));
  return [...wing, ...fin].map(({ s, i, name }) => ({ s, i, name }));
}

/** A colour stop ramp's value at `v`: the stops either side and how far between, 0..1. */
export function rampAt(
  stops: readonly [number, string][],
  v: number,
): { a: number; b: number; t: number } {
  const last = stops.length - 1;
  if (!(v > stops[0]![0])) return { a: 0, b: 0, t: 0 };
  for (let k = 1; k <= last; k++) {
    if (v <= stops[k]![0]) {
      return { a: k - 1, b: k, t: (v - stops[k - 1]![0]) / (stops[k]![0] - stops[k - 1]![0]) };
    }
  }
  return { a: last, b: last, t: 0 };
}
