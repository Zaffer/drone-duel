// The sounds' pure maths (the old sounds.js's): motor and engine speeds, loudness with
// distance, doppler. No Web Audio here, so it can be tested.

export const hz = (midi: number): number => 440 * 2 ** ((midi - 69) / 12);
export const clamp01 = (x: number): number => Math.min(1, Math.max(0, Number(x) || 0));

/** The truck's top speed in each gear, m/s: the revs climb through a gear and drop at a change. */
const GEARS = [6, 13, 21, 34];
/** The 5" quad's motors (2207 on 5.1" tri-blades): about 30,000 rpm at full output. */
export const QUAD_MOTOR = { maxRevS: 500, blades: 3, polePairs: 7, thrustExpo: 0.7 };
/** The Chupito's 2200 KV motor on 6S with a 5x3x3 prop: about 45,000 rpm loaded at full
 * throttle (55,000 unloaded). */
export const WING_MOTOR = { maxRevS: 750, blades: 3, polePairs: 7 };
/** Rotor speeds, rad/s, at which motor() takes rpmNorm 1. */
export const QUAD_MAX_RAD_S = QUAD_MOTOR.maxRevS * 2 * Math.PI;
export const WING_MAX_RAD_S = WING_MOTOR.maxRevS * 2 * Math.PI;

/** A rotor speed, rad/s, as motor()'s rpmNorm 0..1. */
export function rotorNorm(kind: "multirotor" | "fixedwing", radS: number): number {
  return clamp01(Math.abs(radS) / (kind === "multirotor" ? QUAD_MAX_RAD_S : WING_MAX_RAD_S));
}

/** A drone this near is heard at full loudness; twice as far, half as loud. */
const DRONE_FULL_M = 10;
/** The truck, heard from outside it, likewise. */
const TRUCK_FULL_M = 25;
const SOUND_MS = 343;
/** A pitch shifts with speed towards the listener, up to this fraction. */
const DOPPLER_MAX = 0.15;

/** A motor's speed, 0 (stopped) to 1 (full), from its output in µs (1000 to 2000): its thrust
 * on ArduPilot's thrust curve (MOT_THST_EXPO), the speed that thrust's square root. */
export function motorRevs(us: number): number {
  const out = clamp01((us - 1000) / 1000);
  const e = QUAD_MOTOR.thrustExpo;
  return Math.sqrt((1 - e) * out + e * out * out);
}

/** The truck's diesel at `speed` m/s and `throttle` -1..1: revs 0 (idle) to 1, load 0..1, and
 * the speed as a fraction of the top. Feed `rpm` and `load` to motor("ground", load, rpm). */
export function truckEngine(
  speed: number,
  throttle: number,
): { rpm: number; load: number; speed: number } {
  const v = Math.abs(speed);
  let gear = 0;
  let low = 0;
  while (gear < GEARS.length - 1 && v >= GEARS[gear]!) low = GEARS[gear++]!;
  const frac = Math.min(1, (v - low) / (GEARS[gear]! - low));
  const load = clamp01(Math.abs(throttle));
  const rpm = (gear === 0 ? 0.8 * frac : 0.3 + 0.65 * frac) + 0.12 * load;
  return { rpm: Math.min(1, rpm), load, speed: Math.min(1, v / GEARS.at(-1)!) };
}

/** A drone's loudness `range` m from the listener: full up to 10 m, then as 1 / distance. */
export const droneGain = (range: number): number =>
  Math.min(1, DRONE_FULL_M / Math.max(range, 1e-3));

/** The truck's loudness `range` m from the listener. */
export const truckGain = (range: number): number =>
  Math.min(1, TRUCK_FULL_M / Math.max(range, 1e-3));

/** The factor on a pitch closing on the listener at `rate` m/s (opening: negative). */
export function doppler(rate: number): number {
  const f = SOUND_MS / (SOUND_MS - rate);
  return Math.min(1 + DOPPLER_MAX, Math.max(1 - DOPPLER_MAX, f));
}

/** The duel's "drone closing" pings start once the nearest drone is this near the truck, m. */
export const PROXIMITY_M = 150;

/** Seconds between the pings with the nearest drone `range` m from the truck; null when it is
 * too far to ping. */
export function proximityGap(range: number): number | null {
  if (!(range < PROXIMITY_M)) return null;
  return 0.22 + (1.6 * range) / PROXIMITY_M;
}
