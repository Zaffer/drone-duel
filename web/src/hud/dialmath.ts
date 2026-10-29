// The dials' numbers (hud/dials.ts), apart from the drawing so they can be tested: each value to
// its needle's angle, the altimeter's drum, the attitude's horizon and the stick meters.

/** A dial's radius, and where the dials' centres sit, in the SVG's units. */
export const R = 21;
export const CY = 35;
/** Both scales are 270° arcs. Climb: zero at 9 o'clock, gap on the right. Speed: zero at
 * 7:30, clockwise, gap at the bottom. Angles from 3 o'clock, clockwise (SVG's). */
export const SWEEP = 1.5 * Math.PI;
export const CLIMB_T0 = 0.25 * Math.PI;
export const SPEED_T0 = 0.75 * Math.PI;
/** Pitch at which the horizon line reaches most of the dial's edge (0.8 R). */
export const PITCH_FULL_DEG = 45;

export type Units = "metric" | "imperial";
export type VehicleKind = "multirotor" | "fixedwing" | "ground";

/** How each dial reads in a system of units. */
export interface Scales {
  /** The altimeter: its unit, the height for one turn of its needle, the drum's digits. */
  alt: { unit: string; perM: number; turn: number; digits: number };
  /** Vertical speed: unit, per m/s, full scale either way. */
  climb: { unit: string; perMs: number; full: number; decimals: number };
  /** Ground speed: unit, per m/s, full scale. */
  speed: { unit: string; perMs: number; full: number };
}

/** The dials' scales: metres and m/s, or feet, ft/min and mph; ground speed's full scale fits
 * the vehicle (a 5" quad or the truck to 30 m/s, the Chupito to 40). */
export function scalesFor(units: Units, kind: VehicleKind = "multirotor"): Scales {
  const fullMs = kind === "fixedwing" ? 40 : 30;
  if (units === "imperial") {
    return {
      alt: { unit: "ft", perM: 3.28084, turn: 1000, digits: 4 },
      climb: { unit: "ft/min", perMs: 196.85, full: 1000, decimals: 0 },
      speed: { unit: "mph", perMs: 2.23694, full: Math.round((fullMs * 2.23694) / 10) * 10 },
    };
  }
  return {
    alt: { unit: "m", perM: 1, turn: 100, digits: 3 },
    climb: { unit: "m/s", perMs: 1, full: 5, decimals: 1 },
    speed: { unit: "m/s", perMs: 1, full: fullMs },
  };
}

export const clamp = (x: number, lo: number, hi: number): number => Math.min(Math.max(x, lo), hi);
const deg = (rad: number): number => (rad * 180) / Math.PI;

/** The altimeter's needle, degrees clockwise from 12: once round every `turn`. */
export function altNeedleDeg(alt: number, turn: number): number {
  return ((Math.max(alt, 0) % turn) / turn) * 360;
}

/** Each drum column's position for `value`, 0 to 10 digits up its strip, most significant
 * first: the units turn with the value, and each higher digit turns over in the last unit
 * before it changes, as an odometer's do. */
export function drumPositions(value: number, digits = 3): number[] {
  const m = clamp(value, 0, 10 ** digits - 1e-6);
  const out: number[] = [];
  for (let i = digits - 1; i >= 0; i--) {
    const place = 10 ** i;
    const below = m % place;
    const digit = Math.floor(m / place) % 10;
    // Turning over: everything below this digit reads 9s and the units are rolling to 0.
    out.push(i === 0 ? m % 10 : digit + Math.max(0, below - (place - 1)));
  }
  return out;
}

/** The vertical speed needle's rotation, degrees clockwise from 3 o'clock: 9 o'clock level,
 * up climbing, down descending, `full` either way at the scale's ends. */
export function climbNeedleDeg(climb: number, full: number): number {
  return deg(CLIMB_T0 + (0.5 + 0.5 * clamp(climb / full, -1, 1)) * SWEEP);
}

/** The ground speed needle's rotation, degrees clockwise from 3 o'clock: 0 at 7:30, `full` at
 * 4:30. */
export function speedNeedleDeg(speed: number, full: number): number {
  return deg(SPEED_T0 + clamp(Math.max(speed, 0) / full, 0, 1) * SWEEP);
}

/** The horizon line's drop from the dial's centre for `pitchDeg` (nose up: it drops). */
export function horizonDy(pitchDeg: number): number {
  return clamp(pitchDeg / PITCH_FULL_DEG, -1, 1) * R * 0.8;
}

/** A heading, degrees true, 0-359, and as the compass writes it ("007°"). */
export function headingOf(yawDeg: number): number {
  return ((yawDeg % 360) + 360) % 360;
}
export const headingText = (yawDeg: number): string =>
  `${String(Math.round(headingOf(yawDeg)) % 360).padStart(3, "0")}°`;

/** The vertical speed as written under its dial: signed, a true minus. */
export function climbText(climb: number, decimals: number): string {
  const v = Number(Math.abs(climb).toFixed(decimals));
  return `${climb >= 0 || v === 0 ? "+" : "−"}${v.toFixed(decimals)}`;
}

/** The stick meters, each -1..1 (throttle 0..1), from the channels (-1..1, AETR from channel
 * 1); null without them. The pitch meter draws stick forward up: ArduPilot takes forward as a
 * low pulse, INAV as a high one. */
export function stickMeters(
  channels: readonly number[],
  pitchForwardLow: boolean,
): { roll: number; pitch: number; throttle: number; yaw: number } | null {
  const [roll, pitch, throttle, yaw] = channels;
  if (roll === undefined || pitch === undefined || throttle === undefined || yaw === undefined) {
    return null;
  }
  return {
    roll: clamp(roll, -1, 1),
    pitch: clamp(pitchForwardLow ? -pitch : pitch, -1, 1),
    throttle: clamp((throttle + 1) / 2, 0, 1),
    yaw: clamp(yaw, -1, 1),
  };
}
