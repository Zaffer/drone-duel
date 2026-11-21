// From a controller's raw axes to the pilot's channels (DESIGN.md §12): pure functions, so they
// are tested on their own (test/mapping.test.ts). The server maps its own radio the same way
// (drone_duels.rc.mapping): both pass tests/fixtures/mapping.json.
//
// Channels are "natural", whatever the firmware: each -1..1 from channel 1, roll +1 right, pitch
// +1 stick forward (nose down), throttle +1 full, yaw +1 right; switches -1 low, +1 high. The
// firmware's own conventions (ArduPilot reads pitch forward as a low pulse) are applied on the
// way out (flight.ts's `toFirmware`).

import type { Controller, Vehicle } from "../net/protocol.ts";
import { toUs } from "../state/input.ts";
import type { AxisCal, Calibration } from "../state/prefs.ts";
import type { TouchSticks } from "./touch.ts";

/** Channels whose travel has a centre (the sticks other than the throttle). */
export const CENTRED = new Set([0, 1, 3]);

/** The first profile whose `match` strings all... any of them appear in the controller's id. */
export function profileFor(id: string, profiles: Controller[]): Controller | null {
  const lower = id.toLowerCase();
  return profiles.find((p) => p.match.some((m) => lower.includes(m.toLowerCase()))) ?? null;
}

/** A raw axis value through its calibration, to -1..1: centred channels about their centre,
 * the others straight from low to high. */
export function calibrate(raw: number, cal: AxisCal, centred: boolean): number {
  let x: number;
  if (centred) {
    x =
      raw >= cal.mid
        ? (raw - cal.mid) / (cal.max - cal.mid || 1)
        : (raw - cal.mid) / (cal.mid - cal.min || 1);
  } else {
    x = ((raw - cal.min) / (cal.max - cal.min || 1)) * 2 - 1;
  }
  return Math.max(-1, Math.min(1, x));
}

const IDENTITY: AxisCal = { min: -1, mid: 0, max: 1 };

/** The channels from raw axes and buttons: the player's calibration if there is one, else the
 * profile's map, else axis n as channel n + 1. */
export function toChannels(
  axes: readonly number[],
  buttons: readonly number[],
  profile: Controller | null,
  calibration: Calibration | undefined,
): number[] {
  if (calibration && calibration.channels.length > 0) {
    return calibration.channels.map((c, i) => {
      const x = calibrate(axes[c.axis] ?? 0, c.cal, CENTRED.has(i));
      return c.reverse ? -x : x;
    });
  }
  if (profile) {
    return profile.channels.map((src, i) => {
      if ("button" in src) return (buttons[src.button] ?? 0) > 0.5 ? 1 : -1;
      const x = calibrate(axes[src.axis] ?? 0, IDENTITY, CENTRED.has(i));
      return src.reverse ? -x : x;
    });
  }
  return axes.slice(0, 16).map((a) => Math.max(-1, Math.min(1, a)));
}

/** The channels as the firmware's pulses, µs, at least eight: ArduPilot reads pitch forward as
 * a low pulse, INAV as a high one (as Betaflight). */
export function toFirmware(ch: readonly number[], firmware: Vehicle["firmware"]["kind"]): number[] {
  const us = ch.slice(0, 16).map(toUs);
  while (us.length < 8) us.push(1000);
  if (firmware === "ardupilot-copter" && ch[1] !== undefined) us[1] = toUs(-ch[1]);
  return us;
}

/** A switch channel's position: 0 low, 1 middle, 2 high. */
export function switchPos(x: number | undefined): 0 | 1 | 2 {
  if (x === undefined || x < -0.33) return 0;
  return x > 0.33 ? 2 : 1;
}

/** Modes Betaflight will not arm in: its height and position holds are switched on in the air. */
const NOT_ARMED_IN = new Set(["POSHOLD", "ALTHOLD"]);

/** Where a made-up mode switch (the keyboard's, a gamepad's, a touch screen's) starts for a
 * vehicle whose switch picks `modes`, low to high: the first position it can be armed in. A
 * real radio's switch is wherever its pilot left it. */
export function startMode(modes: readonly string[]): number {
  const i = modes.slice(0, 3).findIndex((m) => !NOT_ARMED_IN.has(m));
  return Math.max(i, 0);
}

/** A standard-mapping gamepad (Xbox layout) as a mode 2 radio: left stick throttle and yaw,
 * right stick roll and pitch, A arms (toggles), X steps the flight mode. The throttle is the left
 * stick's travel, centre at half. */
export function gamepadChannels(pad: Gamepad, armed: boolean, modePos: number): number[] {
  const ax = (i: number) => pad.axes[i] ?? 0;
  return [ax(2), -ax(3), -ax(1), ax(0), armed ? 1 : -1, modePos - 1, -1, -1];
}

/** Keyboard flying, for testing without a radio: arrows roll and pitch, Q/E yaw, W/S move the
 * throttle (held where left), Enter arms (toggles), M steps the flight mode. */
export interface KeyState {
  throttle: number;
  armed: boolean;
  modePos: number;
}

export function keyboardChannels(keys: ReadonlySet<string>, state: KeyState, dt: number): number[] {
  const k = (code: string) => (keys.has(code) ? 1 : 0);
  state.throttle = Math.max(-1, Math.min(1, state.throttle + (k("KeyW") - k("KeyS")) * dt * 1.2));
  return [
    k("ArrowRight") - k("ArrowLeft"),
    k("ArrowUp") - k("ArrowDown"),
    state.throttle,
    k("KeyE") - k("KeyQ"),
    state.armed ? 1 : -1,
    state.modePos - 1,
    -1,
    -1,
  ];
}

/** A touch screen's sticks (input/touch.ts) as a mode 2 radio: the left stick throttle and yaw,
 * the right roll and pitch, its Arm and Mode buttons the two switches. */
export function touchChannels(
  t: Pick<TouchSticks, "left" | "right" | "armed" | "modePos">,
): number[] {
  return [t.right.x, t.right.y, t.left.y, t.left.x, t.armed ? 1 : -1, t.modePos - 1, -1, -1];
}
