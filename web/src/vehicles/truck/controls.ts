// The truck's controls from what the player holds (the old drive's driveradio.js and
// drivesession.js): the radio's channels, the keyboard and a standard (Xbox-style) gamepad.
//
// - Radio: each job is one channel (TRUCK_CHANNELS, as the old DEFAULT_MAP). The throttle stick,
//   its whole travel, drives in the gear: forward, neutral or reverse on a 3-position switch
//   (reverse brakes first). The stick stays where it is left, so it drives nothing until it
//   has been at the bottom once after each placing (the rearm rule, `TruckInputState.rearm`).
//   The reset switch puts the truck back on its wheels on every flick, either way.
// - Keyboard (KeyboardEvent.code): W/S or ↑/↓ throttle and brake/reverse, A/D or ←/→ steer,
//   Space the handbrake, R reset.
// - Gamepad: left stick steers, right trigger is the throttle, left trigger brakes and
//   reverses, A the handbrake, Y reset.
//
// The keyboard sets the controls; a radio overrides each that it moves, and a gamepad likewise.
// Steer is +1 to the right here, as a stick; the chassis' own +1 is to the left (car.ts).

export interface TruckControls {
  /** -1..1: forward, or against the motion a brake and then reverse. */
  throttle: number;
  /** -1 (left) .. 1 (right). */
  steer: number;
  handbrake: boolean;
  /** True on the frame the player asks to be put back on the wheels. */
  reset?: boolean;
}

/** A job's channel: its index (channel 1 is 0) and which way round it runs. */
export interface ChannelPick {
  index: number;
  sign: 1 | -1;
}

/** The radio's jobs, by channel. The radio's order is AETR then AUX: CH3 throttle, CH4 yaw
 * steers, CH5 handbrake, CH6 (the 3-position mode switch) the gear, CH7 reset. */
export const TRUCK_CHANNELS: Record<
  "throttle" | "steer" | "gear" | "handbrake" | "reset",
  ChannelPick
> = {
  throttle: { index: 2, sign: 1 },
  steer: { index: 3, sign: 1 },
  gear: { index: 5, sign: 1 },
  handbrake: { index: 4, sign: 1 },
  reset: { index: 6, sign: 1 },
};

const RADIO_DEADBAND = 0.1;
const PAD_STICK_DEADBAND = 0.08;
const PAD_TRIGGER_DEADBAND = 0.05;
/** Standard gamepad buttons. */
const PAD = { handbrake: 0, reset: 3, brake: 6, gas: 7 };

/** -1, 0 or 1: which third of its travel a channel is in. */
export const third = (v: number): -1 | 0 | 1 => (v > 0.5 ? 1 : v < -0.5 ? -1 : 0);

/** What the controls remember between frames: the rearm rule and the reset switch. */
export class TruckInputState {
  /** The radio's throttle has been at the bottom since the last placing. */
  ready = false;
  /** A radio is in use and its throttle waits for the bottom: tell the player. */
  waiting = false;
  private resetAt: number | null = null;
  private resetKey = false;
  private resetButton = false;

  /** The truck has just been put down: the radio's throttle waits for the bottom again. */
  rearm(): void {
    this.ready = false;
  }

  /** @internal Whether the reset switch moved since the last frame. */
  flicked(position: number | null): boolean {
    const was = this.resetAt;
    this.resetAt = position;
    return was !== null && position !== null && position !== was;
  }

  /** @internal Whether a key or button just went down. */
  pressed(which: "key" | "button", down: boolean): boolean {
    const field = which === "key" ? "resetKey" : "resetButton";
    const was = this[field];
    this[field] = down;
    return down && !was;
  }
}

/** The page's one truck input state; the truck rearms it each time it is placed. */
export const truckInput = new TruckInputState();

const has = (keys: ReadonlySet<string>, ...codes: string[]) => codes.some((c) => keys.has(c));

/** This frame's truck controls. `channels`: the radio's, -1..1 from channel 1 (empty with no
 * radio). `keys`: the KeyboardEvent.codes held. `pad`: a gamepad, used only when it has the
 * standard mapping (a radio in joystick mode comes through `channels`). */
export function truckControls(
  channels: readonly number[],
  keys: ReadonlySet<string>,
  pad: Gamepad | null,
  state: TruckInputState = truckInput,
  map: typeof TRUCK_CHANNELS = TRUCK_CHANNELS,
): TruckControls {
  let throttle = (has(keys, "KeyW", "ArrowUp") ? 1 : 0) - (has(keys, "KeyS", "ArrowDown") ? 1 : 0);
  let steer = (has(keys, "KeyD", "ArrowRight") ? 1 : 0) - (has(keys, "KeyA", "ArrowLeft") ? 1 : 0);
  let handbrake = keys.has("Space");
  let reset = state.pressed("key", keys.has("KeyR"));

  // A radio: at least its four sticks.
  if (channels.length >= 4) {
    const value = (job: keyof typeof TRUCK_CHANNELS): number | undefined => {
      const { index, sign } = map[job];
      const v = channels[index];
      return v === undefined ? undefined : v * sign;
    };
    const lift = ((value("throttle") ?? -1) + 1) / 2;
    // No gear switch on this radio: always forward.
    const gearValue = value("gear");
    const gear = gearValue === undefined ? 1 : third(gearValue);
    const atBottom = lift < RADIO_DEADBAND;
    if (atBottom) state.ready = true;
    state.waiting = !state.ready;
    const radioThrottle = state.ready && !atBottom ? gear * lift : 0;
    const radioSteer = value("steer") ?? 0;
    if (Math.abs(radioSteer) > RADIO_DEADBAND) steer = radioSteer;
    if (radioThrottle !== 0) throttle = radioThrottle;
    handbrake ||= (value("handbrake") ?? 0) > 0.5;
    const resetValue = value("reset");
    if (state.flicked(resetValue === undefined ? null : third(resetValue))) reset = true;
  } else {
    state.waiting = false;
    state.flicked(null);
  }

  if (pad?.connected && pad.mapping === "standard") {
    const stick = pad.axes[0] ?? 0;
    if (Math.abs(stick) > PAD_STICK_DEADBAND) steer = stick;
    const gas = pad.buttons[PAD.gas]?.value ?? 0;
    const brake = pad.buttons[PAD.brake]?.value ?? 0;
    if (gas > PAD_TRIGGER_DEADBAND || brake > PAD_TRIGGER_DEADBAND) throttle = gas - brake;
    handbrake ||= pad.buttons[PAD.handbrake]?.pressed ?? false;
    if (state.pressed("button", pad.buttons[PAD.reset]?.pressed ?? false)) reset = true;
  }

  return {
    throttle: Math.max(-1, Math.min(1, throttle)),
    steer: Math.max(-1, Math.min(1, steer)),
    handbrake,
    reset,
  };
}
