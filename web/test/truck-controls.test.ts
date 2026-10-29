import { describe, expect, test } from "bun:test";
import {
  TRUCK_CHANNELS,
  type TruckControls,
  TruckInputState,
  truckControls,
} from "../src/vehicles/truck/controls.ts";

const NONE = new Set<string>();
const keys = (...codes: string[]) => new Set(codes);

/** A radio's channels: roll, pitch, throttle, yaw, CH5 handbrake, CH6 gear, CH7 reset. */
function radio({
  throttle = -1,
  yaw = 0,
  handbrake = -1,
  gear = 1,
  reset = -1,
}: Partial<Record<"throttle" | "yaw" | "handbrake" | "gear" | "reset", number>> = {}) {
  return [0, 0, throttle, yaw, handbrake, gear, reset, -1];
}

function pad(
  axes: number[],
  pressed: Record<number, number> = {},
  mapping: GamepadMappingType = "standard",
): Gamepad {
  const buttons = Array.from({ length: 17 }, (_, i) => ({
    pressed: (pressed[i] ?? 0) > 0.5,
    touched: false,
    value: pressed[i] ?? 0,
  }));
  return { axes, buttons, mapping, connected: true, id: "pad", index: 0 } as unknown as Gamepad;
}

const run = (s: TruckInputState, ch: number[], k = NONE, p: Gamepad | null = null): TruckControls =>
  truckControls(ch, k, p, s);

describe("truck controls: keyboard", () => {
  test("WASD and arrows, space, R", () => {
    const s = new TruckInputState();
    expect(run(s, [], keys("KeyW"))).toMatchObject({ throttle: 1, steer: 0, handbrake: false });
    expect(run(s, [], keys("ArrowDown")).throttle).toBe(-1);
    expect(run(s, [], keys("KeyD")).steer).toBe(1);
    expect(run(s, [], keys("ArrowLeft")).steer).toBe(-1);
    expect(run(s, [], keys("KeyW", "KeyS")).throttle).toBe(0);
    expect(run(s, [], keys("Space")).handbrake).toBe(true);
  });

  test("R resets once per press", () => {
    const s = new TruckInputState();
    expect(run(s, [], keys("KeyR")).reset).toBe(true);
    expect(run(s, [], keys("KeyR")).reset).toBe(false);
    expect(run(s, [], NONE).reset).toBe(false);
    expect(run(s, [], keys("KeyR")).reset).toBe(true);
  });
});

describe("truck controls: radio", () => {
  test("the throttle drives nothing until it has been at the bottom (the rearm rule)", () => {
    const s = new TruckInputState();
    // Plugged in with the stick left at half: waits.
    expect(run(s, radio({ throttle: 0 })).throttle).toBe(0);
    expect(s.waiting).toBe(true);
    // To the bottom, then up: drives.
    expect(run(s, radio({ throttle: -1 })).throttle).toBe(0);
    expect(s.waiting).toBe(false);
    expect(run(s, radio({ throttle: 0 })).throttle).toBeCloseTo(0.5, 9);
    expect(run(s, radio({ throttle: 1 })).throttle).toBeCloseTo(1, 9);
    // Placed again: waits for the bottom again.
    s.rearm();
    expect(run(s, radio({ throttle: 1 })).throttle).toBe(0);
    expect(s.waiting).toBe(true);
    run(s, radio({ throttle: -0.9 }));
    expect(run(s, radio({ throttle: 1 })).throttle).toBeCloseTo(1, 9);
  });

  test("the gear: forward, neutral, reverse", () => {
    const s = new TruckInputState();
    run(s, radio());
    expect(run(s, radio({ throttle: 1, gear: 1 })).throttle).toBeCloseTo(1, 9);
    expect(run(s, radio({ throttle: 1, gear: 0 })).throttle).toBe(0);
    expect(run(s, radio({ throttle: 0, gear: -1 })).throttle).toBeCloseTo(-0.5, 9);
  });

  test("a deadband at the bottom of the stick", () => {
    const s = new TruckInputState();
    run(s, radio());
    expect(run(s, radio({ throttle: -0.85 })).throttle).toBe(0); // lift 0.075
    expect(run(s, radio({ throttle: -0.7 })).throttle).toBeCloseTo(0.15, 9);
  });

  test("steer is the yaw stick, right +, with a deadband", () => {
    const s = new TruckInputState();
    expect(run(s, radio({ yaw: 0.6 })).steer).toBeCloseTo(0.6, 9);
    expect(run(s, radio({ yaw: -1 })).steer).toBe(-1);
    expect(run(s, radio({ yaw: 0.05 })).steer).toBe(0);
  });

  test("the radio overrides the keyboard only where it moves", () => {
    const s = new TruckInputState();
    const c = run(s, radio({ yaw: 0.02 }), keys("KeyW", "KeyA"));
    expect(c.throttle).toBe(1);
    expect(c.steer).toBe(-1);
    expect(run(s, radio({ yaw: 0.5 }), keys("KeyA")).steer).toBeCloseTo(0.5, 9);
  });

  test("handbrake switch", () => {
    const s = new TruckInputState();
    expect(run(s, radio({ handbrake: 1 })).handbrake).toBe(true);
    expect(run(s, radio({ handbrake: -1 })).handbrake).toBe(false);
  });

  test("the reset switch resets on every flick, either way, not on its first reading", () => {
    const s = new TruckInputState();
    expect(run(s, radio({ reset: 1 })).reset).toBe(false);
    expect(run(s, radio({ reset: 1 })).reset).toBe(false);
    expect(run(s, radio({ reset: -1 })).reset).toBe(true);
    expect(run(s, radio({ reset: -1 })).reset).toBe(false);
    expect(run(s, radio({ reset: 1 })).reset).toBe(true);
  });

  test("a radio without a gear switch drives forward", () => {
    const s = new TruckInputState();
    run(s, [0, 0, -1, 0]);
    expect(run(s, [0, 0, 1, 0]).throttle).toBeCloseTo(1, 9);
  });

  test("a channel map", () => {
    const s = new TruckInputState();
    const map = { ...TRUCK_CHANNELS, steer: { index: 0, sign: -1 as const } };
    expect(truckControls([0.4, 0, -1, 0], NONE, null, s, map).steer).toBeCloseTo(-0.4, 9);
  });
});

describe("truck controls: gamepad", () => {
  test("left stick steers, triggers drive and brake, A handbrake, Y reset", () => {
    const s = new TruckInputState();
    expect(run(s, [], NONE, pad([0.5, 0]))).toMatchObject({ steer: 0.5, throttle: 0 });
    expect(run(s, [], NONE, pad([0.03, 0])).steer).toBe(0);
    expect(run(s, [], NONE, pad([0, 0], { 7: 0.8 })).throttle).toBeCloseTo(0.8, 9);
    expect(run(s, [], NONE, pad([0, 0], { 6: 1 })).throttle).toBe(-1);
    expect(run(s, [], NONE, pad([0, 0], { 0: 1 })).handbrake).toBe(true);
    expect(run(s, [], NONE, pad([0, 0], { 3: 1 })).reset).toBe(true);
    expect(run(s, [], NONE, pad([0, 0], { 3: 1 })).reset).toBe(false);
  });

  test("a pad without the standard mapping (a radio) is ignored", () => {
    const s = new TruckInputState();
    expect(run(s, [], NONE, pad([1, 0], { 7: 1 }, "")).throttle).toBe(0);
  });

  test("the gamepad overrides the keyboard when moved", () => {
    const s = new TruckInputState();
    expect(run(s, [], keys("KeyW"), pad([0, 0])).throttle).toBe(1);
    expect(run(s, [], keys("KeyW"), pad([0, 0], { 6: 0.5 })).throttle).toBeCloseTo(-0.5, 9);
  });
});
