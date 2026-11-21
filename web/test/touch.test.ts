import { describe, expect, test } from "bun:test";
import { startMode, touchChannels } from "../src/input/mapping.ts";
import { dragged, TouchSticks } from "../src/input/touch.ts";

describe("the touch screen's sticks", () => {
  test("a stick moves by the thumb's travel from where it stood, up the screen being +1", () => {
    expect(dragged({ x: 0, y: 0 }, 30, -15, 60)).toEqual({ x: 0.5, y: 0.25 });
    expect(dragged({ x: 0, y: -1 }, 0, -60, 60)).toEqual({ x: 0, y: 0 });
    expect(dragged({ x: 0.5, y: 0.5 }, 0, 0, 60)).toEqual({ x: 0.5, y: 0.5 });
  });

  test("it stops at its rim", () => {
    expect(dragged({ x: 0, y: 0 }, 500, 500, 60)).toEqual({ x: 1, y: -1 });
    expect(dragged({ x: 0, y: -1 }, -500, 10, 60)).toEqual({ x: -1, y: -1 });
  });

  test("an aircraft's throttle stays where it is left; the rest spring back", () => {
    const t = new TouchSticks();
    t.reset(false);
    expect(t.left).toEqual({ x: 0, y: -1 });
    t.drag("left", { ...t.left }, 20, -90, 60);
    t.drag("right", { ...t.right }, -30, 30, 60);
    expect(t.left.x).toBeCloseTo(1 / 3);
    expect(t.left.y).toBeCloseTo(0.5);
    t.release("left");
    t.release("right");
    expect(t.left).toEqual({ x: 0, y: 0.5 });
    expect(t.right).toEqual({ x: 0, y: 0 });
    // The next thumb takes the throttle from there: nothing jumps.
    t.drag("left", { ...t.left }, 0, 0, 60);
    expect(t.left.y).toBeCloseTo(0.5);
  });

  test("the truck's is sprung: it starts at its centre and goes back to it", () => {
    const t = new TouchSticks();
    t.reset(true);
    expect(t.left).toEqual({ x: 0, y: 0 });
    t.drag("left", { ...t.left }, 0, -60, 60);
    expect(t.left.y).toBe(1);
    t.release("left");
    expect(t.left).toEqual({ x: 0, y: 0 });
  });

  test("a new vehicle is a radio just switched on: disarmed, the mode low", () => {
    const t = new TouchSticks();
    t.armed = true;
    t.modePos = 2;
    t.handbrake = true;
    t.left = { x: 0.2, y: 0.8 };
    t.reset(false);
    expect(t).toMatchObject({ armed: false, modePos: 0, handbrake: false });
    expect(t.left).toEqual({ x: 0, y: -1 });
  });

  test("as a mode 2 radio's channels: roll, pitch, throttle, yaw, arm, mode", () => {
    const t = new TouchSticks();
    t.reset(false);
    expect(touchChannels(t)).toEqual([0, 0, -1, 0, -1, -1, -1, -1]);
    t.left = { x: -0.25, y: 0.5 };
    t.right = { x: 0.75, y: -1 };
    t.armed = true;
    t.modePos = 1;
    expect(touchChannels(t)).toEqual([0.75, -1, 0.5, -0.25, 1, 0, -1, -1]);
    t.modePos = 2;
    expect(touchChannels(t)[5]).toBe(1);
  });
});

describe("a made-up mode switch", () => {
  test("starts at the first mode the vehicle arms in", () => {
    expect(startMode(["POSHOLD", "ANGLE", "ACRO"])).toBe(1);
    expect(startMode(["LOITER", "STABILIZE", "ACRO"])).toBe(0);
    expect(startMode(["ANGLE", "ACRO", "MANUAL"])).toBe(0);
    expect(startMode([])).toBe(0);
    expect(startMode(["POSHOLD"])).toBe(0);
  });
});

describe("before a reset by a tap", () => {
  test("the sticks are let go: disarmed, the throttle down, the mode kept", () => {
    const t = new TouchSticks();
    t.reset(false, 1);
    t.armed = true;
    t.left = { x: 0, y: 0.6 };
    let drawn = 0;
    t.onChange = () => drawn++;
    t.rest();
    expect(t).toMatchObject({ armed: false, modePos: 1 });
    expect(t.left.y).toBe(-1);
    expect(drawn).toBe(1);
  });
});
