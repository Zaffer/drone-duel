import { describe, expect, test } from "bun:test";
import { calibrate, profileFor, switchPos, toChannels } from "../src/input/mapping.ts";
import type { Controller } from "../src/net/protocol.ts";

const literadio: Controller = {
  id: "literadio2",
  name: "BETAFPV LiteRadio 2",
  match: ["0483", "5750", "BETAFPV"],
  channels: [0, 1, 2, 3, 4, 5, 6, 7].map((axis) => ({
    axis,
    reverse: false,
  })) as Controller["channels"],
  notes: "",
};

describe("mapping", () => {
  test("a profile is picked by any of its match strings", () => {
    expect(
      profileFor("STMicroelectronics BETAFPV Joystick (Vendor: 0483 Product: 5750)", [literadio])
        ?.id,
    ).toBe("literadio2");
    expect(profileFor("Xbox Wireless Controller", [literadio])).toBeNull();
  });

  test("centred channels map about their centre, others end to end", () => {
    const cal = { min: -0.8, mid: 0.1, max: 0.9 };
    expect(calibrate(0.1, cal, true)).toBeCloseTo(0);
    expect(calibrate(0.9, cal, true)).toBeCloseTo(1);
    expect(calibrate(-0.8, cal, true)).toBeCloseTo(-1);
    expect(calibrate(-0.8, cal, false)).toBeCloseTo(-1);
    expect(calibrate(0.05, cal, false)).toBeCloseTo(0);
    expect(calibrate(5, cal, false)).toBe(1);
  });

  test("a calibration overrides the profile, axis and direction", () => {
    const axes = [0.5, -0.2, -1, 0, 1, 0, -1, 1];
    const cal = {
      channels: [
        { axis: 1, reverse: true, cal: { min: -1, mid: 0, max: 1 } },
        { axis: 0, reverse: false, cal: { min: -1, mid: 0, max: 1 } },
      ],
    };
    expect(toChannels(axes, [], literadio, cal)).toEqual([0.2, 0.5]);
    expect(toChannels(axes, [], literadio, undefined)).toEqual(axes);
  });

  test("switch positions", () => {
    expect(switchPos(-1)).toBe(0);
    expect(switchPos(0)).toBe(1);
    expect(switchPos(1)).toBe(2);
    expect(switchPos(undefined)).toBe(0);
  });
});
