import { describe, expect, test } from "bun:test";
import cases from "../../tests/fixtures/mapping.json";
import { calibrate, profileFor, switchPos, toChannels, toFirmware } from "../src/input/mapping.ts";
import type { Controller, Vehicle } from "../src/net/protocol.ts";
import type { Calibration } from "../src/state/prefs.ts";

// The cases the server's mapping (drone_duels.rc.mapping) passes too.
const profiles = cases.profiles as unknown as Record<string, Controller>;

describe("mapping", () => {
  test.each(cases.profile_for)("a profile is picked by any of its match strings: $id", (c) => {
    expect(profileFor(c.id, Object.values(profiles))?.id ?? null).toBe(c.profile);
  });

  test.each(cases.calibrate)("calibrate $raw (centred $centred)", (c) => {
    expect(calibrate(c.raw, c.cal, c.centred)).toBeCloseTo(c.out);
  });

  test.each(cases.channels)("$why", (c) => {
    const profile = c.profile ? profiles[c.profile]! : null;
    const got = toChannels(
      c.axes,
      c.buttons,
      profile,
      (c.calibration ?? undefined) as Calibration | undefined,
    );
    expect(got.length).toBe(c.out.length);
    got.forEach((x, i) => expect(x).toBeCloseTo(c.out[i]!));
  });

  test.each(cases.switch_pos)("switch position of $x", (c) => {
    expect(switchPos(c.x ?? undefined)).toBe(c.pos as 0 | 1 | 2);
  });

  test.each(cases.to_firmware)("$why", (c) => {
    expect(toFirmware(c.channels, c.firmware as Vehicle["firmware"]["kind"])).toEqual(c.us);
  });
});
