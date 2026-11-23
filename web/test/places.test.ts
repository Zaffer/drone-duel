// A place of the player's own (src/world/places.ts): what the menu's GPS entry reads, and the
// location it makes.

import { describe, expect, test } from "bun:test";
import { GPS_PREFIX, gpsPlace, isGpsPlace, parseLatLon } from "../src/world/places.ts";

describe("parseLatLon", () => {
  test.each([
    ["45.319821, -75.758630", [45.319821, -75.75863]],
    ["45.319821 -75.758630", [45.319821, -75.75863]],
    ["  45.3198,-75.7586 ", [45.3198, -75.7586]],
    ["45.3198° N, 75.7586° W", [45.3198, -75.7586]],
    ["33.8688 S 151.2093 E", [-33.8688, 151.2093]],
    ["−33.8688, 151.2093", [-33.8688, 151.2093]],
    ["45, -75", [45, -75]],
    ["https://www.google.com/maps/@43.6309298,-79.3894501,17z", [43.6309298, -79.3894501]],
    ["https://maps.google.com/?q=43.63093,-79.38945", [43.63093, -79.38945]],
  ])("%p", (text, at) => {
    expect(parseLatLon(text)).toEqual(at as [number, number]);
  });

  test.each([
    "",
    "Ottawa",
    "45.3",
    "91, 10",
    "10, 181",
    "45.3, -75.7, 12",
    "1.2.3, 4",
  ])("not %p", (text) => expect(parseLatLon(text)).toBeNull());
});

describe("gpsPlace", () => {
  test("both pads at the point, player 2's 10 m south; the ground looked up", () => {
    const loc = gpsPlace(51.50070123, -0.12457891);
    expect(loc.id).toBe(`${GPS_PREFIX}51.500701,-0.124579`);
    expect(isGpsPlace(loc.id)).toBe(true);
    expect(isGpsPlace("billy-bishop")).toBe(false);
    expect(loc.origin).toEqual([51.500701, -0.124579]);
    expect(loc.ground_alt_m).toBe("auto");
    expect(loc.pads[0]!.at).toEqual(loc.origin);
    const south = (loc.pads[0]!.at[0] - loc.pads[1]!.at[0]) * 111_320;
    expect(south).toBeCloseTo(10, 1);
    expect(loc.pads[1]!.at[1]).toBe(loc.origin[1]);
  });
});
