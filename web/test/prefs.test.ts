import { describe, expect, test } from "bun:test";
import { DEFAULT_PREFS, mergePrefs } from "../src/state/prefs.ts";

describe("mergePrefs", () => {
  test("saved values over the defaults, missing ones from them", () => {
    const p = mergePrefs(DEFAULT_PREFS, { graphics: { quality: "low" } });
    expect(p.graphics.quality).toBe("low");
    expect(p.graphics.timeOfDay).toBe(12);
  });

  test("a saved null is kept: the real time of day, not the default noon", () => {
    const p = mergePrefs(DEFAULT_PREFS, { graphics: { timeOfDay: null } });
    expect(p.graphics.timeOfDay).toBeNull();
  });
});
