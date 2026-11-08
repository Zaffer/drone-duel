import { describe, expect, test } from "bun:test";
import { Air, shear } from "../src/shoot/air.ts";
import { Hold } from "../src/shoot/hold.ts";
import {
  type LevelId,
  levelsFor,
  PATH_POINTS,
  pathAt,
  pathWaypoints,
} from "../src/shoot/levels.ts";
import { Recoil, recoilImpulse } from "../src/shoot/recoil.ts";
import { payloadZone, strikes } from "../src/shoot/target.ts";

const never = () => 1; // a blade is never in the way
const always = () => 0; // a blade is always in the way

describe("what a bullet hits on the quad", () => {
  test("straight down through the middle: the pack, the plates, the stack, in order", () => {
    const hit = strikes([0, 0, -1], [0, 0, 1], never).map((s) => [s.part, s.label]);
    expect(hit).toEqual([
      ["battery", "battery"],
      ["body", "frame"],
      ["body", "flight controller"],
      ["body", "frame"],
    ]);
  });

  test("down through motor 1 (front right): its prop (a blade in the way) and its bell", () => {
    const hit = strikes([0.0868, 0.1051, -1], [0.0868, 0.1051, 1], always);
    expect(hit.map((s) => [s.part, s.motor, s.label])).toEqual([["arm", 1, "motor"]]);
    const offHub = strikes([0.0868 + 0.04, 0.1051, -1], [0.0868 + 0.04, 0.1051, 1], always);
    expect(offHub.map((s) => [s.part, s.motor, s.label])).toEqual([["arm", 1, "prop"]]);
    const clear = strikes([0.0868 + 0.04, 0.1051, -1], [0.0868 + 0.04, 0.1051, 1], never);
    expect(clear).toEqual([]);
  });

  test("level from the side, through the rear left arm", () => {
    // Motor 2 is rear left: its arm runs from (-0.025, -0.014) to (-0.0868, -0.1051).
    const hit = strikes([-0.06, 1, -0.0025], [-0.06, -1, -0.0025], never);
    const arms = hit.filter((s) => s.label === "arm").map((s) => s.motor);
    expect(arms).toContain(2);
    expect(arms).toContain(4);
  });

  test("a payload under it is hit from below, before anything else", () => {
    const charge = payloadZone({
      name: "charge",
      mass_kg: 0.4,
      size_m: [0.12, 0.06],
      below_m: 0.035,
      lethal_m: 5,
    });
    const up = strikes([0, 0, 1], [0, 0, -1], never, [charge]);
    expect(up[0]?.part).toBe("payload");
    expect(up.map((s) => s.part)).toContain("battery");
    // Without one, the same shot meets the body first.
    expect(strikes([0, 0, 1], [0, 0, -1], never)[0]?.part).toBe("body");
  });

  test("a near miss is a miss", () => {
    expect(strikes([0, 0.3, -1], [0, 0.3, 1], always)).toEqual([]);
    expect(strikes([5, 0, -0.1], [5.4, 0, -0.1], always)).toEqual([]);
  });
});

describe("the levels' paths", () => {
  const hover: [number, number, number] = [10, 20, 30];
  const across: [number, number, number] = [1, 0, 0];
  const away: [number, number, number] = [0, 1, 0];

  for (const id of ["line", "circle", "eight"] as const) {
    test(`${id}: starts at the hover, and its velocity is its rate of change`, () => {
      expect(pathAt(id, 0, hover, across, away).at).toEqual(hover);
      for (const t of [0.5, 3, 4.2, 11]) {
        const a = pathAt(id, t, hover, across, away);
        const b = pathAt(id, t + 1e-4, hover, across, away);
        for (let i = 0; i < 3; i++) {
          expect((b.at[i]! - a.at[i]!) / 1e-4).toBeCloseTo(a.vel[i]!, 2);
        }
        expect(a.at[2]).toBe(30);
      }
    });
  }

  test("the figure eight crosses the hover point both ways at its top speed", () => {
    const top = Math.max(
      ...Array.from({ length: 400 }, (_, i) =>
        Math.hypot(...pathAt("eight", i * 0.1, hover, across, away).vel),
      ),
    );
    expect(top).toBeCloseTo(8, 1);
  });

  test("what each drone can fly", () => {
    const paths: LevelId[] = ["hover", "line", "circle", "eight"];
    expect(levelsFor("hover").map((l) => l.id)).toEqual(["hover"]);
    expect(levelsFor("targets").map((l) => l.id)).toEqual(paths);
    expect(levelsFor("mission").map((l) => l.id)).toEqual(paths);
  });

  test("a mission's waypoints: whole laps of the same shapes, as many as a mission takes", () => {
    for (const id of ["line", "circle", "eight"] as const) {
      const w = pathWaypoints(id, hover, across, away);
      expect(w.length).toBe(PATH_POINTS);
      for (const p of w) expect(p[2]).toBe(hover[2]);
      // Whole laps: the line ends at its far end, the circle and the eight back at the hover.
      const last = w[w.length - 1]!;
      if (id === "line") expect(last).toEqual([hover[0] - 25, hover[1], hover[2]]);
      else expect(Math.hypot(last[0] - hover[0], last[1] - hover[1])).toBeLessThan(1e-9);
    }
    // The line, end to end across the view; the circle on its circle, through the hover point.
    const line = pathWaypoints("line", hover, across, away);
    expect(line.slice(0, 3).map((p) => p[0] - hover[0])).toEqual([25, -25, 25]);
    for (const p of pathWaypoints("circle", hover, across, away)) {
      expect(Math.hypot(p[0] - hover[0], p[1] - (hover[1] + 20))).toBeCloseTo(20, 9);
    }
    // The eight crosses itself at the hover point, 25 m either side across, 12.5 away.
    const eight = pathWaypoints("eight", hover, across, away);
    const xs = eight.map((p) => p[0] - hover[0]);
    const ys = eight.map((p) => p[1] - hover[1]);
    expect(Math.max(...xs)).toBeLessThanOrEqual(25);
    expect(Math.min(...xs)).toBeGreaterThanOrEqual(-25);
    expect(Math.max(...ys.map(Math.abs))).toBeLessThanOrEqual(12.5);
  });
});

describe("recoil", () => {
  test("M193 from 11.5 inches: 5.5 N·s, 1.7 m/s free in a 3.2 kg rifle", () => {
    const j = recoilImpulse({ bulletKg: 0.003564, powderKg: 0.00168, muzzleMs: 853 });
    expect(j).toBeCloseTo(5.55, 1);
    expect(j / 3.2).toBeCloseTo(1.73, 1);
  });

  test("standing, the muzzle climbs most of a degree and settles within half a second", () => {
    const r = new Recoil(3.2, () => 0.5);
    r.kick(5.55);
    let peak = 0;
    let at = 0;
    for (let t = 0; t < 1; t += 0.001) {
      r.step(0.001);
      if (r.pitch > peak) {
        peak = r.pitch;
        at = t;
      }
    }
    expect(peak).toBeGreaterThan(0.006);
    expect(peak).toBeLessThan(0.03);
    expect(at).toBeLessThan(0.15);
    expect(Math.abs(r.pitch)).toBeLessThan(0.003);
  });

  test("prone climbs about half as far as standing (the elbows' short lever)", () => {
    const peak = (stance: "standing" | "prone") => {
      const r = new Recoil(3.2, () => 0.5);
      r.stance = stance;
      r.kick(5.55);
      let p = 0;
      for (let i = 0; i < 500; i++) {
        r.step(0.001);
        p = Math.max(p, r.pitch);
      }
      return p;
    };
    expect(peak("prone")).toBeLessThan(peak("standing") * 0.65);
  });
});

describe("the hold", () => {
  const spread = (hold: Hold, effort: "still" | "run", seconds: number, breath = false) => {
    let sum = 0;
    let n = 0;
    for (let t = 0; t < seconds; t += 0.01) {
      hold.step(0.01, effort, breath);
      if (t > seconds / 2) {
        sum += hold.yaw * hold.yaw;
        n++;
      }
    }
    return Math.sqrt(sum / n);
  };

  test("prone holds steadier than standing", () => {
    const standing = new Hold(mulberry(1));
    const prone = new Hold(mulberry(1));
    prone.stance = "prone";
    expect(spread(prone, "still", 30)).toBeLessThan(spread(standing, "still", 30) / 3);
  });

  test("running raises the heart, and the wobble with it", () => {
    const h = new Hold(mulberry(2));
    const calm = spread(h, "still", 20);
    spread(h, "run", 30);
    expect(h.heart).toBeGreaterThan(120);
    const after = spread(h, "still", 4);
    expect(after).toBeGreaterThan(calm);
  });

  test("slow frames (10 a second) leave the aim's wobble as small as fast ones", () => {
    // The heart's 4 Hz nudge, stepped a frame at a time, once grew without end under ~20 fps:
    // the view turned over and over on the sight.
    const fast = new Hold(mulberry(5));
    const slow = new Hold(mulberry(5));
    let worst = 0;
    for (let t = 0; t < 30; t += 0.1) {
      for (let i = 0; i < 10; i++) fast.step(0.01, "run", false);
      slow.step(0.1, "run", false);
      worst = Math.max(worst, Math.abs(slow.pitch), Math.abs(slow.yaw));
    }
    expect(worst).toBeLessThan(0.02);
    expect(Math.abs(slow.pitch - fast.pitch)).toBeLessThan(0.02);
  });

  test("a breath held stills the breathing, then runs out", () => {
    const h = new Hold(mulberry(3));
    for (let t = 0; t < 20; t += 0.01) h.step(0.01, "still", true);
    // Forced to breathe by 14 s, and not able to hold again straight away.
    expect(h.held).toBe(0);
    expect(h.canHold).toBe(false);
  });
});

describe("the air", () => {
  test("the log profile: the 20 ft wind at 20 ft, less below, none at the roughness", () => {
    expect(shear(6.096)).toBeCloseTo(1, 6);
    expect(shear(2)).toBeLessThan(1);
    expect(shear(0.04)).toBe(0);
  });

  test("the mean wind blows from where it says", () => {
    const air = new Air(
      {
        speed_ms: 5,
        from_deg: 270,
        turbulence: 0,
        eddy_scale: 1,
        gust_factor: 1,
        gust_every_s: 10,
      },
      mulberry(4),
    );
    air.step(0.1);
    const [e, n] = air.at(6.096);
    // From the west: blowing east.
    expect(e).toBeCloseTo(5, 6);
    expect(n).toBeCloseTo(0, 6);
    expect(air.meter(6.096).fromDeg).toBeCloseTo(270, 3);
  });
});

/** A seeded random number source, for repeatable tests. */
function mulberry(seed: number): () => number {
  let a = seed;
  return () => {
    a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}
