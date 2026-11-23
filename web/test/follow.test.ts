import { describe, expect, test } from "bun:test";
import { NoseFollow } from "../src/views/follow.ts";
import { quatOf } from "../src/world/geo.ts";

const deg = (d: number) => (d * Math.PI) / 180;

/** Fly `seconds` at 60 frames a second, the attitude given by time; the camera's heading after. */
function fly(f: NoseFollow, seconds: number, q: (t: number) => ReturnType<typeof quatOf>): number {
  let heading = 0;
  for (let t = 0; t < seconds; t += 1 / 60) heading = f.step(q(t), 1 / 60);
  return heading;
}

describe("the chase camera's heading", () => {
  test("starts behind the nose, at once", () => {
    const f = new NoseFollow();
    expect(f.step(quatOf(deg(140), 0, 0), 1 / 60)).toBeCloseTo(deg(140), 6);
  });

  test("stays behind the nose however the quad leans: sideways and backward flight", () => {
    const f = new NoseFollow();
    const lean = (t: number) => quatOf(deg(30), deg(40) * Math.sin(t * 3), deg(55) * Math.cos(t));
    expect(fly(f, 4, lean)).toBeCloseTo(deg(30), 6);
  });

  test("comes round to a new heading without overshoot, most of the way in a second", () => {
    const f = new NoseFollow();
    f.step(quatOf(0, 0, 0), 1 / 60);
    let most = 0;
    let atOne = 0;
    for (let i = 1; i <= 240; i++) {
      const h = f.step(quatOf(deg(90), 0, 0), 1 / 60);
      most = Math.max(most, h);
      if (i === 60) atOne = h;
    }
    expect(atOne).toBeGreaterThan(deg(75));
    expect(most).toBeLessThanOrEqual(deg(90) + 1e-9);
    expect(most).toBeCloseTo(deg(90), 3);
  });

  test("starts a turn gently: a tenth of the way after a tenth of a second, not a third", () => {
    const f = new NoseFollow();
    f.step(quatOf(0, 0, 0), 1 / 60);
    const h = fly(f, 0.1, () => quatOf(deg(90), 0, 0));
    expect(h).toBeLessThan(deg(9));
  });

  test("turns the short way through north", () => {
    const f = new NoseFollow();
    f.step(quatOf(deg(170), 0, 0), 1 / 60);
    const h = fly(f, 4, () => quatOf(deg(-170), 0, 0));
    expect(h).toBeCloseTo(deg(190), 3);
  });

  test("a spin faster than it follows is met from the nearer side, not chased round", () => {
    const f = new NoseFollow();
    f.step(quatOf(0, 0, 0), 1 / 60);
    // 600°/s for 0.7 s: the nose goes round once and 60° more.
    let turned = 0;
    let last = 0;
    for (let i = 1; i <= 240; i++) {
      const t = Math.min(i / 60, 0.7);
      const h = f.step(quatOf(deg(600 * t), 0, 0), 1 / 60);
      turned += Math.abs(h - last);
      last = h;
    }
    expect(Math.cos(last - deg(60))).toBeCloseTo(1, 4);
    expect(turned).toBeLessThan(deg(200));
  });

  test("a nose pointing up says nothing: the camera holds through the top of a flip", () => {
    const f = new NoseFollow();
    f.step(quatOf(deg(20), 0, 0), 1 / 60);
    expect(fly(f, 2, () => quatOf(deg(200), deg(85), 0))).toBeCloseTo(deg(20), 6);
  });
});
