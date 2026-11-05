import { describe, expect, test } from "bun:test";
import { MAX_AHEAD_S, type Sample, SLEW, slerp, Timeline, turn } from "../src/vehicles/timeline.ts";
import { moveBy, neFrom, type Quat, quatOf } from "../src/world/geo.ts";
import arrivals from "./fixtures/pose-arrivals.json";

const LAT = 43.63;
const LON = -79.39;
const G = 9.80665;

/** A coordinated 3 g turn at 30 m/s, level at 100 m: the truth at time t, s. */
function turning(t: number): Sample {
  const V = 30;
  const w = (Math.sqrt(8) * G) / V; // 3 g: the turn's own acceleration is √8 g
  const r = V / w;
  const roll = Math.atan((V * w) / G);
  const [lat, lon] = moveBy(LAT, LON, r * Math.sin(w * t), r * (1 - Math.cos(w * t)));
  return {
    t,
    lat,
    lon,
    alt: 100,
    q: quatOf(w * t, 0, roll),
    v: [V * Math.cos(w * t), V * Math.sin(w * t), 0],
    rates: [0, w * Math.sin(roll), w * Math.cos(roll)],
  };
}

/** Metres between two samples. */
function apart(a: Sample, b: Sample): number {
  const [n, e] = neFrom(a.lat, a.lon, b.lat, b.lon);
  return Math.hypot(n, e, a.alt - b.alt);
}

/** The angle between two attitudes, rad. */
function angle(a: Quat, b: Quat): number {
  const dot = Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]);
  return 2 * Math.acos(Math.min(1, dot));
}

/** A seeded random number generator, 0..1. */
function random(seed: number): () => number {
  let s = seed;
  return () => {
    s = (s * 1664525 + 1013904223) % 4294967296;
    return s / 4294967296;
  };
}

const quantile = (xs: number[], q: number): number =>
  [...xs].sort((a, b) => a - b)[Math.floor(q * (xs.length - 1))]!;

describe("our own aircraft, from the poses as the page really got them", () => {
  // The sim times and arrival times recorded in Chrome at 26 fps: 81% of poses handled in clumps
  // between frames, up to 100 ms late. The aircraft flies a 3 g turn along them.
  const poses = arrivals.poses as [number, number][];
  const frames = arrivals.frames as number[];
  const tl = new Timeline<Sample>();
  const drawn: { at: number; t: number; newest: number; s: Sample }[] = [];
  let next = 0;
  for (const f of frames) {
    while (next < poses.length && poses[next]![1] <= f) {
      const [t, at] = poses[next++]!;
      tl.push(turning(t / 1000), at);
    }
    const s = tl.sample(f);
    if (s) drawn.push({ at: f, t: tl.timeAt(f)!, newest: tl.newest!.t, s });
  }
  const settled = drawn.filter((d) => d.at > 2000);
  const best = Math.min(...poses.map(([t, at]) => at - t));

  test("it is drawn where it was at the time drawn, but for the turn while carried on", () => {
    // Carried on h s past the newest pose along its velocity, a 3 g turn leaves it ½ a h²
    // outside; held past MAX_AHEAD_S (the page stalled), it is behind by the rest at 30 m/s;
    // between poses it is on the path.
    const a = Math.sqrt(8) * G;
    let held = 0;
    for (const d of settled) {
      const ahead = Math.max(0, d.t - d.newest);
      const h = Math.min(ahead, MAX_AHEAD_S);
      const behind = 30 * (ahead - h);
      if (behind > 0) held++;
      expect(apart(d.s, turning(d.t))).toBeLessThan(0.5 * a * h * h + behind + 0.005);
      if (behind === 0) expect(angle(d.s.q, turning(d.t).q)).toBeLessThan(1e-4);
    }
    expect(held / settled.length).toBeLessThan(0.02);
  });

  test("its time runs with the page's, never faster or slower than the slew", () => {
    for (let i = 1; i < settled.length; i++) {
      const a = settled[i - 1]!;
      const b = settled[i]!;
      const rate = (b.t - a.t) / ((b.at - a.at) / 1000);
      expect(rate).toBeGreaterThan(1 - SLEW - 1e-6);
      expect(rate).toBeLessThan(1 + SLEW + 1e-6);
    }
  });

  test("it never steps back along its path", () => {
    for (let i = 1; i < settled.length; i++) {
      const a = settled[i - 1]!.s;
      const b = settled[i]!.s;
      const [n, e] = neFrom(a.lat, a.lon, b.lat, b.lon);
      expect(n * a.v[0] + e * a.v[1]).toBeGreaterThan(0);
    }
  });

  test("it is drawn close to now: at most a few frames behind the quickest delivery", () => {
    const lags = settled.map((d) => d.at - best - d.t * 1000);
    expect(quantile(lags, 0.5)).toBeLessThan(10);
    expect(Math.max(...lags)).toBeLessThan(50);
  });
});

describe("the timeline", () => {
  test("starts again when the samples' clock goes back (a respawn)", () => {
    const tl = new Timeline<Sample>();
    for (let i = 0; i <= 600; i++) tl.push(turning(30 + i / 120), 1000 + (i * 1000) / 120);
    tl.push(turning(0), 6100);
    expect(tl.newest?.t).toBe(0);
    expect(tl.timeAt(6100)).toBeCloseTo(0, 6);
  });

  test("follows a clock that slipped (the physics stalled) at once", () => {
    const tl = new Timeline<Sample>();
    for (let i = 0; i <= 240; i++) tl.push(turning(i / 120), 1000 + (i * 1000) / 120);
    // A 0.3 s stall: the next sample is 0.3 s late, and so is every one after it.
    for (let i = 241; i <= 300; i++) tl.push(turning(i / 120), 1300 + (i * 1000) / 120);
    expect(tl.ahead(1300 + 2500)).toBeCloseTo(0, 2);
  });

  test("keeps its clock through one late clump (the page stalled), so the vehicle never steps back", () => {
    const tl = new Timeline<Sample>();
    const at = (i: number) => 1000 + (i * 1000) / 120;
    for (let i = 0; i <= 360; i++) tl.push(turning(i / 120), at(i));
    const before = tl.timeAt(at(360))!;
    // The page's thread held for 200 ms: the 24 poses sent meanwhile handled at once at its end,
    // then on as before.
    for (let i = 361; i <= 384; i++) tl.push(turning(i / 120), at(384));
    for (let i = 385; i <= 480; i++) tl.push(turning(i / 120), at(i));
    let last = before;
    for (let now = at(360); now <= at(480); now += 1000 / 60) {
      const t = tl.timeAt(now)!;
      expect(t).toBeGreaterThanOrEqual(last);
      last = t;
    }
    expect(tl.timeAt(at(480))).toBeCloseTo(4, 2);
  });

  test("carries on past the newest sample for at most MAX_AHEAD_S, then holds", () => {
    const tl = new Timeline<Sample>();
    for (let i = 0; i <= 120; i++) tl.push(turning(i / 120), 1000 + (i * 1000) / 120);
    const last = tl.newest!;
    const soon = tl.sample(2000 + 50)!;
    expect(apart(soon, turning(1.05))).toBeLessThan(0.05);
    const later = tl.sample(2000 + 400)!;
    expect(later.t).toBeCloseTo(last.t + MAX_AHEAD_S, 6);
  });

  test("keeps a peer's poses in order when they come out of it, and a pose twice once", () => {
    const tl = new Timeline<Sample>();
    for (const i of [0, 1, 3, 2, 4, 4, 5]) tl.push(turning(i / 30), 1000 + (i * 1000) / 30);
    expect(tl.newest?.t).toBeCloseTo(5 / 30, 9);
    const s = tl.sample(1000 + (2.5 * 1000) / 30, 0)!;
    expect(s.t).toBeCloseTo(2.5 / 30, 9);
    expect(apart(s, turning(2.5 / 30))).toBeLessThan(0.005);
  });
});

describe("the other player, over the internet", () => {
  test("drawn a buffer behind, it stays between poses as they come late and get lost", () => {
    // 30 Hz, each 20 to 120 ms on the way (a long tail), one in ten lost.
    const rnd = random(7);
    const tl = new Timeline<Sample>();
    const sent: [number, number][] = [];
    for (let i = 0; i < 30 * 20; i++) {
      if (rnd() < 0.1) continue;
      const delay = 20 + 40 * rnd() + (rnd() < 0.1 ? 60 * rnd() : 0);
      sent.push([i / 30, 5000 + (i / 30) * 1000 + delay]);
    }
    sent.sort((a, b) => a[1] - b[1]);
    let next = 0;
    let between = 0;
    let frames = 0;
    let lastT = 0;
    let lastAt = 0;
    const buffers: number[] = [];
    for (let at = 5000; at < 5000 + 20_000; at += 1000 / 60) {
      while (next < sent.length && sent[next]![1] <= at) {
        const [t, arrived] = sent[next++]!;
        tl.push(turning(t), arrived);
      }
      const buffer = tl.buffer(at);
      const s = tl.sample(at, buffer);
      if (!s || at < 8000) continue;
      frames++;
      buffers.push(buffer);
      if (tl.ahead(at, buffer) <= 0) between++;
      if (lastAt) {
        const rate = (s.t - lastT) / ((at - lastAt) / 1000);
        expect(rate).toBeGreaterThan(0.85);
        expect(rate).toBeLessThan(1.15);
      }
      lastT = s.t;
      lastAt = at;
      expect(apart(s, turning(s.t))).toBeLessThan(0.3);
    }
    expect(between / frames).toBeGreaterThan(0.95);
    expect(Math.min(...buffers)).toBeGreaterThanOrEqual(0.06);
    expect(Math.max(...buffers)).toBeLessThanOrEqual(0.25);
  });
});

describe("attitude", () => {
  test("turned by the body rates, as the turn really goes", () => {
    const a = turning(1);
    expect(angle(turn(a.q, a.rates!, 0.05), turning(1.05).q)).toBeLessThan(1e-6);
  });

  test("slerp: its ends, and halfway on the short way round", () => {
    const a = quatOf(0.1, 0, 0);
    const b = quatOf(0.5, 0, 0);
    expect(angle(slerp(a, b, 0), a)).toBeLessThan(1e-6);
    expect(angle(slerp(a, b, 1), b)).toBeLessThan(1e-6);
    expect(angle(slerp(a, b, 0.5), quatOf(0.3, 0, 0))).toBeLessThan(1e-6);
    const minusB = b.map((x) => -x) as Quat;
    expect(angle(slerp(a, minusB, 0.5), quatOf(0.3, 0, 0))).toBeLessThan(1e-6);
  });
});
