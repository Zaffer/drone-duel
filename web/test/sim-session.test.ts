// The web version's session, whole (src/sim/session.ts): real Betaflight and the quad's plant,
// the simulator's module, flown through the page's own messages as the game's server's session
// is in tests/test_sitl_betaflight.py. Time is the test's, so a minute of flight takes a
// second. Needs the simulator built (scripts/setup-web.sh); skipped without it.

import { beforeAll, describe, expect, test } from "bun:test";
import type { ContentMsg, Location, OsdMsg, StatusMsg, TelemetryMsg } from "../src/net/protocol.ts";
import { type Outgoing, type SimData, WebSession } from "../src/sim/session.ts";
import { gpsPlace } from "../src/world/places.ts";
import data from "./fixtures/sim-data.json";
import { simModule } from "./sim-module.ts";

const module = await simModule();
const QUAD = "quad5-betaflight";
const PLACE = "billy-bishop";
// The radio: AETR, the arm switch on channel 5, the mode switch on 6 (low POSHOLD, middle
// ANGLE, high ACRO), as content/vehicles/quad5-betaflight.toml.
const ARM = 4;
const MODE = 5;

class Game {
  now = 0;
  readonly out: Outgoing[] = [];
  pose: Float64Array | null = null;
  poses = 0;
  readonly rc = [1500, 1500, 1000, 1500, 1000, 1500, 1000, 1000];
  /** The ground heights the session looked up (a GPS point's), and what the lookup answers. */
  readonly groundAsks: [number, number][] = [];
  ground: number | Error = 63.5;
  readonly session = new WebSession({
    content: data.content as unknown as ContentMsg,
    sim: data.sim as SimData,
    module: async () => module!,
    send: (m) => this.out.push(m),
    pose: (f) => {
      this.pose = f;
      this.poses++;
    },
    yield: async () => {},
    groundAlt: async (lat, lon) => {
      this.groundAsks.push([lat, lon]);
      if (this.ground instanceof Error) throw this.ground;
      return this.ground;
    },
  });

  last<C extends Outgoing["ch"]>(ch: C): Extract<Outgoing, { ch: C }> | undefined {
    return this.out.findLast((m) => m.ch === ch) as Extract<Outgoing, { ch: C }> | undefined;
  }

  get status(): StatusMsg {
    return this.last("status")!;
  }

  get telemetry(): TelemetryMsg {
    return this.last("telemetry")!;
  }

  get osd(): OsdMsg {
    return this.last("osd")!;
  }

  /** The height above the pad's ground, m, from the true pose. */
  get height(): number {
    return this.pose![3]! - this.status.origin![2];
  }

  async start(
    vehicle = QUAD,
    video: "analog" | "digital" = "analog",
    place: Location | null = null,
  ): Promise<void> {
    this.session.handle({ ch: "video", system: video }, this.now);
    this.session.handle(
      { ch: "session", action: "start", location: place?.id ?? PLACE, vehicle, pad: 0, place },
      this.now,
    );
    await this.settle();
  }

  /** Let a start (a chain of promises) run to its end. */
  async settle(): Promise<void> {
    for (let i = 0; i < 200 && this.status.state === "starting"; i++) await Promise.resolve();
    for (let i = 0; i < 50; i++) await Promise.resolve();
  }

  /** `seconds` of the game: sticks at 250 Hz (or none: the radio gone quiet). */
  fly(seconds: number, sticks = true): void {
    for (let i = 0; i < seconds * 250; i++) {
      this.now += 0.004;
      if (sticks) this.session.handle({ ch: "rc", us: this.rc }, this.now);
      this.session.tick(this.now);
    }
  }

  /** The OSD's row `row` as text (glyphs outside ASCII as "?"). */
  row(row: number): string {
    const { cols, codes } = this.osd;
    return codes
      .slice(row * cols, (row + 1) * cols)
      .map((c) => (c >= 32 && c < 127 ? String.fromCharCode(c) : "?"))
      .join("");
  }

  get screen(): string {
    return Array.from({ length: this.osd.rows }, (_, r) => this.row(r)).join("\n");
  }
}

describe.skipIf(!module)("the web version's session", () => {
  let game: Game;

  beforeAll(async () => {
    game = new Game();
    await game.start();
  });

  test("starts Betaflight's quad on its pad, ready to arm, saying each step", () => {
    const steps = game.out.filter((m) => m.ch === "status").map((m) => (m as StatusMsg).step);
    expect(steps).toContain("typing Betaflight's settings in");
    expect(steps).toContain("starting Betaflight");
    const s = game.status;
    expect(s.state).toBe("ready");
    expect(s.vehicle).toBe(QUAD);
    expect(s.origin).toEqual([43.63093, -79.38945, 77]);
    expect(s.heading_deg).toBe(270);
    expect(game.last("physics")).toEqual({
      ch: "physics",
      phase: "ground",
      connected: true,
      damage: null,
    });
    expect(game.out.some((m) => m.ch === "log" && m.level === "error")).toBe(false);
  });

  test("on the pad: the true pose at 120 Hz, telemetry at 10 Hz, the OSD on NTSC's grid", () => {
    const before = { poses: game.poses, n: game.out.length };
    game.fly(2);
    expect(game.poses - before.poses).toBeGreaterThan(235);
    expect(game.poses - before.poses).toBeLessThan(245);
    const since = game.out.slice(before.n);
    expect(since.filter((m) => m.ch === "telemetry").length).toBeGreaterThanOrEqual(18);
    const pose = game.pose!;
    expect(pose[1]).toBeCloseTo(43.63093, 6);
    expect(pose[2]).toBeCloseTo(-79.38945, 6);
    expect(game.height).toBeLessThan(0.05);
    // Facing west, level: q (NED from FRD) is a quarter turn about down.
    expect(Math.abs(pose[7]!)).toBeCloseTo(Math.SQRT1_2, 3);
    const t = game.telemetry;
    expect(t.armed).toBe(false);
    expect(t.blockers).toEqual([]);
    expect(t.gps_sats).toBe(12);
    expect(t.lat).toBeCloseTo(43.63093, 5);
    expect(t.lon).toBeCloseTo(-79.38945, 5);
    expect(Math.abs(t.yaw_deg - 270)).toBeLessThan(5);
    expect([game.osd.cols, game.osd.rows, game.osd.font]).toEqual([30, 13, "betaflight"]);
    expect(game.screen).toContain("DISARMED");
  });

  test("arms on its switch, takes off in ANGLE and holds level; telemetry follows the truth", () => {
    // POSHOLD's switch: Betaflight will not arm in it, and says why.
    game.rc[MODE] = 1000;
    game.fly(1);
    expect(game.telemetry.blockers).toContain("altitude hold switch");
    expect(game.telemetry.blockers).toContain("position hold switch");
    game.rc[MODE] = 1500;
    game.fly(1);
    expect(game.telemetry.blockers).toEqual([]);
    game.rc[ARM] = 2000;
    game.fly(1);
    expect(game.telemetry.armed).toBe(true);
    expect(game.telemetry.mode).toBe("ANGLE");
    expect(game.height).toBeLessThan(0.1);
    game.rc[2] = 1450;
    game.fly(2);
    expect(game.last("physics")!.phase).toBe("flying");
    expect(game.height).toBeGreaterThan(3);
    // About the hover throttle (ap_hover_throttle in quad5.cli), it stops climbing.
    game.rc[2] = 1280;
    game.fly(4);
    const height = game.height;
    expect(height).toBeGreaterThan(8);
    expect(Math.abs(game.pose![10]!)).toBeLessThan(1.5);
    const t = game.telemetry;
    expect(Math.abs(t.roll_deg)).toBeLessThan(3);
    expect(Math.abs(t.pitch_deg)).toBeLessThan(3);
    // Betaflight's altitude is from its barometer and GPS, zeroed as it armed.
    expect(Math.abs(t.alt_rel! - height)).toBeLessThan(1.5);
    expect(t.throttle_pct).toBeGreaterThan(15);
  });

  test("the sticks fly it: forward is nose down and on along its heading", () => {
    const north = game.pose![8]!;
    const east = game.pose![9]!;
    game.rc[1] = 1700;
    game.fly(1.5);
    expect(game.telemetry.pitch_deg).toBeLessThan(-5);
    // Facing west: faster west (or slower east, in this wind from the west).
    expect(game.pose![9]!).toBeLessThan(east - 1.5);
    expect(Math.abs(game.pose![8]! - north)).toBeLessThan(1.5);
    game.rc[1] = 1500;
    game.rc[0] = 1700;
    game.fly(1);
    expect(game.telemetry.roll_deg).toBeGreaterThan(5);
    game.rc[0] = 1500;
    game.rc[3] = 1700;
    game.fly(1);
    game.rc[3] = 1500;
    game.fly(0.5);
    // Yaw stick right: the heading comes round clockwise, from west through north.
    const turned = (game.telemetry.yaw_deg - 270 + 360) % 360;
    expect(turned).toBeGreaterThan(20);
    expect(turned).toBeLessThan(180);
  });

  test("POSHOLD, the sticks let go: it stops where it is, in the place's wind", () => {
    game.rc[2] = 1275;
    game.rc[MODE] = 1000;
    for (let i = 0; i < 12 && game.telemetry.mode !== "POSHOLD"; i++) game.fly(0.25);
    expect(game.telemetry.mode).toBe("POSHOLD");
    game.fly(4); // it brakes
    const held = [game.pose![1]!, game.pose![2]!, game.pose![3]!];
    game.fly(4);
    const north = (game.pose![1]! - held[0]!) * 111_320;
    const east = (game.pose![2]! - held[1]!) * 111_320 * Math.cos((43.63 * Math.PI) / 180);
    expect(Math.hypot(north, east)).toBeLessThan(1.5);
    expect(Math.abs(game.pose![3]! - held[2]!)).toBeLessThan(1.5);
    game.rc[MODE] = 1500;
    game.fly(0.5);
  });

  test("the radio gone quiet: Betaflight's own failsafe lands it and disarms", () => {
    for (let i = 0; i < 20 && game.telemetry.mode !== "FAILSAFE"; i++) game.fly(0.25, false);
    expect(game.telemetry.mode).toBe("FAILSAFE");
    for (let i = 0; i < 40 && game.telemetry.armed; i++) game.fly(1, false);
    expect(game.telemetry.armed).toBe(false);
    expect(game.last("physics")!.phase).not.toBe("flying");
    expect(game.height).toBeLessThan(1);
  });

  test("a reset puts it back on its pad, and the page hears it is done", () => {
    game.rc[ARM] = 1000;
    game.rc[2] = 1000;
    game.fly(0.5);
    const n = game.out.length;
    game.session.handle({ ch: "session", action: "reset" }, game.now);
    expect(game.out.slice(n).some((m) => m.ch === "status" && m.state === "ready")).toBe(true);
    game.fly(0.5);
    expect(game.pose![1]).toBeCloseTo(43.63093, 6);
    expect(game.pose![2]).toBeCloseTo(-79.38945, 6);
    expect(game.height).toBeLessThan(0.05);
    expect(game.last("physics")!.phase).toBe("ground");
  });

  test("another video system respawns it with that system's OSD grid", async () => {
    game.session.handle({ ch: "video", system: "digital" }, game.now);
    expect(game.status.state).toBe("starting");
    await game.settle();
    expect(game.status.state).toBe("ready");
    expect(game.status.video).toBe("digital");
    game.fly(1);
    expect([game.osd.cols, game.osd.rows]).toEqual([53, 20]);
    expect(game.screen).toContain("DISARMED");
  });

  test("the ground from the map moves it while it sits on it; a tab asleep is not caught up", () => {
    game.fly(0.5);
    const alt = game.pose![3]!;
    game.session.handle({ ch: "ground", alt: 79 }, game.now);
    game.fly(0.5);
    expect(game.pose![3]! - alt).toBeCloseTo(2, 2);
    const poses = game.poses;
    game.now += 30;
    game.fly(0.1);
    expect(game.poses - poses).toBeLessThan(20);
  });

  test("the truck has no firmware: ready at once, at the pad", async () => {
    const other = new Game();
    await other.start("truck");
    expect(other.status.state).toBe("ready");
    expect(other.status.vehicle).toBe("truck");
    expect(other.status.origin).toEqual([43.63093, -79.38945, 77]);
    other.fly(0.2);
    expect(other.poses).toBe(0);
  });

  test("a vehicle that is not in the web version fails, saying so; a stop is idle", async () => {
    const other = new Game();
    await other.start("chupito");
    expect(other.status.state).toBe("failed");
    expect(other.out.some((m) => m.ch === "log" && m.level === "error")).toBe(true);
    other.session.handle({ ch: "session", action: "stop" }, other.now);
    expect(other.status.state).toBe("idle");
    expect(other.status.origin).toBeNull();
  });
});

// Needs no simulator: the truck has no firmware.
describe("a place of the player's own (a GPS point)", () => {
  const place = gpsPlace(51.5007, -0.1246);

  test("starts there, its ground looked up once, and again after a respawn", async () => {
    const game = new Game();
    await game.start("truck", "analog", place);
    expect(game.status.state).toBe("ready");
    expect(game.status.location).toBe(place.id);
    expect(game.status.origin).toEqual([51.5007, -0.1246, 63.5]);
    expect(game.out.some((m) => m.ch === "status" && m.step === "finding the ground")).toBe(true);
    game.session.handle({ ch: "session", action: "reset" }, game.now);
    game.session.handle({ ch: "session", action: "switch", vehicle: "truck" }, game.now);
    await game.settle();
    expect(game.status.origin).toEqual([51.5007, -0.1246, 63.5]);
    expect(game.groundAsks).toEqual([[51.5007, -0.1246]]);
  });

  test("player 2 starts on its own pad", async () => {
    const game = new Game();
    game.session.handle(
      { ch: "session", action: "start", location: place.id, vehicle: "truck", pad: 1, place },
      game.now,
    );
    await game.settle();
    expect(game.status.origin?.slice(0, 2)).toEqual(place.pads[1]!.at);
  });

  test("a ground it cannot look up fails the start, saying so", async () => {
    const game = new Game();
    game.ground = new Error("offline");
    await game.start("truck", "analog", place);
    expect(game.status.state).toBe("failed");
    expect(game.status.step).toContain("offline");
  });
});
