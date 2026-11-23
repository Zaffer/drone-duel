// The session, in the page (DESIGN.md §18): what drone_duels.session does on the game's
// server, for the web version, which has no server. It takes the page's messages
// (net/protocol.ts: session, rc, ground, wind, video...) and answers as the server would
// (status, telemetry, osd, physics, log, and the true pose), so the page above it is the same
// page.
//
// One vehicle at a time, at a location's start pad: Betaflight's quad (vehicle.ts), stepped a
// millisecond at a time to keep up with the clock it is given (`tick`); or the truck, which has
// no firmware and whose physics is the page's own, so it is only ever "ready".
//
// Starting a quad: the module is fetched and compiled once; Betaflight's settings are typed
// into a fresh eeprom (once per video system, then kept); it boots on that, and its first
// seconds (the gyro's calibration, the boot's arming grace) are run through at once, not
// waited for. It is ready when it could arm on two looks half a second of its time apart, as
// the server waits.
//
// Nothing here touches the page or a worker's globals: worker.ts gives it the content, the
// module and a way out, and web/test/sim-session.test.ts flies it under bun.

import type {
  ContentMsg,
  Location,
  LogMsg,
  OsdMsg,
  PhysicsMsg,
  StatusMsg,
  TelemetryMsg,
} from "../net/protocol.ts";
import { configure, FirmwareExit, Sim } from "./module.ts";
import type { Airframe } from "./quad.ts";
import { BetaflightQuad, FRAME_S } from "./vehicle.ts";
import type { Wind } from "./weather.ts";

/** drone_duels.webdata.sim_data: per vehicle, what its simulator needs. */
export interface SimData {
  vehicles: Record<string, { airframe: string; settings: Record<"analog" | "digital", string> }>;
  airframes: Record<string, Airframe>;
}

/** The page's messages this session acts on (the rest of protocol.ts's it takes and drops). */
export type Incoming =
  | {
      ch: "session";
      action: "start" | "switch" | "reset" | "destroy" | "throw" | "stop";
      location?: string | null;
      vehicle?: string | null;
      pad?: number;
      /** A place not in the content: a GPS point typed in the menu. */
      place?: Location | null;
    }
  | { ch: "rc"; us: number[] }
  | { ch: "ground"; alt: number }
  | { ch: "wind"; wind: Wind; gust_now?: boolean }
  | { ch: "video"; system: "analog" | "digital" }
  | { ch: "target" }
  | { ch: "dynamics" | "radio_source" | "hit" };

export type Outgoing = StatusMsg | TelemetryMsg | OsdMsg | PhysicsMsg | LogMsg;

export interface Host {
  content: ContentMsg;
  sim: SimData;
  /** The simulator's module, compiled: asked for once, when the first quad starts. */
  module(): Promise<WebAssembly.Module>;
  send(msg: Outgoing): void;
  /** The true pose: protocol.POSE_FIELDS, 18 doubles. The array is the caller's to keep. */
  pose(frame: Float64Array): void;
  /** Wait a turn of the event loop (so a long start can be overtaken). */
  yield(): Promise<void>;
  /** The ground's height above sea level at a point, looked up (a place whose ground is
   * "auto": one the player typed in; the content's are numbers in the web version's data). */
  groundAlt(lat: number, lon: number): Promise<number>;
}

/** No sticks for this long, s: the radio link is lost. */
const RC_TIMEOUT_S = 0.2;
const POSE_HZ = 120;
/** Behind the clock by more than this, s (the tab hidden, a pause): carry on from now. */
const LATE_S = 0.1;
/** Starting: Betaflight's first seconds are run this many frames at a time between turns of
 * the event loop, for at most READY_S of its time. */
const WARM_FRAMES = 250;
const READY_S = 30;
/** Ready when it could arm on two looks this far apart, frames. */
const READY_LOOK_FRAMES = 500;

export class WebSession {
  private state: StatusMsg["state"] = "idle";
  private step = "";
  private location: string | null = null;
  /** Where it started: the content's location, or the player's own. */
  private place: Location | null = null;
  /** Ground heights looked up, by "lat,lon": a respawn asks once. */
  private readonly grounds = new Map<string, number>();
  private vehicle: string | null = null;
  private pad = 0;
  private video: "analog" | "digital" = "analog";
  private wind: Wind | null = null;
  private origin: [number, number, number] | null = null;
  private headingDeg = 0;
  private quad: BetaflightQuad | null = null;
  private readonly eeproms = new Map<string, Uint8Array>();
  /** Counts spawns: one overtaken by a later command stops at its next look. */
  private generation = 0;
  /** The clock (s) at which the quad's next frame is due, and its next pose. */
  private due = 0;
  private poseDue = 0;
  private lastRc = Number.NEGATIVE_INFINITY;
  private phase = "";

  constructor(private readonly host: Host) {}

  /** A message from the page, at clock time `now`, s. */
  handle(msg: Incoming, now: number): void {
    switch (msg.ch) {
      case "session":
        this.command(msg, now);
        break;
      case "rc":
        this.lastRc = now;
        if (this.state === "ready") this.quad?.rc(msg.us);
        break;
      case "ground":
        this.quad?.model.setGround(msg.alt);
        break;
      case "wind":
        this.wind = msg.wind;
        this.quad?.model.setWind(msg.wind, msg.gust_now ?? false);
        break;
      case "video":
        this.setVideo(msg.system, now);
        break;
      case "target":
        this.log("warn", "the web version's quad does not fly itself");
        break;
      default:
        break; // dynamics, the server's radio, hits: not in the web version
    }
  }

  /** Run the quad up to clock time `now`, s. */
  tick(now: number): void {
    const quad = this.quad;
    if (!quad || this.state !== "ready") return;
    if (now - this.lastRc > RC_TIMEOUT_S) quad.rcLost();
    if (!Number.isFinite(this.due)) {
      this.due = this.poseDue = now;
    } else if (now - this.due > LATE_S) {
      this.poseDue += now - this.due;
      this.due = now;
    }
    try {
      while (this.due <= now) {
        quad.frame();
        this.due += FRAME_S;
        if (this.due >= this.poseDue) {
          this.poseDue += 1 / POSE_HZ;
          const frame = new Float64Array(18);
          if (quad.model.pose(frame)) this.host.pose(frame);
        }
      }
    } catch (err) {
      this.fail(err instanceof FirmwareExit ? `Betaflight stopped (${err.code})` : String(err));
      return;
    }
    if (quad.phase !== this.phase) this.physics(quad.phase);
  }

  private command(msg: Extract<Incoming, { ch: "session" }>, now: number): void {
    switch (msg.action) {
      case "start": {
        const location = msg.place ?? this.host.content.locations[msg.location ?? ""];
        if (!location) {
          this.fail(`no location ${msg.location}`);
          return;
        }
        this.location = location.id;
        this.place = location;
        this.pad = msg.pad ?? 0;
        this.wind = location.wind;
        void this.spawn(msg.vehicle ?? "", now);
        break;
      }
      case "switch":
        if (this.location === null) {
          this.fail("switch: nothing started");
          return;
        }
        void this.spawn(msg.vehicle ?? this.vehicle ?? "", now);
        break;
      case "reset":
        if (this.state === "ready" && this.quad) {
          this.quad.reset();
          // Still ready, but said again: a page waiting on the reset hears that it is done.
          this.status();
        } else if (this.vehicle !== null) {
          void this.spawn(this.vehicle, now);
        }
        break;
      case "destroy":
        this.teardown();
        this.state = "destroyed";
        this.step = "destroyed";
        this.status();
        break;
      case "stop":
        this.teardown();
        this.state = "idle";
        this.step = "";
        this.location = this.vehicle = this.origin = null;
        this.place = null;
        this.status();
        break;
      default:
        break; // throw: no hand-launched wing here
    }
  }

  /** The OSD's grid follows the video system, and Betaflight takes it at boot: the vehicle
   * respawns at its start. */
  private setVideo(system: "analog" | "digital", now: number): void {
    if (system === this.video) return;
    this.video = system;
    if (this.vehicle && ["ready", "destroyed", "failed"].includes(this.state)) {
      void this.spawn(this.vehicle, now);
    } else {
      this.status();
    }
  }

  private async spawn(vehicleId: string, _now: number): Promise<void> {
    this.teardown();
    const mine = ++this.generation;
    const { content, sim } = this.host;
    const vehicle = content.vehicles[vehicleId];
    const location = this.place;
    if (!vehicle || !location) return this.fail(`no vehicle ${vehicleId}`);
    const pad = location.pads[Math.min(this.pad, location.pads.length - 1)]!;
    let ground = location.ground_alt_m;
    if (typeof ground !== "number") {
      const [lat, lon] = location.origin;
      const key = `${lat.toFixed(6)},${lon.toFixed(6)}`;
      const known = this.grounds.get(key);
      if (known === undefined) {
        this.state = "starting";
        this.setStep("finding the ground");
        try {
          ground = await this.host.groundAlt(lat, lon);
        } catch (err) {
          if (mine === this.generation) {
            this.fail(
              `ground height of ${location.name}: ${err instanceof Error ? err.message : String(err)}`,
            );
          }
          return;
        }
        if (mine !== this.generation) return;
        this.grounds.set(key, ground);
      } else {
        ground = known;
      }
    }
    this.vehicle = vehicle.id;
    this.origin = [pad.at[0], pad.at[1], ground];
    this.headingDeg = pad.heading_deg;
    this.state = "starting";
    if (vehicle.firmware.kind === "none") {
      // The truck: its physics is the page's own.
      this.state = "ready";
      this.step = "";
      this.status();
      this.log("info", `${vehicle.name} ready`);
      return;
    }
    const data = sim.vehicles[vehicle.id];
    const airframe = data ? sim.airframes[data.airframe] : undefined;
    if (vehicle.firmware.kind !== "betaflight" || !data || !airframe) {
      return this.fail(`${vehicle.name} does not fly in the web version`);
    }
    let ready: BetaflightQuad;
    try {
      this.setStep("loading the simulator");
      const module = await this.host.module();
      if (mine !== this.generation) return;
      const video = this.video;
      const settings = data.settings[video];
      let eeprom = this.eeproms.get(settings);
      if (!eeprom) {
        this.setStep("typing Betaflight's settings in");
        await this.host.yield();
        if (mine !== this.generation) return;
        eeprom = configure(module, settings, (line) => this.firmwareSays(line));
        this.eeproms.set(settings, eeprom);
      }
      this.setStep("starting Betaflight");
      const instance = new Sim(module, (line) => this.firmwareSays(line));
      instance.boot(eeprom);
      const quad: BetaflightQuad = new BetaflightQuad(
        instance,
        airframe,
        this.origin,
        this.headingDeg,
        this.wind ?? location.wind,
        video,
        {
          telemetry: (m) => this.state === "ready" && this.quad === quad && this.host.send(m),
          osd: (m) => this.quad === quad && this.host.send(m),
        },
      );
      // Its first seconds, at once: ready when it could arm on two looks in a row.
      let clear = 0;
      let said = "";
      while (clear < 2) {
        for (let i = 0; i < WARM_FRAMES; i++) quad.frame();
        if (quad.frames % READY_LOOK_FRAMES === 0) {
          const blocked = quad.blocked;
          const ready = !quad.armed && blocked.length === 0;
          clear = ready ? clear + 1 : 0;
          const what = `Betaflight: ${blocked.join(", ") || "armed"}`;
          if (!ready && what !== said) {
            said = what;
            this.setStep(what);
          }
        }
        if (quad.frames * FRAME_S > READY_S) {
          throw new Error(`Betaflight is not ready to arm in ${READY_S} s: ${said}`);
        }
        await this.host.yield();
        if (mine !== this.generation) return;
      }
      // The poses' clock starts here, with the clock the page keeps.
      quad.model.t = 0;
      this.quad = ready = quad;
      this.due = this.poseDue = Number.NEGATIVE_INFINITY;
      this.lastRc = Number.NEGATIVE_INFINITY;
    } catch (err) {
      if (mine === this.generation) this.fail(err instanceof Error ? err.message : String(err));
      return;
    }
    this.state = "ready";
    this.step = "";
    this.status();
    this.physics(ready.phase);
    this.log("info", `${vehicle.name} ready`);
  }

  private teardown(): void {
    this.generation++;
    this.quad = null;
    this.phase = "";
  }

  private setStep(step: string): void {
    this.step = step;
    this.status();
  }

  private fail(why: string): void {
    this.teardown();
    this.state = "failed";
    this.step = why;
    this.status();
    this.log("error", why);
  }

  private status(): void {
    this.host.send({
      ch: "status",
      state: this.state,
      location: this.location,
      vehicle: this.vehicle,
      pad: this.pad,
      step: this.step,
      video: this.video,
      origin: this.origin,
      heading_deg: this.headingDeg,
      payload: null,
    });
  }

  private physics(phase: string): void {
    this.phase = phase;
    this.host.send({ ch: "physics", phase, connected: true, damage: null });
  }

  private log(level: LogMsg["level"], text: string): void {
    this.host.send({ ch: "log", level, source: "session", text });
  }

  /** A line the firmware printed: only what went wrong is worth the page's log. */
  private firmwareSays(line: string): void {
    if (line.includes("###ERROR") || line.includes("failureMode")) {
      this.host.send({ ch: "log", level: "warn", source: "betaflight", text: line });
    }
  }
}
