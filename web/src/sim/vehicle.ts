// Betaflight's 5" quad, whole, in the page: one instance of the simulator's module (its flight
// controller and its plant, module.ts), the model around the plant (quad.ts), and the things a
// real quad has plugged into its flight controller: a receiver, a ground station on MSP
// (UART2), an air unit taking the OSD over MSP DisplayPort (UART3), as the vehicle's CLI
// settings lay them out (firmware/betaflight/quad5.cli).
//
// `frame` is one millisecond of both, in lockstep: the model steps with the motors Betaflight
// last wrote, its sensors are set from the true state (drone_duels.bridges.betaflight's
// conventions, in bfweb.c), and Betaflight runs a millisecond of its own clock. The game's
// server paces its physics against the wall clock and Betaflight follows; here neither can
// run without the other, so the firmware's time is the physics' exactly.
//
// The receiver (drone_duels.firmware.betaflight's): until the pilot's sticks come, one sitting
// idle is played (throttle low, arm switch low, the rest centred) at 50 Hz; then only the
// pilot's. When they stop (`rcLost`) nothing more is fed: Betaflight's receiver loses its
// link and its own failsafe flies.

import type { OsdMsg, TelemetryMsg } from "../net/protocol.ts";
import type { Sim } from "./module.ts";
import {
  ARM_SWITCH_BIT,
  blockers,
  DisplayPort,
  MSP_BOXIDS,
  MSP_DISPLAYPORT,
  MSP_FC_VARIANT,
  MSP_STATUS,
  MspLink,
  MspReader,
  OSD_GRIDS,
  requestV1,
  TELEMETRY_ASKS,
  telemetry,
} from "./msp.ts";
import { type Airframe, AP_FROM_BF, type Fdm, type Phase, QuadModel } from "./quad.ts";
import type { Wind } from "./weather.ts";

/** One frame, s: Betaflight's gyro updates at this rate. */
export const FRAME_S = 0.001;
const FRAME_US = 1000;
const CHANNELS = 16;
/** A receiver linked but its sticks let go. */
export const IDLE_US: readonly number[] = [
  1500, 1500, 1000, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
];
/** Frames between the idle receiver's (50 Hz). */
const RC_IDLE_FRAMES = 20;
const MSP_UART = 2;
const OSD_UART = 3;
/** Frames between reads of the UARTs. */
const SERIAL_FRAMES = 5;
/** Frames between telemetry rounds (10 Hz), and between the air unit's polls. */
const TELEMETRY_FRAMES = 100;
const OSD_POLL_FRAMES = 100;
/** Frames between OSD screens passed on, at least (20 Hz). */
const OSD_GAP_FRAMES = 50;
const STOPPED_US = 1000;
/** The air unit's poll: Betaflight sends its OSD only to one it hears. */
const OSD_POLL = new Uint8Array([...requestV1(MSP_FC_VARIANT), ...requestV1(MSP_STATUS)]);

export interface QuadEvents {
  telemetry(msg: TelemetryMsg): void;
  osd(msg: OsdMsg): void;
}

export class BetaflightQuad {
  readonly model: QuadModel;
  /** Frames since it booted. */
  frames = 0;
  private readonly motors = [STOPPED_US, STOPPED_US, STOPPED_US, STOPPED_US];
  private rcSource: "idle" | "page" | "lost" = "idle";
  private rcUs: number[] | null = null;
  private readonly msp = new MspLink();
  private boxes: Uint8Array | null = null;
  private round = new Map<number, Uint8Array>();
  private readonly osdReader = new MspReader();
  private readonly display: DisplayPort;
  private osdFresh = false;
  private osdSent = -OSD_GAP_FRAMES;

  constructor(
    private readonly sim: Sim,
    airframe: Airframe,
    origin: readonly [number, number, number],
    headingDeg: number,
    wind: Partial<Wind>,
    video: "analog" | "digital",
    private readonly events: QuadEvents,
    seed?: number,
  ) {
    this.model = new QuadModel(sim, airframe, origin, headingDeg, wind, seed);
    this.display = new DisplayPort(OSD_GRIDS[video], video === "analog");
    this.msp.ask(MSP_BOXIDS, (boxes) => {
      this.boxes = boxes;
    });
  }

  get phase(): Phase {
    return this.model.phase;
  }

  get armed(): boolean {
    return this.sim.armed;
  }

  /** What stops Betaflight arming now, but the arm switch itself. */
  get blocked(): string[] {
    return blockers(this.sim.armingDisabled & ~(1 << ARM_SWITCH_BIT));
  }

  /** The pilot's sticks, µs from channel 1. */
  rc(us: readonly number[]): void {
    const widths = us.slice(0, CHANNELS).map((w) => Math.max(800, Math.min(2200, Math.round(w))));
    for (let i = widths.length; i < CHANNELS; i++) widths.push(IDLE_US[i]!);
    this.rcSource = "page";
    this.rcUs = widths;
  }

  /** The pilot's sticks have stopped coming. Only theirs can be lost: before any, the idle
   * receiver plays on, or Betaflight would never be ready to arm. */
  rcLost(): void {
    if (this.rcSource === "page") this.rcSource = "lost";
  }

  /** Back on the pad, at rest, a fresh pack; Betaflight flies on (it has no estimator to
   * upset: the jump is only a GPS jump, and its attitude settles on the accelerometer). */
  reset(): void {
    this.model.reset();
  }

  /** One millisecond of the quad and its flight controller. */
  frame(): Fdm {
    const { sim, model } = this;
    const fdm = model.step(FRAME_S, this.motors);
    const [lat, lon] = model.local.toLatLon(fdm.position[0], fdm.position[1]);
    const alt = model.origin[2] - fdm.position[2];
    sim.setImu(fdm.gyro[0], fdm.gyro[1], fdm.gyro[2], fdm.accel[0], fdm.accel[1], fdm.accel[2]);
    sim.setAir(alt, fdm.quat);
    sim.setGps(lat, lon, alt, fdm.velocity);
    if (this.rcSource === "idle") {
      if (this.frames % RC_IDLE_FRAMES === 0) sim.setRc(IDLE_US);
    } else if (this.rcSource === "page" && this.rcUs) {
      sim.setRc(this.rcUs);
      this.rcUs = null;
    }
    sim.step(FRAME_US);
    // Betaflight's motors (rear right, front right, rear left, front left) as ArduPilot's,
    // whole microseconds, as its simulator protocol's are taken.
    const out = sim.motors();
    for (let i = 0; i < 4; i++) this.motors[i] = Math.round(out[AP_FROM_BF[i]!]!);
    this.frames++;
    if (this.frames % SERIAL_FRAMES === 0) this.serial();
    return fdm;
  }

  /** The peripherals: the ground station's telemetry rounds, and the air unit. */
  private serial(): void {
    const { sim, msp } = this;
    if (this.frames % TELEMETRY_FRAMES === 0 && this.boxes && msp.pending === 0) {
      this.round = new Map();
      for (const command of TELEMETRY_ASKS) {
        msp.ask(command, (reply) => this.reply(command, reply));
      }
    }
    for (const bytes of msp.take()) sim.serialWrite(MSP_UART, bytes);
    msp.feed(sim.serialRead(MSP_UART));

    if (this.frames % OSD_POLL_FRAMES === 0) sim.serialWrite(OSD_UART, OSD_POLL);
    for (const f of this.osdReader.feed(sim.serialRead(OSD_UART))) {
      if (f.command === MSP_DISPLAYPORT && this.display.handle(f.payload)) this.osdFresh = true;
    }
    if (this.osdFresh && this.frames - this.osdSent >= OSD_GAP_FRAMES && this.display.screen) {
      this.osdFresh = false;
      this.osdSent = this.frames;
      this.events.osd(this.display.screen);
    }
  }

  private reply(command: number, payload: Uint8Array): void {
    this.round.set(command, payload);
    if (this.round.size < TELEMETRY_ASKS.length || !this.boxes) return;
    try {
      this.events.telemetry(telemetry(this.round, this.boxes));
    } catch (err) {
      console.warn("Betaflight telemetry:", err); // a short reply: the next round tries again
    }
  }
}
