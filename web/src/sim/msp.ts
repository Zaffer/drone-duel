// MSP, the page's side of Betaflight's serial ports (the UARTs of web/wasm/bfweb.c): a ground
// station's requests and their replies (`MspLink`, MSP v2 frames, as
// drone_duels.firmware.inav.MspLink over the native SITL's TCP port), Betaflight's state read
// through them (`telemetry`, as drone_duels.firmware.betaflight.BetaflightMsp), and the far
// side of MSP DisplayPort, the OSD (`MspReader`, `DisplayPort`: drone_duels.video.osd).
//
// Nothing here waits: the firmware answers as its own clock runs, so a caller writes requests,
// steps the firmware, and feeds what the UART sent back.

import type { OsdMsg, TelemetryMsg } from "../net/protocol.ts";

export const MSP_FC_VARIANT = 2;
export const MSP_STATUS = 101;
export const MSP_MOTOR = 104;
export const MSP_RAW_GPS = 106;
export const MSP_ATTITUDE = 108;
export const MSP_ALTITUDE = 109;
export const MSP_ANALOG = 110;
export const MSP_BOXIDS = 119;
export const MSP_STATUS_EX = 150;
export const MSP_DISPLAYPORT = 182;

function crc8DvbS2(data: Uint8Array, from: number, to: number): number {
  let crc = 0;
  for (let i = from; i < to; i++) {
    crc ^= data[i]!;
    for (let b = 0; b < 8; b++) crc = crc & 0x80 ? ((crc << 1) ^ 0xd5) & 0xff : (crc << 1) & 0xff;
  }
  return crc;
}

/** An MSP v2 request: $X<, flag, command, size, payload, CRC. */
export function request(command: number, payload: Uint8Array = new Uint8Array(0)): Uint8Array {
  const out = new Uint8Array(9 + payload.length);
  out.set([0x24, 0x58, 0x3c, 0, command & 0xff, command >> 8, payload.length & 0xff], 0);
  out[7] = payload.length >> 8;
  out.set(payload, 8);
  out[8 + payload.length] = crc8DvbS2(out, 3, 8 + payload.length);
  return out;
}

/** An MSP v1 request: $M<, size, command, payload, checksum. */
export function requestV1(command: number, payload: Uint8Array = new Uint8Array(0)): Uint8Array {
  const out = new Uint8Array(6 + payload.length);
  out.set([0x24, 0x4d, 0x3c, payload.length, command], 0);
  out.set(payload, 5);
  let sum = payload.length ^ command;
  for (const b of payload) sum ^= b;
  out[5 + payload.length] = sum;
  return out;
}

export interface Frame {
  command: number;
  payload: Uint8Array;
}

/** MSP frames from the flight controller ($M> and $X>), out of a byte stream. */
export class MspReader {
  private buf = new Uint8Array(0);

  /** Each whole, valid frame in what has arrived so far. */
  feed(data: Uint8Array): Frame[] {
    if (data.length === 0 && this.buf.length === 0) return [];
    const buf = new Uint8Array(this.buf.length + data.length);
    buf.set(this.buf);
    buf.set(data, this.buf.length);
    const frames: Frame[] = [];
    let at = 0;
    for (;;) {
      // The next "$M>" or "$X>".
      while (
        at + 2 < buf.length &&
        !(
          buf[at] === 0x24 &&
          (buf[at + 1] === 0x4d || buf[at + 1] === 0x58) &&
          buf[at + 2] === 0x3e
        )
      ) {
        at++;
      }
      if (at + 2 >= buf.length) break; // a partial start waits for the next read
      const v2 = buf[at + 1] === 0x58;
      const head = v2 ? 8 : 5;
      if (buf.length < at + head) break;
      const size = v2 ? buf[at + 6]! | (buf[at + 7]! << 8) : buf[at + 3]!;
      const command = v2 ? buf[at + 4]! | (buf[at + 5]! << 8) : buf[at + 4]!;
      const end = at + head + size;
      if (buf.length < end + 1) break;
      let sum: number;
      if (v2) {
        sum = crc8DvbS2(buf, at + 3, end);
      } else {
        sum = 0;
        for (let i = at + 3; i < end; i++) sum ^= buf[i]!;
      }
      if (sum === buf[end]) {
        frames.push({ command, payload: buf.slice(at + head, end) });
        at = end + 1;
      } else {
        at += 3;
      }
    }
    this.buf = buf.slice(at);
    return frames;
  }
}

/** A ground station's link: `ask` queues requests (their bytes come from `take`, to write to
 * the UART), `feed` takes what the UART sent and gives each request its reply. Replies come in
 * the order asked, as Betaflight answers them. */
export class MspLink {
  private readonly reader = new MspReader();
  private out: Uint8Array[] = [];
  private waiting: { command: number; done: (payload: Uint8Array) => void }[] = [];

  ask(command: number, done: (payload: Uint8Array) => void): void {
    this.out.push(request(command));
    this.waiting.push({ command, done });
  }

  /** Requests asked and not yet answered. */
  get pending(): number {
    return this.waiting.length;
  }

  /** The bytes to send now. */
  take(): Uint8Array[] {
    const out = this.out;
    this.out = [];
    return out;
  }

  feed(data: Uint8Array): void {
    for (const frame of this.reader.feed(data)) {
      const i = this.waiting.findIndex((w) => w.command === frame.command);
      if (i < 0) continue;
      const [w] = this.waiting.splice(i, 1);
      w!.done(frame.payload);
    }
  }

  /** Forget what was asked: the flight controller it was asked of is gone. */
  reset(): void {
    this.out = [];
    this.waiting = [];
  }
}

const BOX_ARM = 0;
const GPS_FIX = 1;
/** Betaflight's arming disable flags, bit by bit (fc/runtime_config.c), in the game's words
 * (drone_duels.firmware.betaflight.ARMING_BLOCKERS). */
export const ARMING_BLOCKERS = [
  "no gyro",
  "failsafe",
  "no RC link",
  "not disarmed",
  "failsafe switch",
  "runaway",
  "crash",
  "throttle",
  "not level",
  "booting",
  "no prearm",
  "overloaded",
  "calibrating",
  "CLI",
  "CMS menu",
  "BST",
  "MSP",
  "paralyzed",
  "GPS",
  "rescue switch",
  "DShot telemetry",
  "reboot required",
  "DShot bitbang",
  "accelerometer not calibrated",
  "motor protocol",
  "crash flip switch",
  "altitude hold switch",
  "position hold switch",
  "autopilot switch",
  "arm switch",
] as const;
export const ARM_SWITCH = "arm switch";
/** The arm switch's flag: set when the switch is on and Betaflight will not arm. */
export const ARM_SWITCH_BIT = ARMING_BLOCKERS.indexOf(ARM_SWITCH);
/** Betaflight's flight modes by their boxes' permanent ids (msp/msp_box.c), the one shown
 * first first; with none of them it flies ACRO. */
const MODES: readonly (readonly [number, string])[] = [
  [27, "FAILSAFE"],
  [46, "RESCUE"],
  [56, "AUTOPILOT"],
  [11, "POSHOLD"],
  [3, "ALTHOLD"],
  [1, "ANGLE"],
  [2, "HORIZON"],
];

export function blockers(flags: number): string[] {
  return ARMING_BLOCKERS.filter((_, bit) => (flags >>> bit) & 1);
}

export function modeName(active: ReadonlySet<number>): string {
  for (const [box, name] of MODES) if (active.has(box)) return name;
  return "ACRO";
}

/** MSP_STATUS_EX: the boxes active (permanent ids, by `boxes`: MSP_BOXIDS' reply) and what
 * stops it arming. Cycle time, I2C errors, sensors (u16 each), the first 32 boxes' flags (u32),
 * PID profile (u8), load (u16), PID profile count, rate profile, the count of further flag
 * bytes (u8 each), those bytes, the count of arming flags (u8), arming flags (u32). */
export function status(
  reply: Uint8Array,
  boxes: Uint8Array,
): { active: Set<number>; blocked: string[] } {
  const view = new DataView(reply.buffer, reply.byteOffset, reply.byteLength);
  const extra = reply[15]!;
  const active = new Set<number>();
  for (let i = 0; i < boxes.length; i++) {
    const on =
      i < 32
        ? (view.getUint32(6, true) >>> i) & 1
        : ((reply[16 + ((i - 32) >> 3)] ?? 0) >> ((i - 32) & 7)) & 1;
    if (on && i < 32 + 8 * extra) active.add(boxes[i]!);
  }
  return { active, blocked: blockers(view.getUint32(16 + extra + 1, true)) };
}

/** The replies a telemetry frame is read from, by command. */
export const TELEMETRY_ASKS = [
  MSP_STATUS_EX,
  MSP_ATTITUDE,
  MSP_ALTITUDE,
  MSP_RAW_GPS,
  MSP_ANALOG,
  MSP_MOTOR,
] as const;

/** What a ground station sees, protocol.TelemetryMsg, from one round of TELEMETRY_ASKS'
 * replies (`replies` by command) and MSP_BOXIDS' (`boxes`). */
export function telemetry(
  replies: ReadonlyMap<number, Uint8Array>,
  boxes: Uint8Array,
): TelemetryMsg {
  const of = (command: number): DataView => {
    const r = replies.get(command)!;
    return new DataView(r.buffer, r.byteOffset, r.byteLength);
  };
  const { active, blocked } = status(replies.get(MSP_STATUS_EX)!, boxes);
  const attitude = of(MSP_ATTITUDE);
  const altitude = of(MSP_ALTITUDE);
  const gps = of(MSP_RAW_GPS);
  // MSP_ANALOG: volts (u8, 0.1 V), mAh, RSSI, amps (u16 each), volts (u16, 0.01 V). The
  // simulator's Betaflight has no battery sensor: 0.
  const analog = of(MSP_ANALOG);
  const vbat = analog.byteLength >= 9 ? analog.getUint16(7, true) : 0;
  const motors = of(MSP_MOTOR);
  let throttle = 1000;
  if (motors.byteLength >= 8) {
    throttle = 0;
    for (let i = 0; i < 4; i++) throttle += motors.getUint16(2 * i, true) / 4;
  }
  const fix = gps.getUint8(0) >= GPS_FIX;
  return {
    ch: "telemetry",
    armed: active.has(BOX_ARM),
    mode: modeName(active),
    lat: fix ? gps.getInt32(2, true) / 1e7 : null,
    lon: fix ? gps.getInt32(6, true) / 1e7 : null,
    alt_amsl: fix ? gps.getUint16(10, true) : null,
    alt_rel: altitude.getInt32(0, true) / 100,
    roll_deg: attitude.getInt16(0, true) / 10,
    pitch_deg: -attitude.getInt16(2, true) / 10,
    yaw_deg: attitude.getInt16(4, true),
    groundspeed: gps.getUint16(12, true) / 100,
    airspeed: null,
    climb: altitude.getInt16(4, true) / 100,
    throttle_pct: Math.max(0, Math.min(100, (throttle - 1000) / 10)),
    battery_v: vbat ? vbat / 100 : null,
    battery_pct: null,
    gps_sats: gps.getUint8(1),
    blockers: blocked.filter((b) => b !== ARM_SWITCH),
  };
}

// MSP DisplayPort: the OSD.

const CLEAR = 2;
const WRITE = 3;
const DRAW = 4;
const OPTIONS = 5;
const BLANK = 0x20;
/** An analog chip's grid, and the rows of it an NTSC picture shows. */
const SD: readonly [number, number] = [30, 16];
const NTSC_ROWS = 13;

export type Grids = Readonly<Record<number, readonly [number, number]>>;
/** The grid Betaflight's MSP DisplayPort draws on (io/displayport_msp.c): it names none, so
 * the video system's: NTSC's 30 x 13 for analog, its HD canvas for digital. */
export const OSD_GRIDS: Readonly<Record<"analog" | "digital", Grids>> = {
  analog: { 0: [30, 13] },
  digital: { 0: [53, 20] },
};

/** The far side of MSP DisplayPort: a screen the flight controller writes, then draws, on a
 * grid of `grids` by the resolution it asks for, the first until it asks. `analog`: an analog
 * chip's 30 x 16 is cut to NTSC's rows. */
export class DisplayPort {
  cols: number;
  rows: number;
  private back: Uint8Array[];
  /** The last screen drawn. */
  screen: OsdMsg | null = null;

  constructor(
    private readonly grids: Grids,
    private readonly analog: boolean,
  ) {
    [this.cols, this.rows] = Object.values(grids)[0]!;
    this.back = this.blank();
  }

  private blank(): Uint8Array[] {
    return Array.from({ length: this.rows }, () => new Uint8Array(this.cols).fill(BLANK));
  }

  /** Take one MSP_DISPLAYPORT payload. True when it drew a new screen. */
  handle(payload: Uint8Array): boolean {
    const sub = payload[0];
    if (sub === OPTIONS && payload.length >= 3) {
      const grid = this.grids[payload[2]!] ?? [this.cols, this.rows];
      if (grid[0] !== this.cols || grid[1] !== this.rows) {
        [this.cols, this.rows] = grid;
        this.back = this.blank();
      }
    } else if (sub === CLEAR) {
      this.back = this.blank();
    } else if (sub === WRITE && payload.length >= 4) {
      const row = payload[1]!;
      const col = payload[2]!;
      const line = this.back[row];
      if (line) {
        const n = Math.min(payload.length - 4, Math.max(0, this.cols - col));
        for (let i = 0; i < n; i++) line[col + i] = payload[4 + i]!;
      }
    } else if (sub === DRAW) {
      const sd = this.cols === SD[0] && this.rows === SD[1];
      const shown = this.analog && sd ? NTSC_ROWS : this.rows;
      const codes: number[] = [];
      for (const line of this.back.slice(0, shown)) codes.push(...line);
      this.screen = { ch: "osd", font: "betaflight", cols: this.cols, rows: shown, codes };
      return true;
    }
    return false;
  }
}
