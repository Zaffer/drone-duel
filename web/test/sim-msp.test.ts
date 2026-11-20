// MSP as the page's simulator speaks it (src/sim/msp.ts): frames out of a byte stream, a
// ground station's requests and replies, Betaflight's status, and the OSD's screen.

import { describe, expect, test } from "bun:test";
import {
  ARM_SWITCH_BIT,
  blockers,
  DisplayPort,
  MSP_DISPLAYPORT,
  MSP_STATUS_EX,
  MspLink,
  MspReader,
  modeName,
  OSD_GRIDS,
  request,
  requestV1,
  status,
} from "../src/sim/msp.ts";

/** An MSP v1 frame from the flight controller. */
function frame(payload: number[], command = MSP_DISPLAYPORT): Uint8Array {
  let sum = payload.length ^ command;
  for (const b of payload) sum ^= b;
  return new Uint8Array([0x24, 0x4d, 0x3e, payload.length, command, ...payload, sum]);
}

/** An MSP v2 reply: a request's bytes with the direction turned round, its CRC the same. */
function reply(command: number, payload: number[]): Uint8Array {
  const out = request(command, new Uint8Array(payload));
  out[2] = 0x3e;
  return out;
}

const text = (s: string): number[] => [...s].map((c) => c.charCodeAt(0));
const join = (...parts: Uint8Array[]): Uint8Array => new Uint8Array(parts.flatMap((p) => [...p]));

describe("MSP frames", () => {
  test("a v2 request is $X<, flag, command, size, CRC (DVB-S2): MSP_STATUS_EX's bytes", () => {
    // As drone_duels.firmware.inav.MspLink sends it (its bytes, from Python).
    expect([...request(MSP_STATUS_EX)]).toEqual([0x24, 0x58, 0x3c, 0, 150, 0, 0, 0, 190]);
    expect([...requestV1(2)]).toEqual([0x24, 0x4d, 0x3c, 0, 2, 2]);
  });

  test("frames split across reads come whole; a bad checksum is skipped", () => {
    const bad = frame([4]);
    bad[bad.length - 1]! ^= 0xff;
    const data = join(
      new Uint8Array(text("noise")),
      frame([2]),
      reply(108, [1, 2, 3]),
      bad,
      frame([3, 0, 0, 0, ...text("AB")]),
    );
    const reader = new MspReader();
    const got: [number, number[]][] = [];
    for (let i = 0; i < data.length; i += 3) {
      for (const f of reader.feed(data.subarray(i, i + 3))) got.push([f.command, [...f.payload]]);
    }
    expect(got).toEqual([
      [MSP_DISPLAYPORT, [2]],
      [108, [1, 2, 3]],
      [MSP_DISPLAYPORT, [3, 0, 0, 0, 65, 66]],
    ]);
  });

  test("a link gives each request its reply, in whatever order they come", () => {
    const link = new MspLink();
    const got: string[] = [];
    link.ask(108, (p) => got.push(`attitude ${[...p]}`));
    link.ask(109, (p) => got.push(`altitude ${[...p]}`));
    expect(link.take().length).toBe(2);
    expect(link.take().length).toBe(0);
    expect(link.pending).toBe(2);
    link.feed(join(reply(109, [9]), reply(77, [1]), reply(108, [8])));
    expect(got).toEqual(["altitude 9", "attitude 8"]);
    expect(link.pending).toBe(0);
  });
});

describe("Betaflight's status", () => {
  test("boxes by their permanent ids, past the first 32 too; arming flags by name", () => {
    // 40 boxes: ARM (0) first, ANGLE (1) second, POSHOLD (11) the 34th.
    const boxes = new Uint8Array(40).map((_, i) => 100 + i);
    boxes[0] = 0;
    boxes[1] = 1;
    boxes[33] = 11;
    const r = new Uint8Array(16 + 1 + 1 + 4);
    const view = new DataView(r.buffer);
    view.setUint32(6, 0b11, true); // ARM and ANGLE on
    r[15] = 1; // one more byte of flags
    r[16] = 0b10; // the 34th box
    r[17] = 30; // arming flags' count
    view.setUint32(18, (1 << 7) | (1 << ARM_SWITCH_BIT), true);
    const s = status(r, boxes);
    expect([...s.active].sort((a, b) => a - b)).toEqual([0, 1, 11]);
    expect(s.blocked).toEqual(["throttle", "arm switch"]);
    expect(modeName(s.active)).toBe("POSHOLD");
    expect(modeName(new Set([0]))).toBe("ACRO");
    expect(blockers(0)).toEqual([]);
  });
});

describe("the OSD over MSP DisplayPort", () => {
  const write = (row: number, col: number, s: string) => frame([3, row, col, 0, ...text(s)]);

  test("a screen is drawn only on draw, on the video system's grid", () => {
    const reader = new MspReader();
    const port = new DisplayPort(OSD_GRIDS.digital, false);
    const bytes = join(frame([5, 0, 0]), frame([2]), write(2, 3, "ANGL"), write(19, 51, "XYZ"));
    const drawn = reader.feed(bytes).map((f) => port.handle(f.payload));
    expect(drawn).toEqual([false, false, false, false]);
    expect(port.screen).toBeNull();
    expect(port.handle(reader.feed(frame([4]))[0]!.payload)).toBe(true);
    const screen = port.screen!;
    expect([screen.cols, screen.rows, screen.font]).toEqual([53, 20, "betaflight"]);
    expect(screen.codes.length).toBe(53 * 20);
    expect(String.fromCharCode(...screen.codes.slice(2 * 53 + 3, 2 * 53 + 7))).toBe("ANGL");
    // Writing past the last column stops there.
    expect(String.fromCharCode(...screen.codes.slice(19 * 53 + 51, 19 * 53 + 53))).toBe("XY");
  });

  test("analog video's is NTSC's 30 x 13; a clear blanks it", () => {
    const reader = new MspReader();
    const port = new DisplayPort(OSD_GRIDS.analog, true);
    for (const f of reader.feed(join(write(12, 0, "LOW"), write(13, 0, "GONE"), frame([4])))) {
      port.handle(f.payload);
    }
    expect([port.screen!.cols, port.screen!.rows]).toEqual([30, 13]);
    expect(String.fromCharCode(...port.screen!.codes.slice(12 * 30, 12 * 30 + 3))).toBe("LOW");
    for (const f of reader.feed(join(frame([2]), frame([4])))) port.handle(f.payload);
    expect(port.screen!.codes.every((c) => c === 0x20)).toBe(true);
  });
});
