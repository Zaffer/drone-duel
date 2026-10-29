import { describe, expect, test } from "bun:test";
import {
  cellLayout,
  codeAt,
  fontUrl,
  isBlank,
  NTSC_LINES,
  ROW_LINES,
  rasterSize,
  sharpScale,
  snapCell,
} from "../src/video/osdgrid.ts";

describe("the OSD's grid", () => {
  test("analog: the grid spans the picture across, each row 18 of 240 lines, centred", () => {
    // ArduPilot's 30 × 16 cut to NTSC's 13 rows, on a 4:3 picture.
    const c = cellLayout(1200, 900, 30, 13, "analog");
    expect(c.cw).toBe(40);
    expect(c.ch).toBeCloseTo((900 * ROW_LINES) / NTSC_LINES);
    expect(c.ch * 13).toBeCloseTo(900 * (234 / 240));
    expect(c.y0).toBeCloseTo((900 - c.ch * 13) / 2);
    expect(c.y0 + c.ch * 13 + c.y0).toBeCloseTo(900);
  });

  test("digital: the grid fills the 16:9 picture, rows and columns alike", () => {
    const c = cellLayout(1920, 1080, 50, 18, "digital");
    expect(c.cw).toBeCloseTo(38.4);
    expect(c.ch).toBe(60);
    expect(c.y0).toBe(0);
  });

  test("cells snap to whole pixels, meeting edge to edge", () => {
    let end = 0;
    for (let i = 0; i < 50; i++) {
      const [a, w] = snapCell(0, 38.4, i);
      expect(Number.isInteger(a) && Number.isInteger(w)).toBe(true);
      expect(a).toBe(end);
      end = a + w;
    }
    expect(end).toBe(1920);
  });

  test("the chip's raster: 12 × 18 a character, rows centred in the field", () => {
    expect(rasterSize(30, 13)).toEqual({ width: 360, height: 234, top: 3 });
    // PAL's 16 rows do not fit NTSC's field: from its top.
    expect(rasterSize(30, 16).top).toBe(0);
  });

  test("the font blows up by whole multiples, never below 1", () => {
    expect(sharpScale(40, 67.5, 12, 18)).toEqual([3, 3]);
    expect(sharpScale(8, 10, 12, 18)).toEqual([1, 1]);
  });

  test("codes are row by row; space and NUL are blank", () => {
    const osd = { cols: 3, codes: [1, 2, 3, 4, 5, 6] };
    expect(codeAt(osd, 1, 0)).toBe(4);
    expect(codeAt(osd, 2, 0)).toBe(0);
    expect(isBlank(0x20) && isBlank(0) && !isBlank(0x41)).toBe(true);
    expect(fontUrl("bf hd")).toBe("/osd/font/bf%20hd.png");
  });
});
