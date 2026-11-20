// The OSD's grid as a picture lays it out (DESIGN.md §10), apart from any drawing so it can be
// tested: where each character cell falls on a canvas, and the OSD chip's own raster.
//
// Analog: the chip's grid spans the picture across, and each row is 18 of NTSC's 240 lines, so
// the glyphs stretch with the frame rather than keep their shape, and the rows sit centred (13
// of them take 234 lines). Digital: the goggles spread the grid over the whole 16:9 picture,
// rows and columns alike (DJI, HDZero, Walksnail).

import type { OsdMsg } from "../net/protocol.ts";
import { asset, WEB } from "../net/target.ts";

/** Glyphs in a font: one column of 256. */
export const GLYPHS = 256;
/** A MAX7456 character, in the chip's pixels. */
export const GLYPH_W = 12;
export const GLYPH_H = 18;
/** A character row's height, in NTSC's lines, and the lines in a field. */
export const ROW_LINES = 18;
export const NTSC_LINES = 240;

export type OsdLayout = "analog" | "digital";

/** Where the grid's cells fall: each `cw` × `ch` px, the first row `y0` down. */
export interface Cells {
  cw: number;
  ch: number;
  x0: number;
  y0: number;
}

/** The cells of a `cols` × `rows` grid on a `width` × `height` px picture. */
export function cellLayout(
  width: number,
  height: number,
  cols: number,
  rows: number,
  layout: OsdLayout,
): Cells {
  const cw = width / Math.max(cols, 1);
  const ch = layout === "digital" ? height / Math.max(rows, 1) : (height * ROW_LINES) / NTSC_LINES;
  return { cw, ch, x0: 0, y0: (height - ch * rows) / 2 };
}

/** Cell `i` of `n` along an axis, snapped to whole pixels so neighbours neither overlap nor
 * leave a gap: its start and size. */
export function snapCell(origin: number, size: number, i: number): [number, number] {
  const a = Math.round(origin + i * size);
  const b = Math.round(origin + (i + 1) * size);
  return [a, b - a];
}

/** The chip's raster for the analog chain: a pixel per chip pixel, a line per field line; its
 * rows `top` lines down the field, centred. */
export function rasterSize(
  cols: number,
  rows: number,
): { width: number; height: number; top: number } {
  const width = cols * GLYPH_W;
  const height = rows * ROW_LINES;
  return { width, height, top: Math.max(Math.floor((NTSC_LINES - height) / 2), 0) };
}

/** Whole multiples to blow a `glyphW` × `glyphH` glyph up by, pixel for pixel, before it is
 * smoothed down to a `cw` × `ch` cell: an edge then blurs over a pixel at most. */
export function sharpScale(
  cw: number,
  ch: number,
  glyphW: number,
  glyphH: number,
): [number, number] {
  return [Math.max(1, Math.floor(cw / glyphW)), Math.max(1, Math.floor(ch / glyphH))];
}

/** The character code at `row`, `col` (0 off the grid). */
export function codeAt(osd: Pick<OsdMsg, "cols" | "codes">, row: number, col: number): number {
  return osd.codes[row * osd.cols + col] ?? 0;
}

/** A cell with nothing drawn: space or NUL. */
export const isBlank = (code: number): boolean => code === 0x20 || code === 0;

/** The font's image, as the server serves it. */
export const fontUrl = (font: string): string =>
  WEB
    ? asset(`data/osd/font/${encodeURIComponent(font)}.png`)
    : `/osd/font/${encodeURIComponent(font)}.png`;
