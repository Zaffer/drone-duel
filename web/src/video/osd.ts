// The firmware's own OSD (DESIGN.md §10): the screen it sends over MSP DisplayPort (the `osd`
// channel: a grid of character codes), drawn in its own font, a column of 256 glyphs, white with
// a black edge on clear (served at /osd/font/{font}.png).
//
// OsdView draws it over the picture: for analog as the OSD chip lays it into NTSC (the grid spans
// the picture, each row 18 of 240 lines), for digital as the goggles do (the grid fills the
// 16:9 picture). The glyphs stay sharp: the font is first blown up pixel for pixel to a whole
// multiple of its size, and only that is smoothed down to the cell, so an edge blurs over a pixel
// at most.
//
// OsdRaster is the chip's own pixels for the analog chain (analog-vision's AnalogFeed puts them
// into the video before the transmitter, as the chip does): a pixel per chip pixel, a pixel line
// per field line.

import type { OsdMsg } from "../net/protocol.ts";
import {
  cellLayout,
  codeAt,
  fontUrl,
  GLYPH_H,
  GLYPH_W,
  GLYPHS,
  isBlank,
  type OsdLayout,
  ROW_LINES,
  rasterSize,
  sharpScale,
  snapCell,
} from "./osdgrid.ts";

export type { OsdLayout } from "./osdgrid.ts";

const fonts = new Map<string, HTMLImageElement>();

/** The font at `url`, loading the first time; `onload` once it has. */
function fontImage(url: string, onload: () => void): HTMLImageElement {
  let font = fonts.get(url);
  if (!font) {
    font = new Image();
    font.src = url;
    fonts.set(url, font);
  }
  if (!font.complete) font.decode().then(onload, () => {});
  return font;
}

const ready = (font: HTMLImageElement): boolean => font.complete && font.naturalWidth > 0;

/** `font` blown up `kx` × `ky` times, each pixel a block: no smoothing. */
function sharpFont(font: HTMLImageElement, kx: number, ky: number): HTMLCanvasElement {
  const canvas = document.createElement("canvas");
  canvas.width = font.naturalWidth * kx;
  canvas.height = font.naturalHeight * ky;
  const ctx = canvas.getContext("2d")!;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(font, 0, 0, canvas.width, canvas.height);
  return canvas;
}

/** The OSD over the picture, on a canvas sized to its CSS box. */
export class OsdView {
  private readonly ctx: CanvasRenderingContext2D;
  private drawn: OsdMsg | null = null;
  private size = "";
  private sharp: HTMLCanvasElement | null = null;
  private sharpKey = "";
  private blank = true;

  constructor(
    readonly canvas: HTMLCanvasElement,
    private layoutMode: OsdLayout = "analog",
  ) {
    this.ctx = canvas.getContext("2d")!;
  }

  get layout(): OsdLayout {
    return this.layoutMode;
  }

  set layout(layout: OsdLayout) {
    if (layout === this.layoutMode) return;
    this.layoutMode = layout;
    this.drawn = null;
  }

  /** Draw `osd`, when it or the canvas's size changed since the last. */
  draw(osd: OsdMsg): void {
    const url = fontUrl(osd.font);
    const font = fontImage(url, () => {
      this.drawn = null; // drawn with the next screen
    });
    if (!ready(font)) return;
    const dpr = window.devicePixelRatio || 1;
    const w = Math.round(this.canvas.clientWidth * dpr);
    const h = Math.round(this.canvas.clientHeight * dpr);
    const size = `${w}x${h}`;
    if (osd === this.drawn && size === this.size) return;
    this.drawn = osd;
    if (size !== this.size) {
      this.size = size;
      this.canvas.width = w;
      this.canvas.height = h;
    }
    const ctx = this.ctx;
    ctx.clearRect(0, 0, w, h);
    this.blank = false;
    const { cw, ch, x0, y0 } = cellLayout(w, h, osd.cols, osd.rows, this.layoutMode);
    const glyphH = font.naturalHeight / GLYPHS;
    const [kx, ky] = sharpScale(cw, ch, font.naturalWidth, glyphH);
    const key = `${url}:${kx}x${ky}`;
    if (this.sharpKey !== key || !this.sharp) {
      this.sharpKey = key;
      this.sharp = sharpFont(font, kx, ky);
    }
    const sharp = this.sharp;
    const gw = sharp.width;
    const gh = sharp.height / GLYPHS;
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = "low"; // bilinear: the last step is under 2x
    for (let row = 0; row < osd.rows; row++) {
      const [y, ch1] = snapCell(y0, ch, row);
      for (let col = 0; col < osd.cols; col++) {
        const code = codeAt(osd, row, col);
        if (isBlank(code)) continue;
        const [x, cw1] = snapCell(x0, cw, col);
        ctx.drawImage(sharp, 0, (code & 0xff) * gh, gw, gh, x, y, cw1, ch1);
      }
    }
  }

  clear(): void {
    this.drawn = null;
    if (this.blank) return;
    this.blank = true;
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }
}

/** The OSD as the chip puts it into the analog video: a pixel for each of the chip's, white,
 * black or clear, 12 × 18 a character, a pixel line a field line, its rows `top` field lines
 * down. `changed` until the feed has taken it (AnalogFeed clears it). */
export class OsdRaster {
  readonly canvas: HTMLCanvasElement = document.createElement("canvas");
  top = 0;
  changed = false;
  private readonly ctx = this.canvas.getContext("2d")!;
  private drawn: OsdMsg | null = null;

  /** Draw `osd`, when it is a new one. */
  draw(osd: OsdMsg): void {
    const font = fontImage(fontUrl(osd.font), () => {
      this.drawn = null;
    });
    if (!ready(font) || osd === this.drawn) return;
    this.drawn = osd;
    const gh = font.naturalHeight / GLYPHS;
    const { width, height, top } = rasterSize(osd.cols, osd.rows);
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width;
      this.canvas.height = height;
    }
    this.top = top;
    const ctx = this.ctx;
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, width, height);
    for (let row = 0; row < osd.rows; row++) {
      for (let col = 0; col < osd.cols; col++) {
        const code = codeAt(osd, row, col);
        if (isBlank(code)) continue;
        ctx.drawImage(
          font,
          0,
          (code & 0xff) * gh,
          font.naturalWidth,
          gh,
          col * GLYPH_W,
          row * ROW_LINES,
          GLYPH_W,
          GLYPH_H,
        );
      }
    }
    this.changed = true;
  }

  /** Nothing on screen: an empty raster. */
  clear(): void {
    if (this.drawn === null && this.canvas.width <= 1) return;
    this.drawn = null;
    this.canvas.width = 1;
    this.canvas.height = 1;
    this.changed = true;
  }
}
