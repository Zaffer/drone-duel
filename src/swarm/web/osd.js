// ArduPilot's own OSD over the camera feed, as an analog NTSC picture shows it: the screen the
// flight controller's OSD chip lays into the video (swarm.osd), a grid of character codes,
// each drawn from the chip's font (OSD_FONT). The font is one column of 256 glyphs, white with
// a black edge on clear.
//
// The chip's grid spans the picture across, and each row is 18 of NTSC's 240 lines, so the
// glyphs stretch with the frame rather than keep their shape. They stay sharp: the font is
// first blown up pixel for pixel to a whole multiple of its size (sharpFont), and only that
// is smoothed down to the cell, so an edge blurs over a pixel at most.
//
// Over the analog feed (analog.js) the OSD is not laid over the picture but put into its
// video, as the chip does: OsdRaster draws the chip's pixels for that, one a field line.
//
// A digital system's OSD (the wing's INAV, through a DJI air unit) is drawn by the goggles on a
// grid that fills the picture, rows and columns alike: OsdView's `fill`.

const GLYPHS = 256;
/** A character row's height, in NTSC's lines. */
const ROW_LINES = 18;
const NTSC_LINES = 240;
const GLYPH_W = 12;

/** @type {Map<string, HTMLImageElement>} each font as it is asked for, by its URL. */
const fonts = new Map();

/** ArduPilot's OSD_FONT `index`, as swarm.view serves it. */
const chipFont = (index) => `/osd/font/${index}.png`;

/** The font at `src`, loading the first time; `onload` when it has. */
function fontImage(src, onload) {
  let font = fonts.get(src);
  if (!font) {
    font = new Image();
    font.src = src;
    fonts.set(src, font);
  }
  if (!font.complete) font.decode().then(onload, () => {});
  return font;
}

/** `font` blown up `kx` x `ky` times, each pixel a block: no smoothing. */
function sharpFont(font, kx, ky) {
  const canvas = document.createElement("canvas");
  canvas.width = font.naturalWidth * kx;
  canvas.height = font.naturalHeight * ky;
  const ctx = canvas.getContext("2d");
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(font, 0, 0, canvas.width, canvas.height);
  return canvas;
}

export class OsdView {
  /** On `canvas`; `font`, a screen's font index to its URL (default ArduPilot's chip fonts);
   * `fill`, the grid's rows spread over the whole picture (a digital system's) rather than
   * NTSC's 18 lines each.
   * @param {HTMLCanvasElement} canvas */
  constructor(canvas, { font = chipFont, fill = false } = {}) {
    this.canvas = canvas;
    this.font = font;
    this.fill = fill;
    this.ctx = canvas.getContext("2d");
    this._drawn = null;
    this._size = "";
    /** The font as drawn (sharpFont), and which font at what multiple. */
    this._sharp = null;
    this._sharpKey = "";
  }

  /** Draw `screen` ({cols, rows, font, text: one Latin-1 string per row}); only when it, or
   * the canvas's size, changed. */
  draw(screen) {
    const src = this.font(screen.font ?? 0);
    const font = fontImage(src, () => {
      this._drawn = null; // drawn with the next screen
    });
    if (!font.complete || !font.naturalWidth) return;
    const box = this.canvas.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const size = `${box.width}x${box.height}@${dpr}`;
    if (screen === this._drawn && size === this._size) return;
    this._drawn = screen;
    if (size !== this._size) {
      this._size = size;
      this.canvas.width = Math.round(box.width * dpr);
      this.canvas.height = Math.round(box.height * dpr);
    }
    const ctx = this.ctx;
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    const cw = this.canvas.width / screen.cols;
    const ch = this.fill ? this.canvas.height / screen.rows : (this.canvas.height * ROW_LINES) / NTSC_LINES;
    const y0 = (this.canvas.height - ch * screen.rows) / 2;
    const kx = Math.max(1, Math.floor(cw / font.naturalWidth));
    const ky = Math.max(1, Math.floor(ch / (font.naturalHeight / GLYPHS)));
    const key = `${src}:${kx}x${ky}`;
    if (this._sharpKey !== key) {
      this._sharpKey = key;
      this._sharp = sharpFont(font, kx, ky);
    }
    const sharp = this._sharp;
    const gw = sharp.width;
    const gh = sharp.height / GLYPHS;
    ctx.imageSmoothingEnabled = true;
    ctx.imageSmoothingQuality = "low"; // bilinear: the last step is under 2x
    for (let row = 0; row < screen.rows; row++) {
      const line = screen.text[row] ?? "";
      for (let col = 0; col < line.length; col++) {
        const code = line.charCodeAt(col);
        if (code === 0x20 || code === 0) continue;
        ctx.drawImage(sharp, 0, code * gh, gw, gh, col * cw, y0 + row * ch, cw, ch);
      }
    }
  }

  clear() {
    this._drawn = null;
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }
}

/** The OSD as the chip puts it into the analog video: a pixel for each of the chip's, white,
 * black or clear, 12 x 18 a character, a pixel line a field line. Its rows are `top` field
 * lines down: NTSC's 13 rows take 234 of a field's 240, centred. `changed` until the feed
 * has taken it (analog.js). */
export class OsdRaster {
  constructor() {
    this.canvas = document.createElement("canvas");
    this.ctx = this.canvas.getContext("2d");
    this.top = 0;
    this.changed = false;
    this._drawn = null;
  }

  /** Draw `screen` (as OsdView.draw), when it is a new one. */
  draw(screen) {
    const font = fontImage(chipFont(screen.font ?? 0), () => {
      this._drawn = null;
    });
    if (!font.complete || !font.naturalWidth || screen === this._drawn) return;
    this._drawn = screen;
    const gh = font.naturalHeight / GLYPHS;
    const w = screen.cols * GLYPH_W;
    const h = screen.rows * ROW_LINES;
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    this.top = Math.max(Math.floor((NTSC_LINES - h) / 2), 0);
    const ctx = this.ctx;
    ctx.imageSmoothingEnabled = false;
    ctx.clearRect(0, 0, w, h);
    for (let row = 0; row < screen.rows; row++) {
      const line = screen.text[row] ?? "";
      for (let col = 0; col < line.length; col++) {
        const code = line.charCodeAt(col);
        if (code === 0x20 || code === 0) continue;
        ctx.drawImage(font, 0, code * gh, font.naturalWidth, gh, col * GLYPH_W, row * ROW_LINES, GLYPH_W, ROW_LINES);
      }
    }
    this.changed = true;
  }
}
