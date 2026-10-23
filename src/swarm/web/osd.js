// ArduPilot's own OSD over the camera feed, as an analog NTSC picture shows it: the screen the
// flight controller's OSD chip lays into the video (swarm.osd), a grid of character codes,
// each drawn from the chip's font (OSD_FONT). The font is one column of 256 glyphs, white with
// a black edge on clear.
//
// The chip's grid spans the picture across, and each row is 18 of NTSC's 240 lines, so the
// glyphs stretch with the frame rather than keep their shape. They stay sharp: the font is
// first blown up pixel for pixel to a whole multiple of its size (sharpFont), and only that
// is smoothed down to the cell, so an edge blurs over a pixel at most.

const GLYPHS = 256;
/** A character row's height, in NTSC's lines. */
const ROW_LINES = 18;
const NTSC_LINES = 240;

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
  /** @param {HTMLCanvasElement} canvas */
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    /** @type {Map<number, HTMLImageElement>} each OSD_FONT as it is asked for. */
    this.fonts = new Map();
    this._drawn = null;
    this._size = "";
    /** The font as drawn (sharpFont), and which font at what multiple. */
    this._sharp = null;
    this._sharpKey = "";
  }

  /** OSD_FONT `index`, loading the first time. */
  _font(index) {
    let font = this.fonts.get(index);
    if (!font) {
      font = new Image();
      font.src = `/osd/font/${index}.png`;
      font.decode().then(
        () => {
          this._drawn = null; // drawn with the next screen
        },
        () => {},
      );
      this.fonts.set(index, font);
    }
    return font;
  }

  /** Draw `screen` ({cols, rows, font, text: one Latin-1 string per row}); only when it, or
   * the canvas's size, changed. */
  draw(screen) {
    const font = this._font(screen.font ?? 0);
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
    const ch = (this.canvas.height * ROW_LINES) / NTSC_LINES;
    const y0 = (this.canvas.height - ch * screen.rows) / 2;
    const kx = Math.max(1, Math.floor(cw / font.naturalWidth));
    const ky = Math.max(1, Math.floor(ch / (font.naturalHeight / GLYPHS)));
    const key = `${screen.font ?? 0}:${kx}x${ky}`;
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
