// ArduPilot's own OSD over the camera feed, as digital FPV goggles draw it: the drone's MSP
// DisplayPort screen (swarm.osd), a grid of character codes, each drawn from ArduPilot's
// Walksnail font. The font is one column of 256 glyphs, white with a black edge on clear.
//
// The grid keeps the glyphs' shape and is centred in the canvas, which leaves the view's
// panels and toolbar clear.

const GLYPHS = 256;

export class OsdView {
  /** @param {HTMLCanvasElement} canvas */
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.font = new Image();
    this.font.src = "/osd/font.png";
    this.ready = this.font.decode().then(
      () => true,
      () => false,
    );
    this._drawn = null;
    this._size = "";
  }

  /** Draw `screen` ({cols, rows, text: one Latin-1 string per row}); only when it, or the
   * canvas's size, changed. */
  draw(screen) {
    if (!this.font.complete || !this.font.naturalWidth) return;
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
    const gw = this.font.naturalWidth;
    const gh = this.font.naturalHeight / GLYPHS;
    const cell = Math.min(this.canvas.width / screen.cols / gw, this.canvas.height / screen.rows / gh);
    const cw = gw * cell;
    const ch = gh * cell;
    const x0 = (this.canvas.width - cw * screen.cols) / 2;
    const y0 = (this.canvas.height - ch * screen.rows) / 2;
    ctx.imageSmoothingQuality = "high";
    for (let row = 0; row < screen.rows; row++) {
      const line = screen.text[row] ?? "";
      for (let col = 0; col < line.length; col++) {
        const code = line.charCodeAt(col);
        if (code === 0x20 || code === 0) continue;
        ctx.drawImage(this.font, 0, code * gh, gw, gh, x0 + col * cw, y0 + row * ch, cw, ch);
      }
    }
  }

  clear() {
    this._drawn = null;
    this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
  }
}
