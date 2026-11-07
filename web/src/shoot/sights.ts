// The sight picture over the view (DESIGN.md §7, Shoot Down). A holographic sight's reticle is
// a hologram focused at infinity: it is seen along the sight's own axis, wherever the eye is
// behind the window, and only through the window. EOTech's EXPS: a 68 MOA ring round a 1 MOA dot.
// Iron sights: the rear aperture, a few centimetres from the eye, is out of focus, a soft dark
// ring with a bright hole; the front post is the model's own, in the 3D view.
//
// Each is a small canvas the size of what it shows, moved over the view by a CSS transform and
// faded by its opacity: the compositor's work, no repaint. (A canvas over the whole window,
// cleared and drawn each frame, was painted by Chrome on the GPU thread the map draws on: coming
// off the sight, while the map's whole view came back, it held four frames 40 to 60 ms each,
// measured 2026-11-07; hidden, there were none.) The reticle is drawn again each frame (the
// window it is seen through moves over it); the aperture only when its size changes.

import * as C from "cesium";
import type { World } from "../world/world.ts";

const MOA = Math.PI / (180 * 60);
const RING_MOA = 68;
const DOT_MOA = 1;
/** The irons' aperture (an A2's large one, 5 mm) and how far it is in front of the eye. */
const APERTURE_M = 0.005;

export interface SightPicture {
  sight: "holo" | "irons";
  /** 0 head up .. 1 on the sight. */
  ads: number;
  /** The eye, earth-fixed, and the sight's axis there, unit, earth-fixed. */
  eye: C.Cartesian3;
  axis: C.Cartesian3;
  /** The holo's window corners, earth-fixed. */
  window: C.Cartesian3[];
  /** The rear aperture, earth-fixed. */
  aperture: C.Cartesian3;
  /** The reticle's brightness, 0..1. */
  bright: number;
}

const scratch = new C.Cartesian3();
const s2 = new C.Cartesian2();

/** A small canvas over the view, its own compositor layer. */
function layer(host: HTMLElement): HTMLCanvasElement {
  const c = document.createElement("canvas");
  c.className = "sights-layer";
  c.style.cssText =
    "position:absolute;left:0;top:0;pointer-events:none;will-change:transform,opacity;display:none;";
  host.appendChild(c);
  return c;
}

/** Size `c` to `css` CSS pixels square (at the device's pixel ratio); true if it changed. */
function fit(c: HTMLCanvasElement, css: number): boolean {
  const dpr = devicePixelRatio || 1;
  const px = Math.max(1, Math.ceil(css * dpr));
  if (c.width === px) return false;
  c.width = c.height = px;
  c.style.width = c.style.height = `${px / dpr}px`;
  return true;
}

function show(c: HTMLCanvasElement, on: boolean): void {
  const want = on ? "block" : "none";
  if (c.style.display !== want) c.style.display = want;
}

export class Sights {
  private readonly holo: HTMLCanvasElement;
  private readonly irons: HTMLCanvasElement;
  /** The aperture as last drawn: its hole's radius, CSS px. */
  private hole = 0;

  constructor(
    private readonly world: World,
    host: HTMLElement,
  ) {
    this.holo = layer(host);
    this.irons = layer(host);
  }

  private project(p: C.Cartesian3): C.Cartesian2 | null {
    const r = this.world.scene.cartesianToCanvasCoordinates(p, s2);
    return r ? new C.Cartesian2(r.x, r.y) : null;
  }

  draw(s: SightPicture | null): void {
    const on = !!s && s.ads >= 0.5;
    show(this.holo, on && s?.sight === "holo");
    show(this.irons, on && s?.sight === "irons");
    if (!s || !on) return;
    // Scene canvas coordinates are CSS pixels of the Cesium canvas, which fills the host.
    const scene = this.world.scene;
    // Pixels per radian at the centre.
    const fovy = (this.world.viewer.camera.frustum as C.PerspectiveFrustum).fovy ?? 1;
    const ppr = scene.canvas.clientHeight / (2 * Math.tan(fovy / 2));
    const far = C.Cartesian3.add(
      s.eye,
      C.Cartesian3.multiplyByScalar(s.axis, 2000, scratch),
      scratch,
    );
    const centre = this.project(far);
    if (!centre) return;
    const fade = Math.min(1, (s.ads - 0.5) / 0.4);
    if (s.sight === "holo") this.drawHolo(s, centre, ppr, fade);
    else this.drawIrons(s, centre, ppr, fade);
  }

  /** The reticle, clipped to the window it is seen through. */
  private drawHolo(s: SightPicture, centre: C.Cartesian2, ppr: number, fade: number): void {
    const win = s.window.map((p) => this.project(p));
    if (win.some((p) => !p)) return;
    const c = this.holo;
    const ring = (RING_MOA / 2) * MOA * ppr;
    const line = Math.max(1.2, 1.6 * MOA * ppr);
    const half = Math.ceil(ring + line + 8);
    fit(c, 2 * half);
    const g = c.getContext("2d")!;
    const k = c.width / (2 * half);
    // The canvas' own coordinates: the reticle's centre in its middle.
    const ox = centre.x - half;
    const oy = centre.y - half;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, c.width, c.height);
    g.setTransform(k, 0, 0, k, -ox * k, -oy * k);
    g.save();
    g.beginPath();
    win.forEach((p, i) => (i ? g.lineTo(p!.x, p!.y) : g.moveTo(p!.x, p!.y)));
    g.closePath();
    g.clip();
    g.globalAlpha = 0.55 + 0.45 * s.bright;
    g.shadowColor = "rgba(255,40,30,0.9)";
    g.shadowBlur = 4 * k;
    g.strokeStyle = "rgb(255,45,35)";
    g.fillStyle = "rgb(255,60,45)";
    g.lineWidth = line;
    g.beginPath();
    g.arc(centre.x, centre.y, ring, 0, 2 * Math.PI);
    g.stroke();
    // The ring's four ticks.
    for (const a of [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2]) {
      g.beginPath();
      g.moveTo(centre.x + Math.cos(a) * ring, centre.y + Math.sin(a) * ring);
      g.lineTo(centre.x + Math.cos(a) * ring * 0.86, centre.y + Math.sin(a) * ring * 0.86);
      g.stroke();
    }
    g.beginPath();
    g.arc(centre.x, centre.y, Math.max(1.4, (DOT_MOA / 2) * MOA * ppr), 0, 2 * Math.PI);
    g.fill();
    g.restore();
    c.style.transform = `translate(${ox}px,${oy}px)`;
    c.style.opacity = String(fade);
  }

  /** The aperture: centred where it is, its hole the angle it subtends, soft-edged; the ring
   * round the hole dark, fading out beyond it (the eye sees past the sight). */
  private drawIrons(s: SightPicture, centre: C.Cartesian2, ppr: number, fade: number): void {
    const c = this.irons;
    const at = this.project(s.aperture) ?? centre;
    const dist = C.Cartesian3.distance(s.eye, s.aperture);
    const hole = (APERTURE_M / 2 / Math.max(0.03, dist)) * ppr;
    const outer = Math.ceil(hole * 8);
    if (fit(c, 2 * outer) || Math.abs(hole - this.hole) > 0.5) {
      this.hole = hole;
      const g = c.getContext("2d")!;
      const k = c.width / (2 * outer);
      g.setTransform(k, 0, 0, k, 0, 0);
      g.clearRect(0, 0, 2 * outer, 2 * outer);
      const grad = g.createRadialGradient(outer, outer, hole * 0.85, outer, outer, outer);
      const stop = (r: number) => (r - hole * 0.85) / (outer - hole * 0.85);
      grad.addColorStop(0, "rgba(12,12,14,0)");
      grad.addColorStop(stop(hole * 1.2), "rgba(12,12,14,0.85)");
      grad.addColorStop(stop(hole * 2.5), "rgba(16,16,18,0.8)");
      grad.addColorStop(stop(hole * 4), "rgba(20,20,22,0.12)");
      grad.addColorStop(1, "rgba(20,20,22,0)");
      g.fillStyle = grad;
      g.fillRect(0, 0, 2 * outer, 2 * outer);
    }
    c.style.transform = `translate(${at.x - outer}px,${at.y - outer}px)`;
    c.style.opacity = String(fade);
  }

  dispose(): void {
    this.holo.remove();
    this.irons.remove();
  }
}
