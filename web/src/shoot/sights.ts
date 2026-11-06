// The sight picture over the view (DESIGN.md §7, Shoot Down). A holographic sight's reticle is
// a hologram focused at infinity: it is seen along the sight's own axis, wherever the eye is
// behind the window, and only through the window. EOTech's EXPS: a 68 MOA ring round a 1 MOA dot.
// Iron sights: the rear aperture, a few centimetres from the eye, is out of focus, a soft dark
// ring with a bright hole; the front post is the model's own, in the 3D view.

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

export class Sights {
  readonly canvas: HTMLCanvasElement;
  private g: CanvasRenderingContext2D;

  constructor(
    private readonly world: World,
    host: HTMLElement,
  ) {
    this.canvas = document.createElement("canvas");
    this.canvas.className = "sights-layer";
    this.canvas.style.cssText =
      "position:absolute;inset:0;width:100%;height:100%;pointer-events:none;";
    host.appendChild(this.canvas);
    this.g = this.canvas.getContext("2d")!;
  }

  private project(p: C.Cartesian3): C.Cartesian2 | null {
    const r = this.world.scene.cartesianToCanvasCoordinates(p, s2);
    return r ? new C.Cartesian2(r.x, r.y) : null;
  }

  draw(s: SightPicture | null): void {
    const canvas = this.canvas;
    const g = this.g;
    const dpr = devicePixelRatio || 1;
    const w = Math.round(canvas.clientWidth * dpr);
    const h = Math.round(canvas.clientHeight * dpr);
    if (canvas.width !== w || canvas.height !== h) {
      canvas.width = w;
      canvas.height = h;
    }
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.clearRect(0, 0, w, h);
    if (!s || s.ads < 0.5) return;
    // Scene canvas coordinates are CSS pixels of the Cesium canvas.
    const k = w / Math.max(1, this.world.scene.canvas.clientWidth);
    g.setTransform(k, 0, 0, k, 0, 0);
    // Pixels per radian at the centre.
    const fovy = (this.world.viewer.camera.frustum as C.PerspectiveFrustum).fovy ?? 1;
    const ppr = this.world.scene.canvas.clientHeight / (2 * Math.tan(fovy / 2));
    const far = C.Cartesian3.add(
      s.eye,
      C.Cartesian3.multiplyByScalar(s.axis, 2000, scratch),
      scratch,
    );
    const centre = this.project(far);
    if (!centre) return;
    const fade = Math.min(1, (s.ads - 0.5) / 0.4);
    if (s.sight === "holo") {
      const win = s.window.map((p) => this.project(p));
      if (win.some((p) => !p)) return;
      g.save();
      g.beginPath();
      win.forEach((p, i) => (i ? g.lineTo(p!.x, p!.y) : g.moveTo(p!.x, p!.y)));
      g.closePath();
      g.clip();
      const alpha = 0.55 + 0.45 * s.bright;
      g.globalAlpha = alpha * fade;
      g.shadowColor = "rgba(255,40,30,0.9)";
      g.shadowBlur = 4;
      g.strokeStyle = "rgb(255,45,35)";
      g.fillStyle = "rgb(255,60,45)";
      const ring = (RING_MOA / 2) * MOA * ppr;
      g.lineWidth = Math.max(1.2, 1.6 * MOA * ppr);
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
    } else {
      // The aperture: centred where it is, its hole the angle it subtends, soft-edged.
      const at = this.project(s.aperture) ?? centre;
      const dist = C.Cartesian3.distance(s.eye, s.aperture);
      const hole = (APERTURE_M / 2 / Math.max(0.03, dist)) * ppr;
      const cw = this.world.scene.canvas.clientWidth;
      const ch = this.world.scene.canvas.clientHeight;
      // The ring round the hole is dark and soft; beyond it the eye sees past the sight.
      const grad = g.createRadialGradient(at.x, at.y, hole * 0.85, at.x, at.y, hole * 4);
      grad.addColorStop(0, "rgba(12,12,14,0)");
      grad.addColorStop(0.12, `rgba(12,12,14,${0.85 * fade})`);
      grad.addColorStop(0.55, `rgba(16,16,18,${0.8 * fade})`);
      grad.addColorStop(1, `rgba(20,20,22,${0.12 * fade})`);
      g.fillStyle = grad;
      g.fillRect(0, 0, cw, ch);
    }
  }

  dispose(): void {
    this.canvas.remove();
  }
}
