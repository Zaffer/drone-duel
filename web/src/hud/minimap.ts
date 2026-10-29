// The minimap (DESIGN.md §11, bottom right in flight): a north-up 2D map round the vehicle on
// one canvas, no second Cesium viewer. Esri World Imagery tiles under it at the zoom that keeps
// about 400 m across sharp on this screen; the vehicle as an arrow at its heading, its trail,
// the start pad, and whatever marks the mode adds (gates, the next gate, the other player, the
// truck). Marks that matter beyond the edge (the next gate, the peer, the truck) wait on the rim,
// pointing the way.
//
// It draws from the true pose, as the pilot's own glance at a map would; the Map/COP view is the
// ground station's (views/mapview.ts).

import type { Pose } from "../net/server.ts";
import { status } from "../state/session.ts";
import { eulerOf } from "../world/geo.ts";
import "./hud.css";
import {
  clampToEdge,
  metresPerPx,
  niceLength,
  projector,
  TILE,
  tileUrl,
  visibleTiles,
  worldPx,
  zoomFor,
} from "./tiles.ts";

export type MarkKind = "gate" | "next-gate" | "peer" | "truck" | "pad";

export interface Mark {
  lat: number;
  lon: number;
  kind: MarkKind;
  /** Degrees true: a gate's facing, a peer's or the truck's heading. */
  heading?: number;
  label?: string;
}

export interface MinimapOptions {
  /** Round, or a rounded square (the default). */
  round?: boolean;
  /** Metres across the map (about 400). */
  metresAcross?: number;
}

/** Tiles kept loaded. */
const TILE_CACHE = 96;
/** The trail: a point every so often, ms, if it moved this far, m; kept this long, s; a jump
 * further than this, m, is a respawn and starts it again. */
const TRAIL = { everyMs: 250, minM: 1, seconds: 120, jumpM: 300 };
/** Marks that wait on the rim when off the map. */
const PINNED: ReadonlySet<MarkKind> = new Set(["next-gate", "peer", "truck"]);

interface Palette {
  surface: string;
  bg: string;
  line: string;
  text: string;
  muted: string;
  ember: string;
  tide: string;
  sun: string;
  sky: string;
}

function palette(el: Element): Palette {
  const css = getComputedStyle(el);
  const v = (name: string, fallback: string): string =>
    css.getPropertyValue(name).trim() || fallback;
  return {
    surface: v("--surface", "#151724"),
    bg: v("--bg", "#0d0e15"),
    line: v("--line", "#2c3049"),
    text: v("--text", "#eef0f7"),
    muted: v("--muted", "#8e93ad"),
    ember: v("--ember", "#ff6b35"),
    tide: v("--tide", "#22c7b8"),
    sun: v("--sun", "#ffc845"),
    sky: v("--sky", "#6ea8ff"),
  };
}

export class Minimap {
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly tiles = new Map<string, HTMLImageElement>();
  private readonly round: boolean;
  private readonly metresAcross: number;
  private colors: Palette | null = null;
  private marks: Mark[] = [];
  private trail: { lat: number; lon: number; t: number }[] = [];
  private last: Pose | null = null;

  constructor(
    private readonly el: HTMLElement,
    options: MinimapOptions = {},
  ) {
    this.round = options.round ?? false;
    this.metresAcross = options.metresAcross ?? 400;
    el.classList.add("minimap");
    el.classList.toggle("round", this.round);
    this.canvas = document.createElement("canvas");
    el.append(this.canvas);
    this.ctx = this.canvas.getContext("2d")!;
  }

  /** The vehicle's true pose now (null before it has one): draws the map round it. */
  update(pose: Pose | null): void {
    if (pose) this.remember(pose);
    this.last = pose;
    this.draw();
  }

  /** The mode's marks, replacing the last. */
  setMarks(marks: Mark[]): void {
    this.marks = marks;
  }

  dispose(): void {
    for (const img of this.tiles.values()) img.src = "";
    this.tiles.clear();
    this.canvas.remove();
    this.el.classList.remove("minimap", "round");
  }

  // -- inside --------------------------------------------------------------------------------

  private remember(p: Pose): void {
    const now = performance.now();
    const trail = this.trail;
    const last = trail[trail.length - 1];
    if (last) {
      const m = metresBetween(last.lat, last.lon, p.lat, p.lon);
      if (m > TRAIL.jumpM) trail.length = 0;
      else if (now - last.t < TRAIL.everyMs || m < TRAIL.minM) return;
    }
    trail.push({ lat: p.lat, lon: p.lon, t: now });
    while (trail.length && now - trail[0]!.t > TRAIL.seconds * 1000) trail.shift();
  }

  /** The map's centre: the vehicle, or the pad before it has a pose. */
  private centre(): [number, number] | null {
    if (this.last) return [this.last.lat, this.last.lon];
    const origin = status().origin;
    if (origin) return [origin[0], origin[1]];
    const pad = this.marks.find((m) => m.kind === "pad");
    return pad ? [pad.lat, pad.lon] : null;
  }

  private draw(): void {
    if (!this.colors) this.colors = palette(this.el);
    const c = this.colors;
    const dpr = window.devicePixelRatio || 1;
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (!w || !h) return;
    const W = Math.round(w * dpr);
    const H = Math.round(h * dpr);
    if (this.canvas.width !== W || this.canvas.height !== H) {
      this.canvas.width = W;
      this.canvas.height = H;
    }
    const g = this.ctx;
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.fillStyle = c.surface;
    g.fillRect(0, 0, w, h);
    const centre = this.centre();
    if (!centre) {
      g.fillStyle = c.muted;
      g.font = "600 11px 'JetBrains Mono', monospace";
      g.textAlign = "center";
      g.fillText("NO POSITION", w / 2, h / 2 + 4);
      return;
    }
    const [lat, lon] = centre;
    const across = Math.min(w, h);
    const z = zoomFor(lat, this.metresAcross, across, dpr);
    // CSS px per map px at z.
    const scale = metresPerPx(lat, z) / (this.metresAcross / across);
    const project = projector(lat, lon, z, scale);
    g.save();
    g.translate(w / 2, h / 2);
    this.drawTiles(lat, lon, z, scale, w, h);
    g.fillStyle = c.bg;
    g.globalAlpha = 0.18;
    g.fillRect(-w / 2, -h / 2, w, h);
    g.globalAlpha = 1;
    this.drawTrail(project, c);
    const r = across / 2;
    for (const m of this.marks) this.drawMark(m, project, r, c);
    if (this.last) this.drawVehicle(this.last, c);
    g.restore();
    this.drawFurniture(lat, z, scale, w, h, c);
  }

  private tile(z: number, x: number, y: number): HTMLImageElement | null {
    const key = `${z}/${x}/${y}`;
    let img = this.tiles.get(key);
    if (img) {
      // Most recently used last.
      this.tiles.delete(key);
      this.tiles.set(key, img);
    } else {
      img = new Image();
      img.decoding = "async";
      img.src = tileUrl(z, x, y);
      this.tiles.set(key, img);
      while (this.tiles.size > TILE_CACHE) {
        const oldest = this.tiles.keys().next().value as string;
        this.tiles.delete(oldest);
      }
    }
    return img.complete && img.naturalWidth > 0 ? img : null;
  }

  private drawTiles(
    lat: number,
    lon: number,
    z: number,
    scale: number,
    w: number,
    h: number,
  ): void {
    const g = this.ctx;
    const [cx, cy] = worldPx(lat, lon, z);
    const size = TILE * scale;
    g.imageSmoothingEnabled = true;
    g.imageSmoothingQuality = "high";
    for (const t of visibleTiles(cx, cy, w / 2 / scale, h / 2 / scale, z)) {
      const x = t.dx * scale;
      const y = t.dy * scale;
      const img = this.tile(t.z, t.x, t.y);
      if (img) {
        // A hair over, so neighbours meet with no seam.
        g.drawImage(img, x, y, size + 0.5, size + 0.5);
        continue;
      }
      // Still loading: the quarter of its parent, if that is here.
      if (t.z === 0) continue;
      const parent = this.tile(t.z - 1, t.x >> 1, t.y >> 1);
      if (!parent) continue;
      const half = TILE / 2;
      g.drawImage(
        parent,
        (t.x & 1) * half,
        (t.y & 1) * half,
        half,
        half,
        x,
        y,
        size + 0.5,
        size + 0.5,
      );
    }
  }

  private drawTrail(project: (lat: number, lon: number) => [number, number], c: Palette): void {
    const g = this.ctx;
    const pts = this.trail.map((p) => project(p.lat, p.lon));
    if (this.last) pts.push(project(this.last.lat, this.last.lon));
    if (pts.length < 2) return;
    g.lineJoin = "round";
    g.lineCap = "round";
    g.strokeStyle = c.bg;
    g.globalAlpha = 0.55;
    g.lineWidth = 4;
    this.polyline(pts);
    g.globalAlpha = 1;
    g.strokeStyle = c.sky;
    g.lineWidth = 2;
    this.polyline(pts);
  }

  private polyline(pts: [number, number][]): void {
    const g = this.ctx;
    g.beginPath();
    for (const [i, [x, y]] of pts.entries()) {
      if (i) g.lineTo(x, y);
      else g.moveTo(x, y);
    }
    g.stroke();
  }

  private drawMark(
    m: Mark,
    project: (lat: number, lon: number) => [number, number],
    r: number,
    c: Palette,
  ): void {
    const g = this.ctx;
    const [px, py] = project(m.lat, m.lon);
    const edge = clampToEdge(px, py, r, this.round, 9);
    if (edge.clamped && !PINNED.has(m.kind)) return;
    const { x, y } = edge;
    const color =
      m.kind === "next-gate" ? c.sun : m.kind === "peer" || m.kind === "truck" ? c.tide : c.text;
    g.save();
    g.translate(x, y);
    g.lineWidth = 2;
    g.strokeStyle = color;
    g.fillStyle = color;
    if (edge.clamped) {
      // A chevron on the rim, pointing out toward it.
      g.rotate(Math.atan2(py, px));
      g.beginPath();
      g.moveTo(5, 0);
      g.lineTo(-4, -5);
      g.lineTo(-1, 0);
      g.lineTo(-4, 5);
      g.closePath();
      g.fill();
      g.restore();
      return;
    }
    const heading = ((m.heading ?? 0) * Math.PI) / 180;
    switch (m.kind) {
      case "pad": {
        g.strokeStyle = c.bg;
        g.lineWidth = 4;
        g.beginPath();
        g.arc(0, 0, 6, 0, 2 * Math.PI);
        g.stroke();
        g.strokeStyle = color;
        g.lineWidth = 1.5;
        g.stroke();
        g.font = "700 8px 'JetBrains Mono', monospace";
        g.textAlign = "center";
        g.textBaseline = "middle";
        g.fillText("H", 0, 0.5);
        break;
      }
      case "gate":
      case "next-gate": {
        // A gate seen from above: a bar across its facing.
        g.rotate(heading);
        const half = m.kind === "next-gate" ? 7 : 5;
        g.lineCap = "round";
        g.strokeStyle = c.bg;
        g.lineWidth = m.kind === "next-gate" ? 6 : 4;
        g.beginPath();
        g.moveTo(-half, 0);
        g.lineTo(half, 0);
        g.stroke();
        g.strokeStyle = color;
        g.lineWidth = m.kind === "next-gate" ? 3 : 2;
        g.stroke();
        break;
      }
      case "peer":
        g.rotate(heading);
        arrow(g, 6, color, c.bg);
        break;
      case "truck":
        g.rotate(heading);
        g.fillStyle = c.bg;
        g.fillRect(-4.5, -6.5, 9, 13);
        g.fillStyle = color;
        g.fillRect(-3, -5, 6, 10);
        break;
    }
    g.restore();
    if (m.label) {
      g.font = "600 10px Inter, sans-serif";
      g.textAlign = "left";
      g.textBaseline = "middle";
      g.lineWidth = 3;
      g.strokeStyle = c.bg;
      g.strokeText(m.label, x + 9, y);
      g.fillStyle = color;
      g.fillText(m.label, x + 9, y);
    }
  }

  private drawVehicle(p: Pose, c: Palette): void {
    const g = this.ctx;
    g.save();
    g.rotate(eulerOf(p.q).heading);
    arrow(g, 8, c.ember, c.bg);
    g.restore();
  }

  /** North, and a scale bar. */
  private drawFurniture(
    lat: number,
    z: number,
    scale: number,
    w: number,
    h: number,
    c: Palette,
  ): void {
    const g = this.ctx;
    g.font = "700 10px 'JetBrains Mono', monospace";
    g.textAlign = "center";
    g.textBaseline = "top";
    g.lineWidth = 3;
    g.strokeStyle = c.bg;
    g.strokeText("N", w / 2, 5);
    g.fillStyle = c.text;
    g.fillText("N", w / 2, 5);
    if (this.round) return;
    const mPerCss = metresPerPx(lat, z) / scale;
    const m = niceLength(w * 0.3 * mPerCss);
    const len = m / mPerCss;
    const x = 10;
    const y = h - 10;
    g.strokeStyle = c.bg;
    g.lineWidth = 4;
    g.beginPath();
    g.moveTo(x, y - 4);
    g.lineTo(x, y);
    g.lineTo(x + len, y);
    g.lineTo(x + len, y - 4);
    g.stroke();
    g.strokeStyle = c.text;
    g.lineWidth = 1.5;
    g.stroke();
    g.font = "600 9px 'JetBrains Mono', monospace";
    g.textAlign = "left";
    g.textBaseline = "bottom";
    g.lineWidth = 3;
    g.strokeStyle = c.bg;
    const text = m >= 1000 ? `${m / 1000} km` : `${m} m`;
    g.strokeText(text, x + 4, y - 3);
    g.fillStyle = c.text;
    g.fillText(text, x + 4, y - 3);
  }
}

/** An arrow pointing up (north before rotation), `size` px from its centre to its tip. */
function arrow(g: CanvasRenderingContext2D, size: number, fill: string, outline: string): void {
  g.beginPath();
  g.moveTo(0, -size);
  g.lineTo(size * 0.72, size * 0.8);
  g.lineTo(0, size * 0.38);
  g.lineTo(-size * 0.72, size * 0.8);
  g.closePath();
  g.lineJoin = "round";
  g.lineWidth = 3;
  g.strokeStyle = outline;
  g.stroke();
  g.fillStyle = fill;
  g.fill();
}

/** Metres between two nearby points (flat earth). */
function metresBetween(lat0: number, lon0: number, lat1: number, lon1: number): number {
  const R = 6378137;
  const dn = ((lat1 - lat0) * Math.PI * R) / 180;
  const de = ((lon1 - lon0) * Math.PI * R * Math.cos((lat0 * Math.PI) / 180)) / 180;
  return Math.hypot(dn, de);
}
