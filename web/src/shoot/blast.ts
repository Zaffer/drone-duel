// A charge going off (Shoot Down's payload, DESIGN.md §7.1): a flash, a fireball that swells and
// dies in a few tenths of a second, fragments flying out, smoke rising and spreading for some
// seconds, and dust thrown up if it is near the ground; heard when its sound gets back
// (gunsounds.boom). Drawn in the place's local frame (engine/frame.ts), through world.add.

import * as C from "cesium";
import type { LocalFrame } from "../engine/frame.ts";
import type { Vec3 } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { gunSounds } from "./gunsounds.ts";

type Kind = "flash" | "fire" | "smoke" | "frag" | "dust";

interface Puff {
  b: C.Billboard;
  kind: Kind;
  born: number;
  /** ms. */
  life: number;
  at: Vec3;
  v: Vec3;
  /** Size, m: from, to. */
  size: [number, number];
  /** Falls (fragments) or rises (smoke), m/s². */
  g: number;
}

const FRAGMENTS = 24;
const SMOKE = 10;
/** Below this over the ground, it throws up dust, m. */
const DUST_UNDER_M = 4;

export class Blasts {
  private puffs: C.BillboardCollection;
  private live: Puff[] = [];
  private readonly sprites: Record<Kind, HTMLCanvasElement>;

  constructor(
    private readonly world: World,
    private readonly frame: LocalFrame,
  ) {
    this.puffs = world.add(new C.BillboardCollection()) as C.BillboardCollection;
    this.sprites = {
      flash: sprite([
        [0, "rgba(255,255,240,1)"],
        [0.35, "rgba(255,230,160,0.9)"],
        [1, "rgba(255,180,80,0)"],
      ]),
      fire: sprite([
        [0, "rgba(255,230,150,1)"],
        [0.4, "rgba(255,140,40,0.9)"],
        [0.75, "rgba(170,60,20,0.5)"],
        [1, "rgba(80,30,10,0)"],
      ]),
      smoke: sprite([
        [0, "rgba(55,52,50,0.85)"],
        [0.6, "rgba(70,66,62,0.5)"],
        [1, "rgba(80,76,72,0)"],
      ]),
      frag: sprite([
        [0, "rgba(255,240,200,1)"],
        [0.5, "rgba(255,160,60,0.7)"],
        [1, "rgba(255,120,20,0)"],
      ]),
      dust: sprite([
        [0, "rgba(140,120,95,0.8)"],
        [1, "rgba(140,120,95,0)"],
      ]),
    };
  }

  /** A charge going off at `at` (local), `overGround` m above the ground there; heard at `ear`
   * (`pan` -1 left .. 1 right). */
  burst(at: Vec3, overGround: number, ear: Vec3, pan: number, now: number): void {
    this.add("flash", at, now, 90, [0, 0, 0], [3, 6], 0);
    for (let i = 0; i < 4; i++) {
      this.add("fire", jitter(at, 0.4), now, 450 + Math.random() * 250, rand3(2), [1, 4.5], -2);
    }
    for (let i = 0; i < FRAGMENTS; i++) {
      const v = rand3(1);
      const l = Math.hypot(...v) || 1;
      const speed = 40 + Math.random() * 60;
      const out: Vec3 = [(v[0] / l) * speed, (v[1] / l) * speed, (v[2] / l) * speed];
      this.add("frag", at, now, 250 + Math.random() * 300, out, [0.15, 0.08], 9.8);
    }
    for (let i = 0; i < SMOKE; i++) {
      const v: Vec3 = [rnd(1.2), rnd(1.2), 0.6 + Math.random() * 1.2];
      this.add("smoke", jitter(at, 0.8), now + 80, 4000 + Math.random() * 3000, v, [1.5, 7], -0.2);
    }
    if (overGround < DUST_UNDER_M) {
      const ground: Vec3 = [at[0], at[1], at[2] - overGround];
      for (let i = 0; i < 8; i++) {
        const v: Vec3 = [rnd(5), rnd(5), 1 + Math.random() * 3];
        this.add("dust", jitter(ground, 1), now, 2500 + Math.random() * 1500, v, [1, 5], 3);
      }
    }
    const range = Math.hypot(at[0] - ear[0], at[1] - ear[1], at[2] - ear[2]);
    gunSounds.boom(range, pan);
  }

  private add(
    kind: Kind,
    at: Vec3,
    born: number,
    life: number,
    v: Vec3,
    size: [number, number],
    g: number,
  ): void {
    const b = this.puffs.add({
      image: this.sprites[kind],
      position: this.frame.fixed(at),
      show: false,
    });
    b.sizeInMeters = true;
    b.width = b.height = size[0];
    this.live.push({ b, kind, born, life, at: [...at], v, size, g });
  }

  /** Each frame. */
  update(now: number): void {
    const keep: Puff[] = [];
    for (const p of this.live) {
      const age = now - p.born;
      if (age >= p.life) {
        this.puffs.remove(p.b);
        continue;
      }
      keep.push(p);
      if (age < 0) continue;
      const t = age / 1000;
      // Smoke and dust slow as they spread: their speed falls off over a second or so.
      const drag = p.kind === "smoke" || p.kind === "dust" ? 1 - Math.exp(-t) : t;
      const pos: Vec3 = [
        p.at[0] + p.v[0] * drag,
        p.at[1] + p.v[1] * drag,
        p.at[2] + p.v[2] * drag - 0.5 * p.g * t * t,
      ];
      const k = age / p.life;
      const grow = p.kind === "fire" || p.kind === "flash" ? Math.sqrt(k) : k;
      const size = p.size[0] + (p.size[1] - p.size[0]) * grow;
      p.b.show = true;
      p.b.position = this.frame.fixed(pos);
      p.b.width = p.b.height = size;
      p.b.color = C.Color.WHITE.withAlpha(
        p.kind === "smoke" ? Math.min(1, 4 * k) * (1 - k) * 1.6 : 1 - k,
      );
    }
    this.live = keep;
  }

  dispose(): void {
    this.world.remove(this.puffs);
    this.live = [];
  }
}

function sprite(stops: [number, string][]): HTMLCanvasElement {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d")!;
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  for (const [at, colour] of stops) grad.addColorStop(at, colour);
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  return c;
}

function rnd(k: number): number {
  return (Math.random() * 2 - 1) * k;
}

function rand3(k: number): Vec3 {
  return [rnd(k), rnd(k), rnd(k)];
}

function jitter(p: Vec3, k: number): Vec3 {
  return [p[0] + rnd(k), p[1] + rnd(k), p[2] + rnd(k)];
}
