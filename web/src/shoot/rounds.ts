// The rounds in flight (DESIGN.md §7, Shoot Down). Each is ballistics.ts's Round, stepped in
// real time from when it left the muzzle, and checked step by step against the drone as the
// shooter sees it at that moment (its body's poseAt: the drawn pose, favouring the shooter, as
// the duel's strikes are judged) and, each frame, its way since the last against the ground: a
// segment cast through the place's colliders (engine/ground.ts), microseconds, the point where
// it went in exact. A strike sends the hit to the physics on the server, which does the rest
// (part damage, the push).
//
// The ground is built only near the shooter and the drone (a few hundred metres); past
// COLLIDE_M from the shooter a round is no longer checked against it (there is nothing there to
// meet) but flies on, tracer and all, until it is spent.
//
// Positions are in the place's local frame (engine/frame.ts): east, north, up, metres.

import * as C from "cesium";
import type { Ground } from "../engine/ground.ts";
import type { HitMsg } from "../net/protocol.ts";
import { link, type Pose } from "../net/server.ts";
import { type Quat, rotate, type Vec3 } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { type Load, Round, type Setting, STEP_S } from "./ballistics.ts";
import { gunSounds } from "./gunsounds.ts";
import { MOMENTUM_SHARE, type Strike, strikes } from "./target.ts";

/** A round is lost once this old, s, this slow, m/s, or this far, m. */
const MAX_S = 5;
const MIN_SPEED = 120;
const MAX_RANGE = 3000;
/** Past this far from the shooter a round meets no ground: none is built there, m. */
const COLLIDE_M = 400;
const TRACER_COLOR = C.Color.fromCssColorString("#ff7a2e");
/** A tracer's streak, s of flight behind it; and it shows from this far, m (M196 lights near the
 * muzzle; nearer than this it is behind the shooter's eye anyway). */
const TRACER_TAIL_S = 0.02;
const TRACER_FROM_M = 25;

export interface Flying {
  round: Round;
  /** performance.now() when it left the muzzle. */
  at: number;
  tracer: boolean;
  /** What it has hit, by part and motor: a part once. */
  hit: Set<string>;
  line?: C.Polyline;
  /** The tracer's burning base: seen from behind, a bright dot more than a streak. */
  head?: C.Billboard;
  /** Where it was when the ground was last looked along its way. */
  checked: Vec3;
  /** Where it left the muzzle. */
  muzzle: Vec3;
}

export interface HitReport {
  strike: Strike;
  /** Where, the local frame. */
  where: Vec3;
  speed: number;
  range: number;
}

export class Rounds {
  private flying: Flying[] = [];
  private lines: C.PolylineCollection;
  private puffs: C.BillboardCollection;
  private live: { b: C.Billboard; born: number; life: number; v: Vec3; grow: number; at: Vec3 }[] =
    [];
  private sprites: { dust: HTMLCanvasElement; spark: HTMLCanvasElement; flash: HTMLCanvasElement };
  private flash: C.Billboard | null = null;
  private flashUntil = 0;
  /** Whether the clear tracer and sprites are drawn (`keepShaders`). */
  private kept = false;
  /** Where the shooter's ear is, the local frame, and their right (for panning). */
  ear: Vec3 = [0, 0, 1.6];
  earRight: Vec3 = [1, 0, 0];
  onHit: ((h: HitReport) => void) | null = null;
  /** Rounds that went into the ground. */
  groundHits = 0;

  constructor(
    private readonly world: World,
    private readonly ground: Ground,
    private readonly where: Setting,
    private readonly target: () => ((now: number) => Pose | null) | null,
  ) {
    this.lines = world.add(new C.PolylineCollection()) as C.PolylineCollection;
    this.puffs = world.add(new C.BillboardCollection()) as C.BillboardCollection;
    this.sprites = { dust: sprite("dust"), spark: sprite("spark"), flash: sprite("flash") };
  }

  /** A shot leaves `muzzle` along `dir` (local frame) at `speed`. */
  fire(load: Load, muzzle: Vec3, dir: Vec3, speed: number, tracer: boolean, now: number): void {
    const round = new Round(load, this.where, muzzle, dir, speed);
    const f: Flying = {
      round,
      at: now,
      tracer,
      hit: new Set(),
      checked: [...muzzle],
      muzzle: [...muzzle],
    };
    if (tracer) {
      f.line = this.lines.add({
        positions: [this.fixed(muzzle), this.fixed(muzzle)],
        width: 3,
        material: tracerGlow(TRACER_COLOR),
        show: false,
      });
      f.head = this.puffs.add({
        image: this.sprites.spark,
        position: this.fixed(muzzle),
        width: 7,
        height: 7,
        show: false,
      });
    }
    this.flying.push(f);
    this.muzzleFlash(muzzle, now);
  }

  /** A tracer, a muzzle flash, a spark and dust, clear (alpha 0), drawn ahead of the eye along
   * `look` and kept for good. Cesium compiles a material's shaders the first time it is drawn,
   * and frees them once nothing uses them: the first shot, and the first after a string's
   * tracers had all burnt out, waited 150 to 350 ms for the tracer's to be linked again
   * (measured 2026-11-06, headless on the 4090; less when the GPU's own cache had them).
   * Called each frame from the first the eye is placed; does it once. */
  keepShaders(eye: Vec3, look: Vec3): void {
    if (this.kept) return;
    this.kept = true;
    const at = (d: number): C.Cartesian3 =>
      this.fixed([eye[0] + look[0] * d, eye[1] + look[1] * d, eye[2] + look[2] * d]);
    const clear = C.Color.WHITE.withAlpha(0);
    this.lines.add({
      positions: [at(30), at(31)],
      width: 3,
      material: tracerGlow(TRACER_COLOR.withAlpha(0)),
    });
    for (const kind of ["spark", "flash", "dust"] as const) {
      const b = this.puffs.add({ image: this.sprites[kind], position: at(30), color: clear });
      b.sizeInMeters = true;
      b.width = b.height = 0.1;
    }
  }

  /** Each frame: on to `now`, the drone and the ground in the way of each. */
  update(now: number): void {
    const poseAt = this.target();
    const keep: Flying[] = [];
    for (const f of this.flying) {
      if (this.advance(f, now, poseAt) && !this.intoGround(f, now)) keep.push(f);
      else this.drop(f);
    }
    this.flying = keep;
    this.animate(now);
  }

  private drop(f: Flying): void {
    if (f.line) this.lines.remove(f.line);
    if (f.head) this.puffs.remove(f.head);
  }

  /** Whether its way since last looked meets the ground: it stops there, in a puff of dust. */
  private intoGround(f: Flying, now: number): boolean {
    const a = f.checked;
    const b = f.round.at();
    f.checked = b;
    const far = Math.hypot(a[0] - this.ear[0], a[1] - this.ear[1]);
    if (far > COLLIDE_M) return false;
    const hit = this.ground.segment(a, b);
    if (!hit) return false;
    this.groundHit(hit.point, now);
    this.groundHits++;
    return true;
  }

  get count(): number {
    return this.flying.length;
  }

  /** On to `now`; false once it is done. */
  private advance(f: Flying, now: number, poseAt: ((now: number) => Pose | null) | null): boolean {
    const r = f.round;
    const until = (now - f.at) / 1000;
    while (r.t < until) {
      const a = r.at();
      r.step(STEP_S);
      const b = r.at();
      if (poseAt) this.checkDrone(f, a, b, poseAt(f.at + r.t * 1000));
      const range = Math.hypot(b[0] - f.muzzle[0], b[1] - f.muzzle[1]);
      if (r.t > MAX_S || r.speed < MIN_SPEED || range > MAX_RANGE) return false;
    }
    if (f.line) {
      const p = r.at();
      const tail: Vec3 = [
        p[0] - r.v[0] * TRACER_TAIL_S,
        p[1] - r.v[1] * TRACER_TAIL_S,
        p[2] - r.v[2] * TRACER_TAIL_S,
      ];
      f.line.positions = [this.fixed(tail), this.fixed(p)];
      const lit =
        Math.hypot(p[0] - this.ear[0], p[1] - this.ear[1], p[2] - this.ear[2]) > TRACER_FROM_M;
      f.line.show = lit;
      if (f.head) {
        f.head.position = this.fixed(p);
        f.head.show = lit;
      }
    }
    return true;
  }

  private checkDrone(f: Flying, a: Vec3, b: Vec3, pose: Pose | null): void {
    if (!pose) return;
    const d = this.ground.frame.fromSim(pose.lat, pose.lon, pose.alt);
    // Quick: is the step anywhere near?
    const mid: Vec3 = [
      (a[0] + b[0]) / 2 - d[0],
      (a[1] + b[1]) / 2 - d[1],
      (a[2] + b[2]) / 2 - d[2],
    ];
    const half = Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]) / 2;
    if (Math.hypot(...mid) > half + 0.3) return;
    const toBody = (p: Vec3): Vec3 => {
      const ned: Vec3 = [p[1] - d[1], p[0] - d[0], -(p[2] - d[2])];
      return rotate(conj(pose.q), ned);
    };
    for (const s of strikes(toBody(a), toBody(b))) {
      const key = `${s.part}:${s.motor}`;
      if (f.hit.has(key)) continue;
      f.hit.add(key);
      const r = f.round;
      const speed = r.speed;
      const share = MOMENTUM_SHARE[s.part];
      const j = share * r.load.massKg * speed;
      const v = r.v;
      const impulseNed: [number, number, number] = [
        (v[1] / speed) * j,
        (v[0] / speed) * j,
        (-v[2] / speed) * j,
      ];
      const msg: HitMsg = {
        ch: "hit",
        part: s.part,
        motor: s.motor,
        impulse_ned: impulseNed,
        at_frd: s.at,
      };
      link.send(msg);
      r.slow(share * speed);
      const where: Vec3 = [
        a[0] + (b[0] - a[0]) * s.s,
        a[1] + (b[1] - a[1]) * s.s,
        a[2] + (b[2] - a[2]) * s.s,
      ];
      const range = Math.hypot(
        where[0] - this.ear[0],
        where[1] - this.ear[1],
        where[2] - this.ear[2],
      );
      this.spark(where);
      gunSounds.hit(range, this.pan(where), s.part === "motor" || s.part === "battery");
      this.onHit?.({ strike: s, where, speed, range });
    }
  }

  private groundHit(p: Vec3, now: number): void {
    const range = Math.hypot(p[0] - this.ear[0], p[1] - this.ear[1]);
    for (let i = 0; i < 6; i++) {
      this.puff(
        p,
        "dust",
        now,
        1.2 + Math.random(),
        [rnd(0.6), rnd(0.6), 0.8 + Math.random() * 1.5],
        1.6,
      );
    }
    gunSounds.thud(range, this.pan(p));
  }

  private spark(p: Vec3): void {
    const now = performance.now();
    for (let i = 0; i < 8; i++) {
      this.puff(p, "spark", now, 0.25 + Math.random() * 0.2, [rnd(6), rnd(6), rnd(6)], 0.4);
    }
    this.puff(p, "dust", now, 0.9, [0, 0, 0.3], 0.8);
  }

  private muzzleFlash(p: Vec3, now: number): void {
    if (!this.flash) {
      this.flash = this.puffs.add({
        image: this.sprites.flash,
        position: this.fixed(p),
        width: 1,
        height: 1,
      });
      this.flash.sizeInMeters = true;
    }
    this.flash.position = this.fixed(p);
    const s = 0.12 + Math.random() * 0.06;
    this.flash.width = s;
    this.flash.height = s;
    this.flash.show = true;
    this.flashUntil = now + 30;
  }

  private puff(
    at: Vec3,
    kind: "dust" | "spark",
    now: number,
    life: number,
    v: Vec3,
    grow: number,
  ): void {
    const b = this.puffs.add({
      image: this.sprites[kind],
      position: this.fixed(at),
      width: kind === "spark" ? 0.06 : 0.4,
      height: kind === "spark" ? 0.06 : 0.4,
    });
    b.sizeInMeters = true;
    this.live.push({ b, born: now, life: life * 1000, v, grow, at: [...at] });
  }

  private animate(now: number): void {
    if (this.flash && now > this.flashUntil) this.flash.show = false;
    const keep: typeof this.live = [];
    for (const p of this.live) {
      const age = now - p.born;
      if (age >= p.life) {
        this.puffs.remove(p.b);
        continue;
      }
      const t = age / 1000;
      const pos: Vec3 = [
        p.at[0] + p.v[0] * t,
        p.at[1] + p.v[1] * t,
        p.at[2] + p.v[2] * t - (p.grow < 1 ? 4.9 * t * t : 0),
      ];
      p.b.position = this.fixed(pos);
      const k = age / p.life;
      const size = (p.grow < 1 ? 0.06 : 0.4) * (1 + p.grow * k * 3);
      p.b.width = size;
      p.b.height = size;
      p.b.color = C.Color.WHITE.withAlpha(1 - k);
      keep.push(p);
    }
    this.live = keep;
  }

  /** -1 left .. 1 right of the shooter. */
  private pan(p: Vec3): number {
    const d: Vec3 = [p[0] - this.ear[0], p[1] - this.ear[1], p[2] - this.ear[2]];
    const l = Math.hypot(...d) || 1;
    return (d[0] * this.earRight[0] + d[1] * this.earRight[1] + d[2] * this.earRight[2]) / l;
  }

  /** A local point, earth-fixed. */
  fixed(p: Vec3): C.Cartesian3 {
    return this.ground.frame.fixed(p);
  }

  dispose(): void {
    this.world.remove(this.lines);
    this.world.remove(this.puffs);
    this.flying = [];
  }
}

function tracerGlow(color: C.Color): C.Material {
  return C.Material.fromType("PolylineGlow", { color, glowPower: 0.35, taperPower: 0.6 });
}

function conj(q: Quat): Quat {
  return [q[0], -q[1], -q[2], -q[3]];
}

function rnd(k: number): number {
  return (Math.random() * 2 - 1) * k;
}

/** The sprites: soft dust, a hot spark, the muzzle's flash. */
function sprite(kind: "dust" | "spark" | "flash"): HTMLCanvasElement {
  const c = document.createElement("canvas");
  c.width = c.height = 64;
  const g = c.getContext("2d")!;
  const grad = g.createRadialGradient(32, 32, 0, 32, 32, 32);
  if (kind === "dust") {
    grad.addColorStop(0, "rgba(150,130,105,0.75)");
    grad.addColorStop(1, "rgba(150,130,105,0)");
  } else if (kind === "spark") {
    grad.addColorStop(0, "rgba(255,240,200,1)");
    grad.addColorStop(0.4, "rgba(255,170,60,0.8)");
    grad.addColorStop(1, "rgba(255,120,20,0)");
  } else {
    grad.addColorStop(0, "rgba(255,250,220,1)");
    grad.addColorStop(0.3, "rgba(255,190,90,0.9)");
    grad.addColorStop(1, "rgba(255,120,30,0)");
  }
  g.fillStyle = grad;
  g.fillRect(0, 0, 64, 64);
  return c;
}
