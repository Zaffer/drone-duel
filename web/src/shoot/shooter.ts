// The person on the ground with the rifle (DESIGN.md §7, Shoot Down): where they stand and how,
// where they look, the rifle in their hands, and the shot from the trigger to the muzzle.
//
// The aim has three layers. Where the shooter means to point (the mouse or the right stick);
// what their hold adds (hold.ts: breathing, wobble, heart, the trigger's jerk); and what the
// last shots' recoil adds (recoil.ts). The rifle points where all three put it. On the sight,
// the eye rides on the stock (a cheek weld) and sees what the rifle does; head up, the eye looks
// where the shooter means and the rifle moves in front of it.
//
// Controls (keyboard and mouse, or a standard gamepad):
//   mouse / right stick      look and aim
//   right button / LT         on the sight (held, or a toggle in the options)
//   left button / RT          fire (semi-automatic: a press a shot)
//   R / X                     reload
//   W A S D / left stick      walk; Shift / L3 with forward sprints (the rifle comes down)
//   Space / A                 jump (standing)
//   Shift / RB on the sight   hold the breath
//   C / B                     kneel or stand (B steps standing, kneeling, prone)
//   Z                         prone or stand
//   F / LB                    the rangefinder
//   I / Y                     holo or iron sights

import * as C from "cesium";
import type { Weapon } from "../net/protocol.ts";
import { paused } from "../state/app.ts";
import { prefs, updatePrefs } from "../state/prefs.ts";
import {
  bodyAxesEnu,
  mapOffset,
  moveBy,
  neFrom,
  type Quat,
  quatOf,
  rotate,
  type Vec3,
} from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { type Load, loadOf, type Setting, zeroAngle } from "./ballistics.ts";
import { gunSounds } from "./gunsounds.ts";
import { type Effort, Hold } from "./hold.ts";
import { Recoil, recoilImpulse, type Stance } from "./recoil.ts";
import { offLine, RELOAD, type RifleGeometry, RifleModel } from "./rifle.ts";
import type { Rounds } from "./rounds.ts";
import type { SightPicture } from "./sights.ts";

/** From the trigger breaking to the bullet leaving: the hammer's fall and the barrel, s (EST:
 * an AR's lock time about 7 ms, the bullet a millisecond in an 11.5" barrel). */
const LOCK_S = 0.008;
/** EST: the rifle and ammunition's own spread, one standard deviation each way (an M193 lot from
 * a carbine: some 2–3 MOA for five shots), rad; and the muzzle speed's, m/s. */
const DISPERSION = (0.75 * Math.PI) / (180 * 60);
const MV_SD = 9;
/** The eye's height above the ground, m, by stance; and the time to change, s. */
const EYE_M: Record<Stance, number> = { standing: 1.62, kneeling: 1.05, prone: 0.32 };
const STANCE_S: Record<Stance, number> = { standing: 0.7, kneeling: 0.6, prone: 1.3 };
/** Walking, sprinting, crawling, m/s; walking on the sight (EST: a man in kit sprints some
 * 6 m/s, and gets there in about a second; walking, in a step). */
const WALK = 1.5;
const SPRINT = 6;
const CRAWL = 0.35;
const WALK_ADS = 0.8;
const SPEED_TAU_S = 0.18;
const SPRINT_TAU_S = 0.5;
/** The rifle coming down for a sprint and back up, s. */
const SPRINT_POSE_S = 0.25;
/** A standing jump in kit, m/s up (EST: some 0.35 m); gravity, m/s². */
const JUMP_MS = 2.6;
const G = 9.81;
/** What the legs take: a rise this high is a step (a jump clears it above the feet), m; a drop
 * this deep a fall. The body's reach ahead, m, for walls. */
const STEP_M = 0.3;
const FALL_M = 0.25;
const BODY_M = 0.3;
/** The knees taking a landing: the eye's dip per m/s of landing, and its spring (EST). */
const DIP_PER_MS = 0.8;
const DIP_HZ = 2.5;
const DIP_ZETA = 0.8;
/** The ground ahead, looked up as they move, in cells this big, m, good this long, ms. */
const CELL_M = 0.5;
const CELL_LIFE_MS = 5000;
const PROBE_MS = 90;
/** Under the feet: this many looks in a row say a wall before it is taken as the ground (the
 * tiles drawn sharper, not a wall walked into). */
const WALL_LOOKS = 3;
/** The view and the rifle stay off straight up and down (past them the camera turns over). */
const PITCH_MAX = Math.PI / 2 - 0.02;
/** How fast the eye comes onto the sight and off it, s. */
const ADS_S = 0.22;
/** Head up: the eye this far above and behind where it is on the sight, m (body frame). */
const HEAD_UP: Vec3 = [-0.03, 0, -0.06];
/** On the sight, how much of the rifle's motion the eye follows (the cheek slips a little). */
const CHEEK = 0.8;
/** Pitch limits, rad: standing and kneeling, prone (the neck). */
const PITCH: Record<Stance, [number, number]> = {
  standing: [-1.3, 1.45],
  kneeling: [-1.2, 1.4],
  prone: [-0.3, 0.45],
};
/** Turning while prone means shuffling the body round, rad/s. */
const PRONE_TURN = 0.9;
const GROUND_MS = 120;

function clampPitch(p: number): number {
  return Math.max(-PITCH_MAX, Math.min(PITCH_MAX, p));
}

function gauss(): number {
  const u = 1 - Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * Math.random());
}

export interface Spot {
  lat: number;
  lon: number;
  /** Ground, sim height (m above sea level). */
  alt: number;
  headingDeg: number;
}

export class Shooter {
  lat: number;
  lon: number;
  ground: number;
  /** Where they mean to point: clockwise from north, up; rad. */
  heading: number;
  pitch = 0.12;
  stance: Stance = "standing";
  /** 0..1 through a change of stance; 1 when settled. */
  private stanceK = 1;
  private eyeH = EYE_M.standing;
  private eyeFrom = EYE_M.standing;
  ads = 0;
  /** On the sight wanted: toggled, or held on the mouse or the pad. */
  private adsWanted = false;
  private adsMouse = false;
  private adsPad = false;
  sight: "holo" | "irons";
  /** Rounds in the magazine, and one in the chamber. */
  mag: number;
  chambered = true;
  /** Seconds into a reload, null when not; and whether it began empty (the bolt locked back). */
  reloading: number | null = null;
  private reloadEmpty = false;
  readonly hold = new Hold();
  readonly recoil: Recoil;
  /** Shots fired, for the tracers' count. */
  fired = 0;
  private pending: number[] = [];
  private keys = new Set<string>();
  private dx = 0;
  private dy = 0;

  private padWas: boolean[] = [];
  private bob = 0;
  private effort: Effort = "still";
  private groundBusy = false;
  private lastGround = 0;
  /** The feet's height (sim, m above sea level) and how fast it rises; off the ground or not. */
  private feet: number;
  private vz = 0;
  private airborne = false;
  /** Over the ground, m/s: east, north. */
  private vel: [number, number] = [0, 0];
  /** The knees taking a landing: the eye down, m, and how fast. */
  private dip = 0;
  private dipV = 0;
  /** 0..1, the rifle down for a sprint. */
  private sprintK = 0;
  /** Ground heights looked up around them (sim), by cell of the local frame. */
  private cells = new Map<string, { h: number; at: number }>();
  private probeBusy = false;
  private lastProbe = 0;
  /** Where the ground under the feet was last one they could stand on; looks since that said
   * a wall; and whether it has been looked at yet. */
  private lastGood: [number, number];
  private walled = 0;
  private footed = false;
  private readonly load: Load;
  private readonly impulse: number;
  private readonly zero: Record<"holo" | "irons", number>;
  readonly rifle: RifleModel;
  private readonly abort = new AbortController();
  private readonly near: number;
  /** The rangefinder was pressed. */
  onLase: (() => void) | null = null;
  /** The last frame's camera: eye (local frame), and attitude. */
  eyeLocal: Vec3 = [0, 0, 0];
  viewQ: Quat = [1, 0, 0, 0];
  rifleQ: Quat = [1, 0, 0, 0];

  constructor(
    private readonly world: World,
    readonly weapon: Weapon,
    private readonly geometry: RifleGeometry,
    spot: Spot,
    private readonly rounds: Rounds,
    where: Setting,
    private readonly exclude: () => object[],
  ) {
    this.lat = spot.lat;
    this.lon = spot.lon;
    this.ground = spot.alt;
    this.feet = spot.alt;
    this.lastGood = [spot.lat, spot.lon];
    this.heading = (spot.headingDeg * Math.PI) / 180;
    this.mag = weapon.magazine;
    this.sight = prefs().shoot.sight;
    this.load = loadOf(weapon);
    const c = weapon.cartridge;
    this.impulse = recoilImpulse({
      bulletKg: c.bullet_g / 1000,
      powderKg: c.powder_g / 1000,
      muzzleMs: c.muzzle_ms,
    });
    this.recoil = new Recoil(weapon.mass_kg);
    const g = geometry;
    this.zero = {
      holo: zeroAngle(
        this.load,
        where,
        offLine(g.holoAxis[0], g.boreAxis[0], g.boreAxis[1]),
        weapon.zero_m,
      ),
      irons: zeroAngle(
        this.load,
        where,
        offLine(g.rearAperture, g.boreAxis[0], g.boreAxis[1]),
        weapon.zero_m,
      ),
    };
    this.rifle = new RifleModel(world, `/${weapon.model}`, geometry);
    this.rifle.setSight(this.sight);
    this.near = (world.viewer.camera.frustum as C.PerspectiveFrustum).near;
    this.listen();
  }

  /** The recoil's momentum, N·s (for the HUD's notes). */
  get recoilNs(): number {
    return this.impulse;
  }

  // Input.

  private listen(): void {
    const signal = this.abort.signal;
    const canvas = this.world.scene.canvas;
    window.addEventListener(
      "keydown",
      (e) => {
        if (paused() || (e.target as HTMLElement).tagName === "INPUT") return;
        // Not a press of whatever button last had the focus, nor a scroll.
        if (e.code === "Space") e.preventDefault();
        this.keys.add(e.code);
        if (e.repeat) return;
        if (e.code === "KeyR") this.reload();
        else if (e.code === "KeyC")
          this.toStance(this.stance === "kneeling" ? "standing" : "kneeling");
        else if (e.code === "KeyZ") this.toStance(this.stance === "prone" ? "standing" : "prone");
        else if (e.code === "KeyF") this.onLase?.();
        else if (e.code === "KeyI") this.switchSight();
        else if (e.code === "Space") this.jump();
      },
      { signal },
    );
    window.addEventListener("keyup", (e) => this.keys.delete(e.code), { signal });
    window.addEventListener("blur", () => this.keys.clear(), { signal });
    // Pointer events, not mouse events: Cesium cancels pointerdown on its canvas, and a cancelled
    // pointerdown takes the mouse events with it. A second button pressed while one is held comes
    // as a pointermove with new `buttons`, so presses and releases are read from that mask.
    let held = 0;
    const buttons = (e: PointerEvent) => {
      const was = held;
      held = e.buttons;
      if (document.pointerLockElement !== canvas || paused()) return;
      const pressed = held & ~was;
      const released = was & ~held;
      if (pressed & 1) this.pull();
      if (pressed & 2) this.aim(true);
      if (released & 2) this.aim(false);
    };
    window.addEventListener(
      "pointerdown",
      (e) => {
        if (e.target !== canvas || paused() || e.pointerType !== "mouse") return;
        // Locked, the press is the rifle's alone: Cesium's own handler would try to capture a
        // locked pointer, which Chrome refuses (an error every shot). Caught here, before it.
        if (document.pointerLockElement === canvas) e.stopPropagation();
        else {
          // The first click takes the mouse; it does not fire. Asked once Cesium has had the
          // press, so its pointer capture comes before the lock and not after it.
          held = e.buttons;
          setTimeout(() => {
            if (signal.aborted) return;
            void (canvas.requestPointerLock as (o?: object) => Promise<void>)
              .call(canvas, { unadjustedMovement: true })
              ?.catch?.(() => canvas.requestPointerLock());
          });
          return;
        }
        buttons(e);
      },
      { signal, capture: true },
    );
    window.addEventListener("pointerup", buttons, { signal });
    window.addEventListener(
      "pointermove",
      (e) => {
        if (e.buttons !== held) buttons(e);
        if (document.pointerLockElement !== canvas || paused()) return;
        this.dx += e.movementX;
        this.dy += e.movementY;
      },
      { signal },
    );
    // Losing the mouse (Esc, the pause menu) lets go of the sight.
    document.addEventListener(
      "pointerlockchange",
      () => {
        if (document.pointerLockElement === canvas) return;
        held = 0;
        this.aim(false);
      },
      { signal },
    );
    canvas.addEventListener("contextmenu", (e) => e.preventDefault(), { signal });
  }

  private aim(down: boolean): void {
    if (prefs().shoot.adsHold) this.adsMouse = down;
    else if (down) this.adsWanted = !this.adsWanted;
  }

  private get wantsAds(): boolean {
    return prefs().shoot.adsHold ? this.adsMouse || this.adsPad : this.adsWanted;
  }

  private pull(): void {
    if (this.reloading !== null || this.stanceK < 1 || this.sprintK > 0.2) return;
    if (!this.chambered) {
      gunSounds.dry();
      return;
    }
    this.hold.pull();
    this.chambered = false;
    this.pending.push(performance.now() + LOCK_S * 1000);
  }

  reload(): void {
    if (this.reloading !== null || this.mag >= this.weapon.magazine) return;
    this.reloadEmpty = !this.chambered;
    this.reloading = 0;
  }

  /** The other sight (the Rifle tab's choice: it stays for next time). */
  switchSight(): void {
    if (this.weapon.sights.length < 2) return;
    const sight = this.sight === "holo" ? "irons" : "holo";
    updatePrefs((p) => ({ ...p, shoot: { ...p.shoot, sight } }));
  }

  /** Up off the ground, standing. */
  jump(): void {
    if (this.airborne || !this.footed || this.stance !== "standing" || this.stanceK < 1) return;
    this.airborne = true;
    this.vz = JUMP_MS;
  }

  toStance(s: Stance): void {
    if (s === this.stance || this.stanceK < 1 || this.airborne) return;
    this.eyeFrom = this.eyeH;
    this.stance = s;
    this.stanceK = 0;
    this.hold.stance = s;
    this.recoil.stance = s;
  }

  private pad(dt: number): { move: [number, number]; run: boolean; breath: boolean } {
    const out = { move: [0, 0] as [number, number], run: false, breath: false };
    const gp = navigator.getGamepads().find((p): p is Gamepad => !!p && p.mapping === "standard");
    if (!gp) return out;
    const dz = (x: number) =>
      Math.abs(x) < 0.12 ? 0 : Math.sign(x) * ((Math.abs(x) - 0.12) / 0.88);
    const b = (i: number) => (gp.buttons[i]?.value ?? 0) > 0.5;
    const pressed = (i: number) => b(i) && !this.padWas[i];
    const rate = ((prefs().shoot.padDegS * Math.PI) / 180) * this.zoomK();
    const rx = dz(gp.axes[2] ?? 0);
    const ry = dz(gp.axes[3] ?? 0);
    this.heading += Math.sign(rx) * Math.abs(rx) ** 2.2 * rate * dt;
    this.pitch -= Math.sign(ry) * Math.abs(ry) ** 2.2 * rate * dt;
    out.move = [dz(gp.axes[0] ?? 0), -dz(gp.axes[1] ?? 0)];
    out.run = b(10);
    out.breath = b(5);
    this.adsPad = b(6);
    if (!prefs().shoot.adsHold && pressed(6)) this.adsWanted = !this.adsWanted;
    if (pressed(7)) this.pull();
    if (pressed(2)) this.reload();
    if (pressed(1)) {
      const next: Record<Stance, Stance> = {
        standing: "kneeling",
        kneeling: "prone",
        prone: "standing",
      };
      this.toStance(next[this.stance]);
    }
    if (pressed(3)) this.switchSight();
    if (pressed(4)) this.onLase?.();
    if (pressed(0)) this.jump();
    this.padWas = gp.buttons.map((x) => x.value > 0.5);
    return out;
  }

  /** The view's narrowing on the sight: mouse and stick turn slower with it. */
  private zoomK(): number {
    return 1 - this.ads * (1 - prefs().shoot.adsZoom);
  }

  // Each frame.

  frame(now: number, dt: number): SightPicture | null {
    const p = prefs().shoot;
    if (p.sight !== this.sight && this.weapon.sights.includes(p.sight)) {
      this.sight = p.sight;
      this.rifle.setSight(this.sight);
    }
    const headingWas = this.heading;
    const pad = paused()
      ? { move: [0, 0] as [number, number], run: false, breath: false }
      : this.pad(dt);
    // Look.
    const k = (p.mouseMrad / 1000) * this.zoomK();
    this.heading += this.dx * k;
    this.pitch -= this.dy * k;
    this.dx = this.dy = 0;
    if (this.stance === "prone") {
      const turn = PRONE_TURN * dt;
      this.heading = headingWas + Math.max(-turn, Math.min(turn, this.heading - headingWas));
    }
    const [lo, hi] = PITCH[this.stance];
    this.pitch = Math.max(lo, Math.min(hi, this.pitch));
    // Move.
    const key = (c: string) => this.keys.has(c);
    let fwd = (key("KeyW") ? 1 : 0) - (key("KeyS") ? 1 : 0) + pad.move[1];
    let side = (key("KeyD") ? 1 : 0) - (key("KeyA") ? 1 : 0) + pad.move[0];
    const len = Math.hypot(fwd, side);
    if (len > 1) {
      fwd /= len;
      side /= len;
    }
    const moving = len > 0.05 && this.stanceK >= 1 && this.stance !== "kneeling";
    const shift = key("ShiftLeft") || key("ShiftRight");
    const sprinting =
      moving &&
      (shift || pad.run) &&
      this.stance === "standing" &&
      this.ads < 0.1 &&
      this.reloading === null &&
      fwd > 0.5;
    if (sprinting) this.adsWanted = this.adsMouse = false;
    this.sprintK = Math.max(0, Math.min(1, this.sprintK + (sprinting ? dt : -dt) / SPRINT_POSE_S));
    this.move(dt, moving ? fwd : 0, moving ? side : 0, sprinting);
    void this.probe(now);
    const speedNow = Math.hypot(this.vel[0], this.vel[1]);
    if (!this.airborne && speedNow > 0.1) {
      this.bob += dt * (speedNow > WALK + 0.5 ? 2.8 : 1.9) * Math.PI * 2;
    }
    this.effort = speedNow > WALK + 0.5 ? "run" : speedNow > 0.1 ? "walk" : "still";
    // Stance and the sight.
    if (this.stanceK < 1) {
      this.stanceK = Math.min(1, this.stanceK + dt / STANCE_S[this.stance]);
      const s = this.stanceK * this.stanceK * (3 - 2 * this.stanceK);
      this.eyeH = this.eyeFrom + (EYE_M[this.stance] - this.eyeFrom) * s;
    }
    const want =
      this.wantsAds && this.reloading === null && this.stanceK >= 1 && this.sprintK === 0;
    this.ads = Math.max(0, Math.min(1, this.ads + (want ? dt : -dt) / ADS_S));
    // The hold and the recoil.
    const breath = this.ads > 0.5 && (shift || pad.breath) && !moving && !this.airborne;
    this.hold.step(dt, this.effort, breath);
    this.recoil.step(dt);
    this.reloadStep(dt);
    void this.followGround(now);
    return this.place(now);
  }

  private reloadStep(dt: number): void {
    if (this.reloading === null) return;
    const before = this.reloading;
    const t = (this.reloading += dt);
    const passed = (at: number) => before < at && t >= at;
    if (passed(RELOAD.drop[0])) gunSounds.magOut();
    if (passed(RELOAD.insert[1])) gunSounds.magIn();
    const bolt = RELOAD.back + 0.25;
    if (this.reloadEmpty && passed(bolt)) gunSounds.bolt();
    this.rifle.reloadPose(t);
    const end = this.reloadEmpty ? Math.max(this.weapon.reload_s, bolt + 0.1) : RELOAD.back + 0.15;
    if (t >= end) {
      this.reloading = null;
      this.rifle.reloadPose(null);
      if (this.reloadEmpty) {
        this.mag = this.weapon.magazine - 1;
        this.chambered = true;
      } else this.mag = this.weapon.magazine;
    }
  }

  /** On over the ground by `dt`: speeding up or slowing to what the keys or stick ask (no say
   * in it off the ground), stopped by walls, and up and down with the ground and a jump. */
  private move(dt: number, fwd: number, side: number, sprinting: boolean): void {
    const speed =
      this.stance === "prone" ? CRAWL : sprinting ? SPRINT : this.ads > 0.5 ? WALK_ADS : WALK;
    if (!this.airborne) {
      const h = this.heading;
      const want = [
        (Math.sin(h) * fwd + Math.cos(h) * side) * speed,
        (Math.cos(h) * fwd - Math.sin(h) * side) * speed,
      ];
      const k = 1 - Math.exp(-dt / (sprinting ? SPRINT_TAU_S : SPEED_TAU_S));
      this.vel[0] += (want[0]! - this.vel[0]) * k;
      this.vel[1] += (want[1]! - this.vel[1]) * k;
    }
    // Along a wall rather than into it: the whole way, or east or north alone.
    const de = this.vel[0] * dt;
    const dn = this.vel[1] * dt;
    if (de !== 0 || dn !== 0) {
      if (!this.blocked(de, dn)) this.step(de, dn);
      else if (de !== 0 && !this.blocked(de, 0)) {
        this.step(de, 0);
        this.vel[1] = 0;
      } else if (dn !== 0 && !this.blocked(0, dn)) {
        this.step(0, dn);
        this.vel[0] = 0;
      } else this.vel = [0, 0];
    }
    // Up and down.
    if (this.airborne) {
      this.vz -= G * dt;
      this.feet += this.vz * dt;
      if (this.feet <= this.ground) this.land();
    } else if (this.ground < this.feet - FALL_M) {
      this.airborne = true;
      this.vz = 0;
    } else if (this.ground > this.feet + STEP_M) this.feet = this.ground;
    else this.feet += (this.ground - this.feet) * (1 - Math.exp(-dt / 0.08));
    // The knees.
    const w = 2 * Math.PI * DIP_HZ;
    const n = Math.max(1, Math.ceil(dt / 0.004));
    for (let i = 0; i < n; i++) {
      this.dipV += (-w * w * this.dip - 2 * DIP_ZETA * w * this.dipV) * (dt / n);
      this.dip += this.dipV * (dt / n);
    }
  }

  private land(): void {
    const impact = Math.max(0, -this.vz);
    this.feet = this.ground;
    this.vz = 0;
    this.airborne = false;
    this.dipV += impact * DIP_PER_MS;
  }

  private step(de: number, dn: number): void {
    [this.lat, this.lon] = moveBy(this.lat, this.lon, dn, de);
  }

  /** Here, the local frame: east, north. */
  private here(): [number, number] {
    const [n, e] = neFrom(this.rounds.origin.lat, this.rounds.origin.lon, this.lat, this.lon);
    return [e, n];
  }

  private cellKey(e: number, n: number): string {
    return `${Math.floor(e / CELL_M)},${Math.floor(n / CELL_M)}`;
  }

  /** Whether a step of (`de`, `dn`) m walks the body into something higher than the legs (or,
   * off the ground, the feet) clear, as the ground looked up ahead has it. */
  private blocked(de: number, dn: number): boolean {
    const len = Math.hypot(de, dn);
    if (len === 0) return false;
    const [e, n] = this.here();
    const at = this.cells.get(
      this.cellKey(e + de + (de / len) * BODY_M, n + dn + (dn / len) * BODY_M),
    );
    if (!at || performance.now() - at.at > CELL_LIFE_MS) return false;
    return at.h > this.feet + STEP_M;
  }

  /** The ground ahead, looked up while they move: just past the body, and as far as the next
   * look's time will carry them. */
  private async probe(now: number): Promise<void> {
    const speed = Math.hypot(this.vel[0], this.vel[1]);
    if (this.probeBusy || now - this.lastProbe < PROBE_MS || speed < 0.1) return;
    this.probeBusy = true;
    this.lastProbe = now;
    try {
      const [e, n] = this.here();
      const u = [this.vel[0] / speed, this.vel[1] / speed];
      const at = [BODY_M + CELL_M / 2, BODY_M + 0.3 + speed * 0.25].map(
        (d) => [e + u[0]! * d, n + u[1]! * d] as [number, number],
      );
      const o = this.rounds.origin;
      const hs = await this.world.groundHeights(
        at.map(([pe, pn]) => moveBy(o.lat, o.lon, pn, pe)),
        this.exclude(),
      );
      const t = performance.now();
      hs.forEach((h, i) => {
        if (h !== undefined) {
          const [pe, pn] = at[i]!;
          this.cells.set(this.cellKey(pe, pn), { h: h - mapOffset(), at: t });
        }
      });
      if (this.cells.size > 400) {
        for (const [k, v] of this.cells) if (t - v.at > CELL_LIFE_MS) this.cells.delete(k);
      }
    } finally {
      this.probeBusy = false;
    }
  }

  /** The drawn ground under their feet, now and then. Higher than a step over the feet, a wall
   * the looks ahead missed: back to where they last stood (unless it stays, a few looks on). */
  private async followGround(now: number): Promise<void> {
    if (this.groundBusy || now - this.lastGround < GROUND_MS) return;
    this.groundBusy = true;
    this.lastGround = now;
    try {
      const at: [number, number] = [this.lat, this.lon];
      const h = await this.world.groundHeight(at[0], at[1], this.exclude());
      if (h === undefined) return;
      const sim = h - mapOffset();
      const [e, n] = this.here();
      this.cells.set(this.cellKey(e, n), { h: sim, at: performance.now() });
      if (!this.footed) {
        this.footed = true;
        this.ground = this.feet = sim;
        this.lastGood = at;
        return;
      }
      if (sim > this.feet + STEP_M && ++this.walled < WALL_LOOKS) {
        [this.lat, this.lon] = this.lastGood;
        this.vel = [0, 0];
        return;
      }
      this.walled = 0;
      this.lastGood = at;
      this.ground = sim;
    } finally {
      this.groundBusy = false;
    }
  }

  /** The camera, the rifle, and any shot due: the sight picture. */
  private place(now: number): SightPicture {
    const g = this.geometry;
    const h = this.hold;
    const r = this.recoil;
    const walkBob = this.effort === "still" ? 0 : this.effort === "run" ? 1 : 0.4;
    const bobUp = Math.abs(Math.sin(this.bob)) * 0.03 * walkBob;
    const bobSide = Math.sin(this.bob / 2) * 0.012 * walkBob;
    const swayYaw = Math.sin(this.bob / 2) * 0.01 * walkBob;
    const swayPitch = Math.sin(this.bob) * 0.008 * walkBob;
    // The rifle's aim.
    const cosP = Math.max(0.2, Math.cos(this.pitch));
    const rifleHeading = this.heading + (h.yaw + r.yawRight + swayYaw) / cosP;
    const riflePitch = clampPitch(this.pitch + h.pitch + r.pitch + swayPitch);
    // The reload: the rifle tips in towards the shooter and down to see the well.
    const rl =
      this.reloading === null
        ? 0
        : Math.sin(Math.min(1, this.reloading / this.weapon.reload_s) * Math.PI);
    // A sprint: the rifle down and across the chest.
    const sk = this.sprintK * this.sprintK * (3 - 2 * this.sprintK);
    const rifleQ = quatOf(
      rifleHeading - 0.15 * rl - 0.2 * sk,
      riflePitch - 0.25 * rl - 0.1 * sk,
      -0.35 * rl - 0.7 * sk,
    );
    // The eye's aim: on the sight it rides the stock; head up, it looks where they mean.
    const a = this.ads;
    const eyeHeading = this.heading + (a * CHEEK + (1 - a) * 0.15) * (rifleHeading - this.heading);
    // Reloading, the eye goes down to the magazine well with the rifle.
    const eyePitch = clampPitch(
      this.pitch + (a * CHEEK + (1 - a) * 0.15) * (riflePitch - this.pitch) - 0.3 * rl,
    );
    const viewQ = quatOf(eyeHeading, eyePitch, 0);
    // Where the eye is: over the ground, by stance, with the walk's bob and the recoil's rock.
    const fwdH: Vec3 = [Math.sin(this.heading), Math.cos(this.heading), 0];
    const rightH: Vec3 = [Math.cos(this.heading), -Math.sin(this.heading), 0];
    const [n0, e0] = neFrom(this.rounds.origin.lat, this.rounds.origin.lon, this.lat, this.lon);
    const back = r.headBack;
    const eye: Vec3 = [
      e0 + rightH[0] * bobSide - fwdH[0] * back,
      n0 + rightH[1] * bobSide - fwdH[1] * back,
      this.feet - this.rounds.origin.alt + this.eyeH + bobUp - this.dip,
    ];
    // The rifle: its eye point (holo or irons) at the eye on the sight, lower and ahead head up,
    // and back into the shoulder by the recoil.
    const sightEye = this.sight === "holo" ? g.eye : g.ironEye;
    const eyeOnRifle: Vec3 = [
      sightEye[0] + (1 - a) * HEAD_UP[0] - (r.rifleBackM - r.headBack),
      sightEye[1] + (1 - a) * HEAD_UP[1],
      sightEye[2] + (1 - a) * HEAD_UP[2],
    ];
    const local = (p: Vec3): Vec3 => {
      const ned = rotate(rifleQ, [
        p[0] - eyeOnRifle[0],
        p[1] - eyeOnRifle[1],
        p[2] - eyeOnRifle[2],
      ]);
      return [eye[0] + ned[1], eye[1] + ned[0], eye[2] - ned[2]];
    };
    const butt = local([0, 0, 0]);
    this.rifle.place(this.rounds.fixed(butt), rifleQ);
    this.eyeLocal = eye;
    this.viewQ = viewQ;
    this.rifleQ = rifleQ;
    // The camera.
    const camera = this.world.viewer.camera;
    const eyeFixed = this.rounds.fixed(eye);
    camera.setView({
      destination: eyeFixed,
      orientation: { heading: eyeHeading, pitch: eyePitch, roll: 0 },
    });
    const frustum = camera.frustum as C.PerspectiveFrustum;
    frustum.fov = C.Math.toRadians(prefs().view.fovDeg * this.zoomK());
    frustum.near = 0.02;
    // The ear, for the sounds' panning.
    this.rounds.ear = eye;
    this.rounds.earRight = rightH;
    // A shot due leaves now, along the bore as the rifle points this instant.
    const axes = bodyAxesEnu(rifleQ);
    while (this.pending.length && this.pending[0]! <= now) {
      this.pending.shift();
      this.leave(now, local(g.muzzle), axes);
    }
    // The sight picture.
    const toFixed = (p: Vec3) => this.rounds.fixed(local(p));
    const axisFixed = C.Cartesian3.normalize(
      C.Cartesian3.subtract(
        toFixed([1000, 0, sightEye[2]]),
        toFixed([0, 0, sightEye[2]]),
        new C.Cartesian3(),
      ),
      new C.Cartesian3(),
    );
    return {
      sight: this.sight,
      ads: this.ads,
      eye: eyeFixed,
      axis: axisFixed,
      window: g.holoWindow.map(toFixed),
      aperture: toFixed(g.rearAperture),
      bright: prefs().shoot.holoBright,
    };
  }

  /** The bullet leaves the muzzle: along the sight's line tipped up by the zero, spread a
   * little, and the rifle kicks. */
  private leave(now: number, muzzle: Vec3, axes: { fwd: Vec3; right: Vec3; up: Vec3 }): void {
    const z = this.zero[this.sight] + gauss() * DISPERSION;
    const s = gauss() * DISPERSION;
    const dir: Vec3 = [0, 1, 2].map(
      (i) => axes.fwd[i]! + axes.up[i]! * Math.tan(z) + axes.right[i]! * Math.tan(s),
    ) as Vec3;
    const speed = this.weapon.cartridge.muzzle_ms + gauss() * MV_SD;
    const every = prefs().shoot.tracers;
    const loaded = this.weapon.cartridge.tracer_every;
    const tracer =
      every === "every" || (every === "loaded" && loaded > 0 && this.fired % loaded === 0);
    this.rounds.fire(this.load, muzzle, dir, speed, tracer, now);
    this.fired++;
    this.recoil.kick(this.impulse);
    gunSounds.shot();
    // The next round: from the magazine, or the bolt locks back on an empty one.
    if (this.mag > 0) {
      this.mag--;
      this.chambered = true;
    }
  }

  /** Where the eye looks (the camera's forward), local frame. */
  get lookDir(): Vec3 {
    return bodyAxesEnu(this.viewQ).fwd;
  }

  dispose(): void {
    this.abort.abort();
    this.rifle.dispose();
    if (document.pointerLockElement) document.exitPointerLock();
    const frustum = this.world.viewer.camera.frustum as C.PerspectiveFrustum;
    frustum.near = this.near;
  }
}
