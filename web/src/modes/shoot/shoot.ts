// Shoot Down (DESIGN.md §7): the player is a person on the ground with a rifle, and a real drone
// (ArduCopter or Betaflight on SkyFlow-Dynamics, the server's session) hovers out in front of
// them. Bring it down. Every bullet flies (shoot/ballistics.ts) through the place's air; what it
// hits on the drone goes to the physics as damage by part, and the drone falls as it would.
//
// The shooter starts 100 m behind the pad, facing it; the drone takes off from the pad and holds
// 30 m above it (the server flies it, as a ground station would, or Betaflight's pilot on its
// switches: target hover). Once it is down
// (crashed, or on the ground after a hit) it comes back after a few seconds: reset and hover.

import { effect } from "alien-signals";
import * as C from "cesium";
import { droneGain, rotorNorm } from "../../audio/levels.ts";
import { sounds } from "../../audio/sounds.ts";
import type { Flight, ModeRuntime } from "../../flight.ts";
import { radio } from "../../input/radio.ts";
import type { Weapon } from "../../net/protocol.ts";
import { link, type Pose } from "../../net/server.ts";
import { Air } from "../../shoot/air.ts";
import { airAt, setting } from "../../shoot/ballistics.ts";
import { setSpeedOfSound } from "../../shoot/gunsounds.ts";
import { rifleGeometry } from "../../shoot/rifle.ts";
import { type Origin, Rounds } from "../../shoot/rounds.ts";
import { Shooter } from "../../shoot/shooter.ts";
import { Sights } from "../../shoot/sights.ts";
import { content, toast, wind } from "../../state/app.ts";
import { hudBanner, hudPanel } from "../../state/hud.ts";
import { prefs } from "../../state/prefs.ts";
import { phase, status } from "../../state/session.ts";
import { h } from "../../ui/h.ts";
import type { RigContext } from "../../views/rigs.ts";
import { mapOffset, moveBy, neFrom, type Vec3 } from "../../world/geo.ts";
import type { ModeDef } from "../types.ts";

/** Where the shooter starts: this far behind the pad, m; and the drone's hover, m above it. */
const STAND_OFF_M = 100;
const HOVER_M = 30;
/** Down, it comes back after this long, s. */
const RESPAWN_S = 5;
/** A rangefinder's beam spreads (EST: a hunting rangefinder's, 2 mrad); its reading shows for
 * this long, s. */
const BEAM_RAD = 0.002;
const LASE_SHOW_S = 8;

const COMPASS = [
  "N",
  "NNE",
  "NE",
  "ENE",
  "E",
  "ESE",
  "SE",
  "SSE",
  "S",
  "SSW",
  "SW",
  "WSW",
  "W",
  "WNW",
  "NW",
  "NNW",
];

export const shoot: ModeDef = {
  id: "shoot",
  name: "Shoot Down",
  desc: "On the ground with a rifle. A drone hovers out in front: bring it down.",
  // The target: a multirotor its firmware can hover on its own (ArduCopter in GUIDED, or
  // Betaflight in POSHOLD, the server on its switches).
  vehicles: (all) =>
    all.filter(
      (v) =>
        v.kind === "multirotor" &&
        (v.firmware.kind === "ardupilot-copter" || v.firmware.kind === "betaflight"),
    ),
  pickHint: "The drone to shoot at",
  ownsView: true,
  run: (flight) => new ShootRuntime(flight),
};

type DroneState = "starting" | "climbing" | "hovering" | "hit" | "down";

class ShootRuntime implements ModeRuntime {
  readonly ownsView = true;
  private shooter: Shooter | null = null;
  private rounds: Rounds | null = null;
  private sights: Sights;
  private air: Air | null = null;
  private weapon: Weapon | null;
  private disposers: (() => void)[] = [];
  private disposed = false;
  private lastNow = 0;
  private drone: DroneState = "starting";
  private hoverSent = false;
  private firstShotAt: number | null = null;
  private shotsAtThis = 0;
  private stats = {
    shots: 0,
    hits: 0,
    downs: 0,
    best: null as number | null,
    last: null as number | null,
  };
  private damaged = new Set<string>();
  private lase: { text: string; until: number } | null = null;
  private panel = h("div", { class: "shoot-hud panel" });
  private lastPanel = 0;

  constructor(private readonly flight: Flight) {
    this.weapon = Object.values(content()?.weapons ?? {})[0] ?? null;
    radio.onFoot = true;
    this.sights = new Sights(flight.world, flight.world.viewer.container as HTMLElement);
    if (!this.weapon) toast("No weapon in content/weapons", "bad", 8000);
    hudPanel(this.panel);
    hudBanner({ big: "", small: "The drone is starting…" });
    this.disposers.push(
      effect(() => {
        const s = status();
        const p = phase();
        this.onSession(s.state, p);
      }),
    );
    void this.setup();
  }

  /** The shooter and their rifle, once the pad's ground is known (flight.pad). */
  private async setup(): Promise<void> {
    const flight = this.flight;
    const weapon = this.weapon;
    if (!weapon) return;
    const pad = await new Promise<RigContext["pad"]>((resolve) => {
      const stop = effect(() => {
        const p = flight.pad();
        if (p) {
          queueMicrotask(() => stop());
          resolve(p);
        }
      });
    });
    if (this.disposed) return;
    const hd = (pad.heading_deg * Math.PI) / 180;
    const [lat, lon] = moveBy(
      pad.lat,
      pad.lon,
      -Math.cos(hd) * STAND_OFF_M,
      -Math.sin(hd) * STAND_OFF_M,
    );
    const map = await flight.world.groundHeightSettled(lat, lon);
    const ground = map - mapOffset();
    if (this.disposed) return;
    const origin: Origin = { lat, lon, alt: ground };
    const w = wind() ?? content()?.locations[status().location ?? ""]?.wind;
    if (!w) return;
    this.air = new Air(w);
    const air = this.air;
    const where = setting(lat, ground, (p) => air.at(Math.max(0.1, p[2])));
    setSpeedOfSound(airAt(ground).sound);
    const exclude = () => this.primitives();
    this.rounds = new Rounds(
      flight.world,
      origin,
      where,
      () => {
        const body = flight.body();
        return body ? (now: number) => body.poseAt(now) : null;
      },
      exclude,
    );
    this.rounds.onHit = (hit) => {
      this.stats.hits++;
      const p = hit.strike.part;
      if (p !== "camera" && p !== "frame") this.damaged.add(`${p}${hit.strike.motor ?? ""}`);
      if (this.drone === "hovering" || this.drone === "climbing") this.drone = "hit";
      const what = {
        prop: "prop",
        motor: "motor",
        arm: "arm",
        battery: "battery",
        stack: "flight controller",
        camera: "camera",
        frame: "frame",
      }[p];
      const motor = hit.strike.motor ? ` ${hit.strike.motor}` : "";
      toast(
        `Hit: ${what}${motor} at ${hit.range.toFixed(0)} m, ${hit.speed.toFixed(0)} m/s`,
        "good",
        2500,
      );
    };
    const geometry = await rifleGeometry(`/${weapon.model}`);
    if (this.disposed) return;
    this.shooter = new Shooter(
      flight.world,
      weapon,
      geometry,
      { lat, lon, alt: ground, headingDeg: pad.heading_deg },
      this.rounds,
      where,
      exclude,
    );
    // Aim at where the drone will hover.
    this.shooter.pitch = Math.atan2(HOVER_M - 1.6, STAND_OFF_M);
    this.shooter.onLase = () => this.laseNow();
  }

  private onSession(state: string, p: string): void {
    if (state !== "ready") {
      this.drone = "starting";
      this.hoverSent = false;
      return;
    }
    // Ready on the ground: up it goes.
    if (!this.hoverSent && (p === "ground" || p === "")) {
      this.hoverSent = true;
      this.drone = "climbing";
      this.damaged.clear();
      this.firstShotAt = null;
      this.shotsAtThis = 0;
      link.send({ ch: "target", action: "hover", height_m: HOVER_M });
      hudBanner({ big: "", small: "The drone is taking off" });
      setTimeout(() => this.drone === "climbing" && hudBanner(null), 4000);
    }
    const down =
      p === "crashed" || (p === "ground" && this.damaged.size > 0 && this.drone !== "climbing");
    if (down && this.drone !== "down") this.downed();
  }

  private downed(): void {
    const now = performance.now();
    this.drone = "down";
    this.stats.downs++;
    const t = this.firstShotAt === null ? null : (now - this.firstShotAt) / 1000;
    this.stats.last = t;
    if (t !== null && (this.stats.best === null || t < this.stats.best)) this.stats.best = t;
    hudBanner({
      big: "DOWN",
      small: `${this.shotsAtThis} shot${this.shotsAtThis === 1 ? "" : "s"}${t !== null ? ` · ${t.toFixed(1)} s` : ""}`,
      tone: "leaf",
    });
    setTimeout(() => {
      if (this.disposed || this.drone !== "down") return;
      hudBanner({ big: "", small: "Another drone" });
      this.hoverSent = false;
      this.flight.reset();
    }, RESPAWN_S * 1000);
  }

  tick(now: number, pose: Pose | null): void {
    const dt = this.lastNow ? Math.min(0.1, (now - this.lastNow) / 1000) : 0;
    this.lastNow = now;
    const s = this.shooter;
    if (!s || !this.rounds || !this.air) return;
    const w = wind();
    if (w) this.air.wind = w;
    this.air.step(dt);
    const before = s.fired;
    const picture = s.frame(now, dt);
    if (s.fired > before) {
      this.stats.shots += s.fired - before;
      this.shotsAtThis += s.fired - before;
      if (this.firstShotAt === null && this.drone !== "down") this.firstShotAt = now;
    }
    this.rounds.update(now);
    this.sights.draw(picture);
    this.droneSound(now, pose);
    if (this.drone === "climbing" && pose && pose.alt - this.rounds.origin.alt > HOVER_M * 0.6) {
      this.drone = "hovering";
    }
    if (now - this.lastPanel > 100) {
      this.lastPanel = now;
      this.drawPanel(now);
    }
  }

  /** The drone heard from the shooter: as it was when the sound left it. */
  private droneSound(now: number, pose: Pose | null): void {
    const s = this.shooter;
    const body = this.flight.body();
    if (!s || !pose || !body || !this.rounds) return;
    const local = (p: Pose): Vec3 => {
      const [n, e] = neFrom(this.rounds!.origin.lat, this.rounds!.origin.lon, p.lat, p.lon);
      return [e, n, p.alt - this.rounds!.origin.alt];
    };
    const ear = s.eyeLocal;
    const d0 = local(pose);
    const range0 = Math.hypot(d0[0] - ear[0], d0[1] - ear[1], d0[2] - ear[2]);
    const heardPose = body.poseAt(now - (range0 / 340) * 1000) ?? pose;
    const d = local(heardPose);
    const rel: Vec3 = [d[0] - ear[0], d[1] - ear[1], d[2] - ear[2]];
    const range = Math.hypot(...rel);
    const right: Vec3 = [Math.cos(s.heading), -Math.sin(s.heading), 0];
    const pan = (rel[0] * right[0] + rel[1] * right[1]) / (range || 1);
    const each = heardPose.act.map((w) => rotorNorm("multirotor", w));
    const mean = each.reduce((a, b) => a + b, 0) / 4;
    sounds.heard(
      phase() === "crashed" || droneGain(range) <= 0
        ? []
        : [{ id: "target", kind: "multirotor", level: mean, rpmNorm: mean, each, range, pan }],
    );
  }

  private laseNow(): void {
    const s = this.shooter;
    const body = this.flight.body();
    if (!s || !this.rounds) return;
    const look = s.lookDir;
    const eye = s.eyeLocal;
    const pose = body?.poseAt(performance.now());
    let range: number | null = null;
    if (pose) {
      const [n, e] = neFrom(this.rounds.origin.lat, this.rounds.origin.lon, pose.lat, pose.lon);
      const d: Vec3 = [e - eye[0], n - eye[1], pose.alt - this.rounds.origin.alt - eye[2]];
      const along = d[0] * look[0] + d[1] * look[1] + d[2] * look[2];
      const miss = Math.hypot(
        d[0] - look[0] * along,
        d[1] - look[1] * along,
        d[2] - look[2] * along,
      );
      if (along > 0 && miss < 0.15 + (BEAM_RAD * along) / 2) range = along;
    }
    if (range === null) {
      const scene = this.flight.world.scene;
      const c = new C.Cartesian2(scene.canvas.clientWidth / 2, scene.canvas.clientHeight / 2);
      const hit = scene.pickPositionSupported ? scene.pickPosition(c) : undefined;
      if (hit) range = C.Cartesian3.distance(hit, this.rounds.fixed(eye));
    }
    const angle = (Math.asin(Math.max(-1, Math.min(1, look[2]))) * 180) / Math.PI;
    this.lase = {
      text:
        range === null
          ? "LRF  ---"
          : `LRF  ${range.toFixed(0)} m  ${angle >= 0 ? "↑" : "↓"}${Math.abs(angle).toFixed(0)}°`,
      until: performance.now() + LASE_SHOW_S * 1000,
    };
  }

  private drawPanel(now: number): void {
    const s = this.shooter;
    const w = this.air;
    if (!s || !w) return;
    const row = (k: string, v: string, cls = "") =>
      h("div", { class: `row ${cls}` }, h("span", null, k), h("b", { class: "num" }, v));
    const ammo =
      s.reloading !== null
        ? "reloading"
        : `${s.mag}${s.chambered ? "+1" : ""} / ${s.weapon.magazine}`;
    const stance = { standing: "standing", kneeling: "kneeling", prone: "prone" }[s.stance];
    const breath =
      s.hold.held > 0
        ? `holding ${s.hold.held.toFixed(0)} s`
        : s.hold.canHold
          ? ""
          : "out of breath";
    const rows = [
      row("Rifle", `${s.weapon.name.split(",")[0]} · ${s.sight === "holo" ? "holo" : "irons"}`),
      row("Ammo", ammo, s.mag === 0 && !s.chambered ? "warn" : ""),
      row("Stance", stance),
      row("Heart", `${s.hold.heart.toFixed(0)} bpm${breath ? ` · ${breath}` : ""}`),
    ];
    if (prefs().shoot.windMeter) {
      const m = w.meter();
      const rel = (((m.fromDeg - (s.heading * 180) / Math.PI) % 360) + 360) % 360;
      const clock = Math.round(rel / 30) % 12 || 12;
      rows.push(
        row(
          "Wind",
          `${m.speed.toFixed(1)} m/s from ${COMPASS[Math.round(m.fromDeg / 22.5) % 16]} · ${clock} o'clock`,
        ),
      );
    }
    if (this.lase && now < this.lase.until) rows.push(row("", this.lase.text, "lrf"));
    const droneText = {
      starting: "starting",
      climbing: "taking off",
      hovering: `hovering ${HOVER_M} m`,
      hit: this.damaged.size ? `hit: ${[...this.damaged].join(", ")}` : "hit",
      down: "down",
    }[this.drone];
    rows.push(row("Drone", droneText, this.drone === "down" ? "good" : ""));
    const st = this.stats;
    rows.push(
      row(
        "Score",
        `${st.downs} down · ${st.hits} hits / ${st.shots} shots${st.best !== null ? ` · best ${st.best.toFixed(1)} s` : ""}`,
      ),
    );
    if (document.pointerLockElement !== this.flight.world.scene.canvas)
      rows.push(h("div", { class: "row hint" }, "Click to take the mouse"));
    this.panel.replaceChildren(...rows);
  }

  /** Not ground: the drone, the rifle, the rounds' tracers and dust. */
  primitives(): object[] {
    return [
      ...(this.flight.body()?.primitives ?? []),
      ...(this.shooter?.rifle.primitives ?? []),
      ...(this.rounds?.primitives ?? []),
    ];
  }

  /** A rifleman has no map: the minimap is hidden in this mode. */
  marks() {
    return [];
  }

  dispose(): void {
    this.disposed = true;
    radio.onFoot = false;
    for (const d of this.disposers) d();
    this.shooter?.dispose();
    this.rounds?.dispose();
    this.sights.dispose();
    sounds.heard([]);
    hudPanel(null);
    hudBanner(null);
  }
}
