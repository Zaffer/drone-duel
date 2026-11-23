// Shoot Down (DESIGN.md §7): the player is a person on the ground with a rifle, and a real drone
// (ArduCopter or Betaflight on SkyFlow-Dynamics, the server's session) hovers out in front of
// them. Bring it down. Every bullet flies (shoot/ballistics.ts) through the place's air; what it
// hits on the drone goes to the physics as damage by part, and the drone falls as it would.
//
// The shooter starts 100 m behind the pad, facing it; the drone takes off from the pad and holds
// 30 m above it (the server flies it, as a ground station would, or Betaflight's pilot on its
// switches: target hover). Then, level by level (shoot/levels.ts), it flies: a line across the
// view, a circle, a figure eight (ArduCopter, the page sending it targets: target go; Betaflight,
// a mission of the path's waypoints, sent once: target path). It carries a charge (content's
// payloads: the mode fits it, whichever quad), which goes off when it is hit or one time in two
// when the drone comes down (the physics decides); the shooter is killed within its lethal
// radius. Once the drone is down (crashed, on the ground after a hit, or blown up) it comes back
// after a few seconds, at the next level: reset and hover. Killed, the shooter tries the same
// level again.

import { effect } from "alien-signals";
import * as C from "cesium";
import { droneGain, rotorNorm } from "../../audio/levels.ts";
import { sounds } from "../../audio/sounds.ts";
import type { Ground } from "../../engine/ground.ts";
import { loop } from "../../engine/loop.ts";
import type { Flight, ModeRuntime } from "../../flight.ts";
import { radio } from "../../input/radio.ts";
import type { PayloadSpec, Weapon } from "../../net/protocol.ts";
import { link, type Pose } from "../../net/server.ts";
import { Air } from "../../shoot/air.ts";
import { airAt, setting } from "../../shoot/ballistics.ts";
import { Blasts } from "../../shoot/blast.ts";
import { setSpeedOfSound } from "../../shoot/gunsounds.ts";
import { type Flies, type Level, levelsFor, pathAt, pathWaypoints } from "../../shoot/levels.ts";
import { rifleGeometry } from "../../shoot/rifle.ts";
import { Rounds } from "../../shoot/rounds.ts";
import { Shooter } from "../../shoot/shooter.ts";
import { Sights } from "../../shoot/sights.ts";
import { payloadZone } from "../../shoot/target.ts";
import { content, vehicle as currentVehicle, locations, toast, wind } from "../../state/app.ts";
import { hudBanner, hudPanel } from "../../state/hud.ts";
import { prefs } from "../../state/prefs.ts";
import { damage, phase, status } from "../../state/session.ts";
import { h } from "../../ui/h.ts";
import type { RigContext } from "../../views/rigs.ts";
import { mapOffset, moveBy, type Vec3 } from "../../world/geo.ts";
import type { ModeDef } from "../types.ts";

/** Where the shooter starts: this far behind the pad, m; and the drone's hover, m above it. */
const STAND_OFF_M = 100;
const HOVER_M = 30;
/** Down, it comes back after this long, s. */
const RESPAWN_S = 5;
/** The shooter's body is stepped this often, Hz (engine/loop.ts). */
const BODY_HZ = 240;
/** A rangefinder's beam spreads (EST: a hunting rangefinder's, 2 mrad); its reading shows for
 * this long, s. */
const BEAM_RAD = 0.002;
const LASE_SHOW_S = 8;
/** At a level that flies, it holds its hover this long first, s; then a target goes to it this
 * often, Hz. */
const HOLD_FIRST_S = 3;
const GO_HZ = 5;
/** The shooter's chest, this far below their eye, m: where a blast's distance is measured. */
const CHEST_BELOW_EYE_M = 0.3;
/** The payload Shoot Down fits to its target (content/payloads). */
const PAYLOAD = "charge";

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
  payload: PAYLOAD,
  run: (flight) => new ShootRuntime(flight),
};

type DroneState = "starting" | "climbing" | "hovering" | "flying" | "hit" | "down";

/** A target for the session's quad: the generated type has every field. */
function target(t: {
  action: "hover" | "go" | "path";
  height_m?: number;
  lat?: number;
  lon?: number;
  vel_ned?: [number, number, number];
  yaw_deg?: number;
  points?: [number, number][];
}) {
  return {
    ch: "target" as const,
    action: t.action,
    height_m: t.height_m ?? 0,
    lat: t.lat ?? null,
    lon: t.lon ?? null,
    vel_ned: t.vel_ned ?? null,
    yaw_deg: t.yaw_deg ?? null,
    points: t.points ?? null,
  };
}

/** How this firmware flies a path (shoot/levels.ts). */
function fliesFor(kind: string | undefined): Flies {
  if (kind === "ardupilot-copter") return "targets";
  if (kind === "betaflight") return "mission";
  return "hover";
}

class ShootRuntime implements ModeRuntime {
  readonly ownsView = true;
  private shooter: Shooter | null = null;
  private rounds: Rounds | null = null;
  private sights: Sights;
  private air: Air | null = null;
  private weapon: Weapon | null;
  private disposers: (() => void)[] = [];
  private disposed = false;
  private readonly abort = new AbortController();
  private ground: Ground | null = null;
  private drone: DroneState = "starting";
  private hoverSent = false;
  /** How it flies a path, the levels it can fly, and which it is at. */
  private flies: Flies;
  private levels: Level[];
  private level = 0;
  private payload: PayloadSpec | null;
  /** When it reached its hover, and when it was last sent a target (performance.now()). */
  private hoverAt: number | null = null;
  private lastGo = 0;
  /** This level's mission went (Betaflight). */
  private pathSent = false;
  private blasts: Blasts | null = null;
  /** Its charge has gone off (drawn); the shooter was killed by it. */
  private blown = false;
  private killed = false;
  private firstShotAt: number | null = null;
  private shotsAtThis = 0;
  private stats = {
    shots: 0,
    hits: 0,
    downs: 0,
    deaths: 0,
    best: null as number | null,
    last: null as number | null,
  };
  private damaged = new Set<string>();
  private lase: { text: string; until: number } | null = null;
  private panel = h("div", { class: "shoot-hud panel" });
  private lastPanel = 0;

  constructor(private readonly flight: Flight) {
    this.weapon = Object.values(content()?.weapons ?? {})[0] ?? null;
    const spec = currentVehicle();
    // Fitted by the session (the mode's start names it): the content's, until it says.
    this.payload = content()?.payloads[PAYLOAD] ?? null;
    this.flies = fliesFor(spec?.firmware.kind);
    this.levels = levelsFor(this.flies);
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
      effect(() => {
        if (damage()?.exploded && !this.blown) this.blowUp();
      }),
    );
    void this.setup();
  }

  /** The shooter and their rifle, once the pad and the place's ground are known. */
  private async setup(): Promise<void> {
    const flight = this.flight;
    const weapon = this.weapon;
    if (!weapon) return;
    const [pad, ground] = await new Promise<[RigContext["pad"], Ground]>((resolve) => {
      const stop = effect(() => {
        const p = flight.pad();
        const g = flight.ground();
        if (p && g) {
          queueMicrotask(() => stop());
          resolve([p, g]);
        }
      });
    });
    if (this.disposed) return;
    this.ground = ground;
    const hd = (pad.heading_deg * Math.PI) / 180;
    const [lat, lon] = moveBy(
      pad.lat,
      pad.lon,
      -Math.cos(hd) * STAND_OFF_M,
      -Math.sin(hd) * STAND_OFF_M,
    );
    // Its ground is built only once someone stands there: until then, the map's.
    const map = await flight.world.groundHeightSettled(lat, lon);
    if (this.disposed) return;
    const [e, n, z] = ground.frame.fromMap(lat, lon, map);
    ground.focus("shooter", [e, n, z]);
    const groundSim = map - mapOffset();
    const w = wind() ?? locations()[status().location ?? ""]?.wind;
    if (!w) return;
    this.air = new Air(w);
    const air = this.air;
    // The wind's height is over the shooter's ground.
    const where = setting(lat, groundSim, (p) => air.at(Math.max(0.1, p[2] - z)));
    setSpeedOfSound(airAt(groundSim).sound);
    this.rounds = new Rounds(flight.world, ground, where, () => {
      const body = flight.body();
      return body ? (now: number) => body.poseAt(now) : null;
    });
    this.rounds.extraZones = this.payload ? [payloadZone(this.payload)] : [];
    this.blasts = new Blasts(flight.world, ground.frame);
    this.rounds.onHit = (hit) => {
      this.stats.hits++;
      const { label, motor } = hit.strike;
      const what = `${label}${motor ? ` ${motor}` : ""}`;
      this.damaged.add(what);
      if (this.drone !== "down") this.drone = "hit";
      toast(`Hit: ${what} at ${hit.range.toFixed(0)} m, ${hit.speed.toFixed(0)} m/s`, "good", 2500);
    };
    const geometry = await rifleGeometry(`/${weapon.model}`);
    if (this.disposed) return;
    const shooter = new Shooter(
      flight.world,
      ground,
      weapon,
      geometry,
      { e, n, z, headingDeg: pad.heading_deg },
      this.rounds,
      where,
    );
    this.shooter = shooter;
    // Aim at where the drone will hover.
    shooter.pitch = Math.atan2(HOVER_M - 1.6, STAND_OFF_M);
    shooter.onLase = () => this.laseNow();
    // Into the frame loop: the controls, the body at its own rate, the rounds, then the view.
    const signal = this.abort.signal;
    const rounds = this.rounds;
    loop.each("shooter input", "input", (_now, dt) => shooter.input(dt), signal);
    loop.fixed("shooter", BODY_HZ, (h) => shooter.step(h), signal);
    loop.each("rounds", "sim", (now) => rounds.update(now), signal);
    loop.each("shoot down", "place", (now, dt) => this.frame(now, dt), signal);
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
      this.hoverAt = null;
      this.pathSent = false;
      this.blown = false;
      this.killed = false;
      link.send(target({ action: "hover", height_m: HOVER_M }));
      const lv = this.levels[this.level];
      hudBanner({
        big: "",
        small: `Level ${this.level + 1} of ${this.levels.length}: ${lv?.name ?? ""}. ${lv?.brief ?? ""}`,
      });
      setTimeout(() => this.drone === "climbing" && hudBanner(null), 6000);
    }
    const down =
      p === "crashed" || (p === "ground" && this.damaged.size > 0 && this.drone !== "climbing");
    if (down && this.drone !== "down") this.downed();
  }

  private downed(): void {
    const now = performance.now();
    this.drone = "down";
    if (!this.killed) {
      this.stats.downs++;
      const t = this.firstShotAt === null ? null : (now - this.firstShotAt) / 1000;
      this.stats.last = t;
      if (t !== null && (this.stats.best === null || t < this.stats.best)) this.stats.best = t;
      hudBanner({
        big: "DOWN",
        small: `${this.shotsAtThis} shot${this.shotsAtThis === 1 ? "" : "s"}${t !== null ? ` · ${t.toFixed(1)} s` : ""}`,
        tone: "leaf",
      });
    }
    setTimeout(() => {
      if (this.disposed || this.drone !== "down") return;
      // Killed, the same level again; else the next (from the first, after the last).
      if (!this.killed) this.level = (this.level + 1) % this.levels.length;
      hudBanner({
        big: "",
        small: this.level === 0 && !this.killed ? "From the top" : "Another drone",
      });
      this.hoverSent = false;
      this.flight.reset();
    }, RESPAWN_S * 1000);
  }

  /** Its charge went off (the physics says so): the blast where it is drawn, and the shooter
   * killed if within its lethal radius. The drone is gone: down. */
  private blowUp(): void {
    this.blown = true;
    const s = this.shooter;
    const ground = this.ground;
    const now = performance.now();
    const pose = this.flight.body()?.poseAt(now);
    if (!s || !ground || !pose || !this.blasts) return;
    const at = ground.frame.fromSim(pose.lat, pose.lon, pose.alt);
    const under = ground.heightAt(at[0], at[1], at[2] + 5);
    const ear = s.eyeLocal;
    const rel: Vec3 = [at[0] - ear[0], at[1] - ear[1], at[2] - ear[2]];
    const range = Math.hypot(...rel);
    const pan = (rel[0] * Math.cos(s.heading) - rel[1] * Math.sin(s.heading)) / (range || 1);
    this.blasts.burst(at, under === undefined ? 99 : at[2] - under, ear, pan, now);
    const chest: Vec3 = [ear[0], ear[1], ear[2] - CHEST_BELOW_EYE_M];
    const away = Math.hypot(at[0] - chest[0], at[1] - chest[1], at[2] - chest[2]);
    if (this.payload && away < this.payload.lethal_m) {
      this.killed = true;
      this.stats.deaths++;
      hudBanner({
        big: "KILLED",
        small: `Its charge went off ${away.toFixed(1)} m from you`,
        tone: "rose",
      });
    }
    if (this.drone !== "down") this.downed();
  }

  /** At a level that flies: its path, as targets every so often (ArduCopter) or a mission sent
   * once (Betaflight). */
  private fly(now: number, pose: Pose | null): void {
    const lv = this.levels[this.level];
    const s = this.shooter;
    const ground = this.ground;
    const pad = this.flight.pad();
    if (!lv || lv.id === "hover" || !s || !ground || !pad || !pose) return;
    if (
      this.drone === "hovering" &&
      this.hoverAt !== null &&
      now - this.hoverAt > HOLD_FIRST_S * 1000
    ) {
      this.drone = "flying";
      this.hoverAt = now;
    }
    if (this.drone !== "flying") return;
    const frame = ground.frame;
    const padAt = frame.fromSim(pad.lat, pad.lon, pad.alt);
    const hover: Vec3 = [padAt[0], padAt[1], padAt[2] + HOVER_M];
    const hd = (pad.heading_deg * Math.PI) / 180;
    const away: Vec3 = [Math.sin(hd), Math.cos(hd), 0];
    const across: Vec3 = [Math.cos(hd), -Math.sin(hd), 0];
    if (this.flies === "mission") {
      if (this.pathSent) return;
      this.pathSent = true;
      const points = pathWaypoints(lv.id, hover, across, away).map((w): [number, number] => {
        const g = frame.toGeo(w);
        return [g.lat, g.lon];
      });
      link.send(target({ action: "path", points, height_m: HOVER_M }));
      return;
    }
    if (now - this.lastGo < 1000 / GO_HZ) return;
    this.lastGo = now;
    const yaw = (v: Vec3) => (Math.atan2(v[0], v[1]) * 180) / Math.PI;
    const ned = (v: Vec3): [number, number, number] => [v[1], v[0], -v[2]];
    const p = pathAt(lv.id, (now - (this.hoverAt ?? now)) / 1000, hover, across, away);
    const geo = frame.toGeo(p.at);
    const moving = Math.hypot(...p.vel) > 0.5;
    link.send(
      target({
        action: "go",
        lat: geo.lat,
        lon: geo.lon,
        height_m: p.at[2] - padAt[2],
        vel_ned: ned(p.vel),
        ...(moving ? { yaw_deg: yaw(p.vel) } : {}),
      }),
    );
  }

  /** Each frame, once the body has stepped and the rounds flown: the eye, the rifle, the sight,
   * the drone's sound, the panel. */
  private frame(now: number, dt: number): void {
    const s = this.shooter;
    if (!s || !this.rounds || !this.air) return;
    const pose = this.flight.pose(now);
    const w = wind();
    if (w) this.air.wind = w;
    this.air.step(dt);
    const before = s.fired;
    const picture = s.place(now);
    this.rounds.keepShaders(s.eyeLocal, s.lookDir);
    if (s.fired > before) {
      this.stats.shots += s.fired - before;
      this.shotsAtThis += s.fired - before;
      if (this.firstShotAt === null && this.drone !== "down") this.firstShotAt = now;
    }
    this.sights.draw(picture);
    this.droneSound(now, pose);
    this.blasts?.update(now);
    if (
      this.drone === "climbing" &&
      pose &&
      pose.alt - (this.flight.pad()?.alt ?? 0) > HOVER_M * 0.6
    ) {
      this.drone = "hovering";
    }
    // Up to height: the clock for a level that flies starts.
    if (
      this.drone === "hovering" &&
      this.hoverAt === null &&
      pose &&
      pose.alt - (this.flight.pad()?.alt ?? 0) > HOVER_M - 2
    ) {
      this.hoverAt = now;
    }
    this.fly(now, pose);
    if (now - this.lastPanel > 100) {
      this.lastPanel = now;
      this.drawPanel(now);
    }
  }

  /** The drone heard from the shooter: as it was when the sound left it. */
  private droneSound(now: number, pose: Pose | null): void {
    const s = this.shooter;
    const body = this.flight.body();
    const frame = this.ground?.frame;
    if (!s || !pose || !body || !frame) return;
    const local = (p: Pose): Vec3 => frame.fromSim(p.lat, p.lon, p.alt);
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
      phase() === "crashed" || this.blown || droneGain(range) <= 0
        ? []
        : [{ id: "target", kind: "multirotor", level: mean, rpmNorm: mean, each, range, pan }],
    );
  }

  private laseNow(): void {
    const s = this.shooter;
    const body = this.flight.body();
    const ground = this.ground;
    if (!s || !this.rounds || !ground) return;
    const look = s.lookDir;
    const eye = s.eyeLocal;
    const pose = body?.poseAt(performance.now());
    let range: number | null = null;
    if (pose) {
      const p = ground.frame.fromSim(pose.lat, pose.lon, pose.alt);
      const d: Vec3 = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
      const along = d[0] * look[0] + d[1] * look[1] + d[2] * look[2];
      const miss = Math.hypot(
        d[0] - look[0] * along,
        d[1] - look[1] * along,
        d[2] - look[2] * along,
      );
      if (along > 0 && miss < 0.15 + (BEAM_RAD * along) / 2) range = along;
    }
    // Else the ground, where it is built near; else whatever is drawn there.
    if (range === null) range = ground.cast(eye, look, 400)?.toi ?? null;
    if (range === null) {
      const world = this.flight.world;
      const c = world.scene.canvas;
      const hit = world.pickPoint(new C.Cartesian2(c.clientWidth / 2, c.clientHeight / 2));
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
    const ob = s.obstacle;
    const doing = { vault: "vaulting", climb: "climbing", push: "pushing through" };
    const ahead = {
      vault: "low wall · walk on or Space to vault",
      climb: "high wall · Space to climb",
      wall: "too high · lean in to push through",
    };
    if (ob.doing) rows.push(row("Ahead", doing[ob.doing]));
    else if (ob.ahead) rows.push(row("Ahead", ahead[ob.ahead]));
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
    const lv = this.levels[this.level];
    rows.push(row("Level", `${this.level + 1}/${this.levels.length} · ${lv?.name ?? ""}`));
    const droneText = {
      starting: "starting",
      climbing: "taking off",
      hovering: `hovering ${HOVER_M} m`,
      flying: `flying: ${lv?.name.toLowerCase() ?? ""}`,
      hit: this.damaged.size ? `hit: ${[...this.damaged].join(", ")}` : "hit",
      down: this.blown ? "blown up" : "down",
    }[this.drone];
    rows.push(row("Drone", droneText, this.drone === "down" ? "good" : ""));
    const st = this.stats;
    rows.push(
      row(
        "Score",
        `${st.downs} down · ${st.hits} hits / ${st.shots} shots${st.deaths ? ` · killed ${st.deaths}` : ""}${st.best !== null ? ` · best ${st.best.toFixed(1)} s` : ""}`,
      ),
    );
    if (document.pointerLockElement !== this.flight.world.scene.canvas)
      rows.push(h("div", { class: "row hint" }, "Click to take the mouse"));
    this.panel.replaceChildren(...rows);
  }

  /** A rifleman has no map: the minimap is hidden in this mode. */
  marks() {
    return [];
  }

  dispose(): void {
    this.disposed = true;
    this.abort.abort();
    this.ground?.focus("shooter", null);
    radio.onFoot = false;
    for (const d of this.disposers) d();
    this.shooter?.dispose();
    this.rounds?.dispose();
    this.blasts?.dispose();
    this.sights.dispose();
    sounds.heard([]);
    hudPanel(null);
    hudBanner(null);
  }
}
