// The pilot's turn in the duel (duel.js), in the COP: sends the driver the field and the
// drones, draws the driver's truck, and judges each drone. A drone that strikes the truck
// scores and is spent; one that hits the ground or anything else is out. Either way its sim
// copter is stopped dead (/sim/destroy): its link goes quiet, as a destroyed drone's would.
// After the turn every copter starts again where it began (/sim/reset, duelmatch.js).
//
// Judged on what the pilot sees:
// - a strike: an armed drone, as drawn, within HIT_MARGIN_M of the truck's box as drawn;
// - a crash: a drone that has flown, out of LAND mode, at the ground (under GROUND_M above
//   home), OBSTACLE_M inside the map's 3D tiles (a tree, a building, a hill), or within
//   COLLIDE_M of another drone. LAND puts a drone down safely.

import { packPose, PoseTrack } from "./duel.js";
import { callsign, TRUCK_MODEL } from "./fleet.js";
import { poof } from "./smoke.js";
import { sfx } from "./sounds.js";

const C = Cesium;

/** The truck is drawn this long after its pose comes (PoseTrack). */
const TRUCK_DELAY_S = 0.15;
/** No pose this long: the driver's page has stopped; the truck goes from the map. */
const TRUCK_STALE_MS = 2000;
/** A drone this close to the truck's box has struck it. The drawn drone is 1.4 m across. */
const HIT_MARGIN_M = 1;
/** A drone must get this much further away before it strikes again (before the turn). */
const HIT_RELEASE_M = 1.5;
/** A drone this high above home has flown: from then on the ground is a crash. */
const AIRBORNE_M = 2;
const GROUND_M = 0.4;
/** A drone this far under the 3D tiles' surface has flown into something. Coarse tiles far
 * from the camera are a metre or so out, so not less. */
const OBSTACLE_M = 1;
/** Two 5 inch quads this close, centre to centre, have collided. */
const COLLIDE_M = 0.6;
const TRUCK_CSS = "#ff5a5a";
/** The score's sound comes this long after the strike's. */
const SCORE_AFTER_MS = 180;

/** POST to the viewer's sim controls (swarm.view); a failure is only logged. */
export function simPost(path, body = {}) {
  return fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  }).catch((err) => console.warn(`duel: ${path} failed`, err));
}

export class PilotRole {
  /**
   * @param {object} ctx fleet, map, config (the page's), link (DuelLink), match (DuelMatch),
   *   toast(text, kind)
   */
  constructor({ fleet, map, config, link, match, toast }) {
    this.fleet = fleet;
    this.map = map;
    this.viewer = map.viewer;
    this.config = config;
    this.link = link;
    this.match = match;
    this.toast = toast;
    this.track = new PoseTrack(TRUCK_DELAY_S);
    this.truckSpeed = 0;
    /** sysid -> whether that drone was on the truck when last looked at. */
    this._onTruck = new Map();
    /** Drones that have flown this turn, so the ground is a crash for them. */
    this._airborne = new Set();

    const live = new C.CallbackProperty(() => this.truckLive, false);
    const entities = this.viewer.entities;
    this._entities = [
      entities.add({
        id: "duel-truck",
        name: "DUEL TRUCK",
        position: this.track.position,
        orientation: this.track.orientation,
        model: { uri: TRUCK_MODEL.uri, minimumPixelSize: 32, maximumScale: 20, show: live },
      }),
      // Its label is a map symbol, out of the camera feed: an entity of its own.
      entities.add({
        id: "duel-truck-label",
        position: this.track.position,
        label: {
          text: new C.CallbackProperty(() => `TRUCK · ${Math.round(Math.abs(this.truckSpeed) * 3.6)} km/h`, false),
          font: "600 12px Inter, Arial, sans-serif",
          fillColor: C.Color.fromCssColorString(TRUCK_CSS),
          outlineColor: C.Color.BLACK,
          outlineWidth: 3,
          style: C.LabelStyle.FILL_AND_OUTLINE,
          pixelOffset: new C.Cartesian2(0, -34),
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          show: live,
        },
      }),
    ];
    this._unlisten = this.viewer.scene.preRender.addEventListener(() => this._checkHits());
    link.on("car", (pose) => {
      this.track.add(pose);
      this.truckSpeed = Number(pose[7]) || 0;
    });
    this.sendField();
  }

  destroy() {
    this._unlisten();
    for (const e of this._entities) this.viewer.entities.remove(e);
    this.link.on("car", null);
    this.fleet.duelTruck = null;
  }

  /** The driver's truck is on the map, its poses coming. */
  get truckLive() {
    return this.link.connected && this.track.age < TRUCK_STALE_MS;
  }

  /** Where the truck is drawn now; for the range and bearing line (Fleet.duelTruck). */
  currentPosition() {
    return this.track.position.getValue(C.JulianDate.now());
  }

  /** The driver's field: point B as the watcher has it, else mission.toml's, and the fence. */
  sendField() {
    const b = this.fleet.frame?.target ?? this.config.pointB;
    this.link.send("field", { b, fence: this.config.fence ?? [] });
  }

  /** The drones ready (link on, the EKF navigating, not out), of all, and how many are in the
   * air. */
  readiness() {
    let up = 0;
    let flying = 0;
    for (const view of this.fleet.vehicles.values()) {
      const d = view.data;
      if (!d?.link || this.fleet.lost.has(view.sysid)) continue;
      if (d.nav) up++;
      if (d.armed && d.alt > AIRBORNE_M) flying++;
    }
    return { up, total: this.fleet.vehicles.size, flying };
  }

  turnStarted() {
    this._airborne.clear();
    this._onTruck.clear();
  }

  /** Each telemetry frame: judge the drones in a turn, and send the driver those flying. */
  frame() {
    if (this.match.state.phase === "live") this._checkCrashes();
    if (!this.link.connected) return;
    const drones = [];
    for (const view of this.fleet.vehicles.values()) {
      if (!view.data?.link || this.fleet.lost.has(view.sysid)) continue;
      const pose = view.framePose();
      if (pose) drones.push([view.sysid, ...packPose(pose.pos, pose.quat), view.data.armed ? 1 : 0]);
    }
    this.link.send("drones", drones);
  }

  _checkCrashes() {
    const fleet = this.fleet;
    const flying = [];
    for (const view of fleet.vehicles.values()) {
      const d = view.data;
      if (!d?.link || fleet.lost.has(view.sysid)) continue;
      if (d.alt > AIRBORNE_M) this._airborne.add(view.sysid);
      if (d.mode === "LAND") {
        if (d.alt < GROUND_M) this._airborne.delete(view.sysid); // down safely
        continue;
      }
      const pose = view.framePose();
      if (!this._airborne.has(view.sysid) || !pose) continue;
      if (d.alt < GROUND_M) this._out(view.sysid, pose.pos, "hit the ground");
      else if (this._insideMap(pose.pos)) this._out(view.sysid, pose.pos, "flew into the scenery");
      else flying.push({ sysid: view.sysid, pos: pose.pos });
    }
    for (let i = 0; i < flying.length; i++) {
      for (let j = i + 1; j < flying.length; j++) {
        const [a, b] = [flying[i], flying[j]];
        if (C.Cartesian3.distance(a.pos, b.pos) >= COLLIDE_M) continue;
        this._out(a.sysid, a.pos, `collided with ${callsign(b.sysid)}`);
        this._out(b.sysid, b.pos, `collided with ${callsign(a.sysid)}`);
      }
    }
  }

  /** Whether `pos` is OBSTACLE_M or more under the 3D tiles' surface there. */
  _insideMap(pos) {
    const tileset = this.map.tileset;
    if (!tileset) return false;
    const carto = C.Cartographic.fromCartesian(pos, undefined, scratchCarto);
    let surface;
    try {
      surface = tileset.getHeight(carto, this.viewer.scene);
    } catch {
      return false; // the tiles there are not loaded
    }
    return surface !== undefined && surface - carto.height > OBSTACLE_M;
  }

  /** Drone `sysid` is out: off the map, its sim copter stopped dead. */
  _out(sysid, at, why, tellDriver = true) {
    if (this.fleet.lost.has(sysid)) return;
    this.fleet.lost.add(sysid);
    this._onTruck.delete(sysid);
    poof(this.viewer.scene, at);
    if (tellDriver) this.link.send("lost", { sysid, at: [at.x, at.y, at.z], why });
    if (why) {
      this.toast(`${callsign(sysid)} ${why}: out`, "bad");
      sfx.play("crash");
    }
    if (this.config.sim) simPost("/sim/destroy", { sysid });
    const all = [...this.fleet.vehicles.keys()];
    if (all.length && all.every((s) => this.fleet.lost.has(s))) this.match.pilotOut();
  }

  /** Every rendered frame: has an armed drone, as drawn, come onto the truck, as drawn? */
  _checkHits() {
    const live = this.truckLive;
    this.fleet.duelTruck = live ? this : null;
    const truck = live ? this.track.modelMatrix(scratchTruck) : undefined;
    if (!truck) {
      this._onTruck.clear();
      return;
    }
    const toTruck = C.Matrix4.inverseTransformation(truck, scratchInverse);
    const { length, width, height } = TRUCK_MODEL.size;
    for (const view of this.fleet.vehicles.values()) {
      const out = this.fleet.lost.has(view.sysid);
      const p = view.data?.armed && !out ? view.currentPosition() : undefined;
      if (!p) {
        this._onTruck.delete(view.sysid);
        continue;
      }
      // The truck's frame: +x forward, +y left, +z up, from the ground under its middle.
      const local = C.Matrix4.multiplyByPoint(toTruck, p, scratchLocal);
      const was = this._onTruck.get(view.sysid);
      const pad = was ? HIT_MARGIN_M + HIT_RELEASE_M : HIT_MARGIN_M;
      const on =
        Math.abs(local.x) <= length / 2 + pad &&
        Math.abs(local.y) <= width / 2 + pad &&
        local.z >= -pad &&
        local.z <= height + pad;
      this._onTruck.set(view.sysid, on);
      // A drone already on the truck when it was first seen has not just struck it.
      if (on && was === false) this._strike(view.sysid, C.Cartesian3.clone(p));
    }
  }

  _strike(sysid, at) {
    const counted = this.match.state.phase === "live";
    this.link.send("hit", { sysid, at: [at.x, at.y, at.z], counted });
    sfx.play("strike");
    if (!counted) {
      poof(this.viewer.scene, at);
      this.toast(`${callsign(sysid)} struck the truck (before the turn: no score)`);
      return;
    }
    this.toast(`${callsign(sysid)} struck the truck: +1`, "leader");
    setTimeout(() => sfx.play("score"), SCORE_AFTER_MS);
    this.match.hitBy(sysid);
    this._out(sysid, at, "", false); // spent
  }
}

const scratchTruck = new C.Matrix4();
const scratchInverse = new C.Matrix4();
const scratchLocal = new C.Cartesian3();
const scratchCarto = new C.Cartographic();
