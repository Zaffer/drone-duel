// The driver's turn in the duel (duel.js), in the COP: the map turns into the drive page
// (drivesession.js) on the pilot's point B, inside the pilot's fence. It draws the pilot's
// drones as they come, with their ranges, and sends the truck's pose back. The pilot's page
// judges the strikes and crashes; this one shows them.

import { packModelMatrix, PoseTrack } from "./duel.js";
import { DriveSession } from "./drivesession.js";
import { callsign, DRONE_MODEL } from "./fleet.js";
import { poof } from "./smoke.js";

const C = Cesium;
const $ = (id) => document.getElementById(id);

/** The truck's pose goes to the pilot this often. */
const TRUCK_HZ = 20;
/** Each drone is drawn this long after its pose comes: the pilot sends them 10 times a second. */
const DRONE_DELAY_S = 0.2;
/** No pose this long: the drone has gone from the pilot's telemetry. */
const DRONE_STALE_MS = 2000;
/** A point B this far from the drive's start needs a new start there. */
const B_MOVED_M = 30;
const THREAT = C.Color.fromCssColorString("#ff5a5a");
const FENCE = C.Color.fromCssColorString("#ff5a5a");
/** The fence is drawn as a see-through wall this far below and above point B's ground. */
const FENCE_WALL_M = { below: 15, above: 12 };

export class DriverRole {
  /**
   * @param {object} ctx map, link (DuelLink), match (DuelMatch), banner(text, kind),
   *   setDriving(on): the COP hands the map over to the drive, or takes it back
   */
  constructor({ map, link, match, banner, setDriving }) {
    this.map = map;
    this.viewer = map.viewer;
    this.link = link;
    this.match = match;
    this.banner = banner;
    this.setDriving = setDriving;
    /** @type {DriveSession | null} */
    this.session = null;
    this._field = null;
    this._destroyed = false;
    this._sentAt = 0;
    /** sysid -> {track, armed, out} */
    this.drones = new Map();
    this.source = new C.CustomDataSource("duel");
    this.viewer.dataSources.add(this.source);
    this._fenceLocal = [];
    link.on("drones", (list) => this._drones(list));
    link.on("hit", (h) => this._hit(h));
    link.on("lost", (l) => this._lost(l));
    this._timer = setInterval(() => this._render(), 250);
    setDriving(true);
    this._notice("Waiting for the pilot's field…");
    if (match.field) this.onField(match.field);
  }

  destroy() {
    this._destroyed = true;
    clearInterval(this._timer);
    for (const type of ["drones", "hit", "lost"]) this.link.on(type, null);
    this.session?.destroy();
    this.session = null;
    this.viewer.dataSources.remove(this.source, true);
    this.setDriving(false);
  }

  /** The pilot's field: drive from its point B, inside its fence. */
  async onField(field) {
    const [lat, lon] = field.b ?? [];
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;
    const old = this._field?.b;
    this._field = field;
    if (this.session && old) {
      const moved = C.Cartesian3.distance(C.Cartesian3.fromDegrees(old[1], old[0]), C.Cartesian3.fromDegrees(lon, lat));
      if (moved < B_MOVED_M) {
        this._placeFence();
        return;
      }
    }
    this.session?.destroy();
    this.session = null;
    this._notice("Finding the ground…");
    const session = await DriveSession.start(this.map, {
      lat,
      lon,
      ui: {
        notice: (text) => this._notice(text),
        hud: { speed: $("drive-speed"), wheels: $("drive-wheels") },
        camMode: $("drive-cam"),
        help: $("drive-help"),
      },
    });
    if (this._destroyed || this._field !== field) {
      session.destroy();
      return;
    }
    this.session = session;
    session.outside = (x, y) => this._outside(x, y);
    session.onStep = (s) => this._sendTruck(s);
    this._placeFence();
  }

  /** The match moved on: a turn starting puts the truck back on B. */
  phaseChanged(previous, phase) {
    if (phase === "live" && previous !== "live") this.session?.placeAt(0, 0, this.session.car.heading, 0);
  }

  _notice(text) {
    $("drive-notice").hidden = !text;
    $("drive-notice").textContent = text ?? "";
  }

  _sendTruck(session) {
    if (!session.placed) return;
    const now = performance.now();
    if (now - this._sentAt < 1000 / TRUCK_HZ) return;
    this._sentAt = now;
    this.link.send("car", [...packModelMatrix(session.car.modelMatrix), Math.round(session.car.speed * 10) / 10]);
  }

  /** Whether local (x, y) is outside the fence. Never, without a fence. */
  _outside(x, y) {
    const poly = this._fenceLocal;
    if (poly.length < 3) return false;
    let inside = false;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const [a, b] = [poly[i], poly[j]];
      if (a.y > y !== b.y > y && x < ((b.x - a.x) * (y - a.y)) / (b.y - a.y) + a.x) inside = !inside;
    }
    return !inside;
  }

  _placeFence() {
    this.source.entities.removeById("duel-fence");
    this._fenceLocal = [];
    const corners = this._field?.fence ?? [];
    if (!this.session || corners.length < 3) return;
    const h = C.Cartographic.fromCartesian(this.session.origin).height;
    const ring = [...corners, corners[0]];
    this._fenceLocal = corners.map(([lat, lon]) => this.session.toLocal(lat, lon, h));
    this.source.entities.add({
      id: "duel-fence",
      wall: {
        positions: ring.map(([lat, lon]) => C.Cartesian3.fromDegrees(lon, lat, h)),
        minimumHeights: ring.map(() => h - FENCE_WALL_M.below),
        maximumHeights: ring.map(() => h + FENCE_WALL_M.above),
        material: FENCE.withAlpha(0.14),
        outline: true,
        outlineColor: FENCE.withAlpha(0.7),
      },
    });
  }

  _drones(list) {
    if (!Array.isArray(list)) return;
    for (const [sysid, x, y, z, qx, qy, qz, qw, armed] of list) {
      if (!Number.isInteger(sysid)) continue;
      const d = this.drones.get(sysid) ?? this._addDrone(sysid);
      d.track.add([x, y, z, qx, qy, qz, qw]);
      d.armed = Boolean(armed);
    }
  }

  _addDrone(sysid) {
    const d = { sysid, track: new PoseTrack(DRONE_DELAY_S), armed: false, out: false };
    this.drones.set(sysid, d);
    const shown = () => !d.out && d.track.age < DRONE_STALE_MS;
    const threat = new C.CallbackProperty(() => shown() && d.armed, false);
    this.source.entities.add({
      position: d.track.position,
      orientation: d.track.orientation,
      model: {
        uri: DRONE_MODEL.uri,
        scale: DRONE_MODEL.scale,
        minimumPixelSize: 24,
        maximumScale: 150,
        show: new C.CallbackProperty(shown, false),
      },
      // Over buildings and through them: where each armed drone is, and how far.
      point: {
        pixelSize: 8,
        color: THREAT,
        outlineColor: C.Color.BLACK,
        outlineWidth: 1.5,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: threat,
      },
      label: {
        text: new C.CallbackProperty(() => {
          const range = this._range(d);
          return `${callsign(sysid)}${range === undefined ? "" : ` ${Math.round(range)} m`}`;
        }, false),
        font: "600 12px Inter, Arial, sans-serif",
        fillColor: THREAT,
        outlineColor: C.Color.BLACK,
        outlineWidth: 3,
        style: C.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new C.Cartesian2(0, -18),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: threat,
      },
    });
    return d;
  }

  /** Drone `d`'s distance from the truck as drawn, metres; undefined without both. */
  _range(d) {
    const p = d.track.position.getValue(C.JulianDate.now(), scratchDrone);
    if (!p || !this.session?.placed) return undefined;
    return C.Cartesian3.distance(p, C.Matrix4.getTranslation(this.session.car.modelMatrix, scratchTruck));
  }

  _poof(at) {
    if (Array.isArray(at) && at.length === 3 && at.every(Number.isFinite)) {
      poof(this.viewer.scene, new C.Cartesian3(...at));
    }
  }

  _hit({ sysid, at, counted }) {
    this._poof(at);
    const flash = $("hit-flash");
    flash.classList.remove("on");
    void flash.offsetWidth; // restart the animation
    flash.classList.add("on");
    const name = callsign(sysid);
    if (!counted) {
      this.banner(`${name} struck the truck (before the turn: no score)`);
      return;
    }
    this.banner(`${name} struck the truck`, "bad");
    this.match.hitBy(sysid);
    const d = this.drones.get(sysid);
    if (d) d.out = true;
  }

  _lost({ sysid, at, why }) {
    this._poof(at);
    const d = this.drones.get(sysid);
    if (d) d.out = true;
    this.banner(`${callsign(sysid)} ${String(why ?? "crashed").slice(0, 60)}: out`, "win");
  }

  _render() {
    let nearest;
    for (const d of this.drones.values()) {
      if (d.out || !d.armed || d.track.age >= DRONE_STALE_MS) continue;
      const range = this._range(d);
      if (range !== undefined && (nearest === undefined || range < nearest.range)) nearest = { d, range };
    }
    $("drive-near").textContent = nearest ? `${callsign(nearest.d.sysid)} ${Math.round(nearest.range)} m` : "–";
  }
}

const scratchDrone = new C.Cartesian3();
const scratchTruck = new C.Cartesian3();
