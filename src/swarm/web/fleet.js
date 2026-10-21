// Map entities for the swarm: one VehicleView per copter (model, 2525 symbol, label, trail,
// altitude stalk, slot, sensor frustum), the leader's control area, point B with its
// landing radius and the truck parked on it, and fixed markers. A drone striking the truck
// puffs white smoke.
//
// Every drone is drawn in one of two colours: blue while a human flies it (armed in a pilot
// mode), orange otherwise, when the autopilot is in charge.
//
// Frames arrive at the coordinator rate (10 Hz). Each one is scheduled DELAY_S in the future
// on the system clock, so Cesium interpolates between the last two frames instead of jumping.
// ?delay=0 in the page's address shows each frame as it comes instead.
//
// The camera feed (Fleet.feed) shows what a drone's camera would: in the sim, every drone
// where its physics (swarm.physics) puts it, as each state arrives, at its real size, and the
// truck; none of the map's symbols, labels, trails, slots or plans.
//
// In the duel (duelpilot.js) the other player's truck stands in for the one parked on B
// (Fleet.duelTruck): B's truck is put away, and the range and bearing line points at theirs.
// A drone the duel has destroyed (Fleet.lost) goes from the map until its sim brings it back.

import { cameraAxes, FadeMaterialProperty, viewCorners } from "./sensor.js";
import { poof } from "./smoke.js";
import { pointMarker, sidcUav } from "./symbols.js";

const C = Cesium;

const LEADER_CSS = "#f2f5f7";
const TARGET_CSS = "#ff5a5a";
const MARKER_CSS = "#f2f5f7";

const DELAY_S = Number(new URLSearchParams(location.search).get("delay") ?? 0.25);
/** A drone's physics this old is gone: the camera feed falls back to its telemetry. */
const PHYSICS_STALE_MS = 500;
/** Entities the camera feed shows: real things, not map symbols. */
const PHYSICAL_ID = /^(vehicle-\d+|target-B-truck|duel-truck)$/;
const KEEP_SAMPLES_S = 5;
const TRAIL_S = 120;
const TRAIL_STEP_S = 0.5;
/** Sketchfab's "FPV Drone" (PinaPipe, CC BY 4.0), converted by swarm.fpvmodel. */
const MODEL_URI = "/models/fpv-drone.glb";
/** The model's nose is glTF +Z; this turns it to face the vehicle heading. */
const MODEL_HEADING_OFFSET = -Math.PI / 2;
/** The drone is 0.31 m across; drawn about 1.4 m so it reads at map distances. */
const MODEL_SCALE = 4.5;
const MODEL_LIFT_M = 0.2;
/** Sketchfab's "Military Truck" (Raphael Escamilla, CC BY 4.0), converted by swarm.truckmodel,
 * parked on point B. In metres, its front on glTF +Z like the drone's nose. */
const TRUCK_URI = "/models/military-truck.glb";
/** Which way the parked truck faces, degrees true. */
const TRUCK_HEADING_DEG = 0;
/** The truck's size, mirrors included (swarm.truckmodel). */
const TRUCK_SIZE_M = { length: 4.9, width: 2.3, height: 1.75 };
/** The drone and truck models, for the duel's pages to draw them as the COP does. */
export const DRONE_MODEL = { uri: MODEL_URI, scale: MODEL_SCALE, headingOffset: MODEL_HEADING_OFFSET };
export const TRUCK_MODEL = { uri: TRUCK_URI, size: TRUCK_SIZE_M };
/** A drone this close to the truck has struck it, allowing for GPS error. */
const HIT_MARGIN_M = 1;
/** A drone must get this much further away before its next hit counts, so GPS jitter at the
 * edge does not puff twice. */
const HIT_RELEASE_M = 1.5;
const M_PER_DEG_LAT = 111_320;
const AIRBORNE_M = 2;
/** Propeller turns per second: an armed and flying indicator, not the real rate. Disarmed
 * propellers stand still. */
const PROP_TURNS_PER_S = { armed: 0.25, flying: 1 };
/** Seconds for the propellers to settle at a new rate. */
const PROP_SETTLE_S = 0.8;
/** The model's propeller nodes and their turn seen from above, +1 anticlockwise: ArduPilot's
 * quad X, which swarm.fpvmodel checks the blades are pitched for. */
const PROPS = {
  prop_front_right: 1,
  prop_rear_left: 1,
  prop_front_left: -1,
  prop_rear_right: -1,
};
/** How far ahead the camera's view lines reach, fading out on the way. */
const SENSOR_RANGE_M = 60;
/** The image frame drawn across the view lines, as a fraction of SENSOR_RANGE_M. */
const SENSOR_FRAME_AT = 0.3;
const SYMBOL_LIFT_PX = 50;
/** Text labels hide beyond this camera distance; the 2525 symbols stay. */
const LABEL_RANGE_M = 1500;
/** Followers lift off only this close to the leader (swarm.launch.LEADER_RADIUS_M). */
const LEADER_RADIUS_M = 50;

/** Each drone's name, by MAVLink system id; SWARM n beyond these. */
const CALLSIGNS = { 1: "LEADER", 2: "ALPHA", 3: "BRAVO", 4: "CHARLIE", 5: "DELTA" };
export const callsign = (sysid) => CALLSIGNS[sysid] ?? `SWARM ${sysid}`;
/** Colours the operator can give a drone from its fleet card. */
export const DRONE_COLORS = {
  black: "#15181c",
  white: "#f2f4f6",
  red: "#e0352b",
  blue: "#2a78d6",
  green: "#2fa84f",
  yellow: "#f5c518",
  orange: "#eb6834",
  purple: "#8e4ec6",
  pink: "#ec5fa8",
  grey: "#8a929b",
};
/** Every drone's colour until one is picked for it. */
export const DEFAULT_COLOR = "blue";
/** How a drone is being flown, from a frame vehicle, as MIL-STD-2525E's common modifiers
 * name it. RP (remotely piloted): a pilot flies it (armed in a pilot mode). AUT (autonomous
 * control): the autopilot does. Null while disarmed. */
export const controlMode = (v) => (!v?.armed ? null : v.pilot ? "RP" : "AUT");

const shades = new Map();
/** Cesium colour for a CSS colour at an alpha; cached so callbacks return stable objects. */
function shade(css, alpha = 1) {
  const key = `${css}/${alpha}`;
  let c = shades.get(key);
  if (!c) {
    c = C.Color.fromCssColorString(css).withAlpha(alpha);
    shades.set(key, c);
  }
  return c;
}

/** "412 m · 263°": ground distance and true bearing from one point to another. */
export function rangeBearing(from, to) {
  const a = C.Cartographic.fromCartesian(from);
  const b = C.Cartographic.fromCartesian(to);
  const geodesic = new C.EllipsoidGeodesic(a, b);
  const bearing = (C.Math.toDegrees(geodesic.startHeading) + 360) % 360;
  return `${Math.round(geodesic.surfaceDistance)} m · ${String(Math.round(bearing) % 360).padStart(3, "0")}°`;
}

// Polylines use arcType NONE. With the default GEODESIC, Cesium's PolylinePipeline.generateArc
// miscounts zero-length segments (repeated points, e.g. a drone hovering still) and writes the
// last point into the middle of the line, drawing a V from the drone back into its trail.

const hold = (prop) => {
  prop.forwardExtrapolationType = C.ExtrapolationType.HOLD;
  prop.backwardExtrapolationType = C.ExtrapolationType.HOLD;
  return prop;
};

const later = (seconds) => C.JulianDate.addSeconds(C.JulianDate.now(), seconds, new C.JulianDate());

const olderThan = (seconds) =>
  new C.TimeInterval({
    start: C.JulianDate.fromIso8601("2000-01-01T00:00:00Z"),
    stop: later(-seconds),
  });

/** Closest heading to `previous` that is equivalent to `heading`, so interpolation never spins. */
function unwrap(heading, previous) {
  if (previous === undefined) return heading;
  let h = heading;
  while (h - previous > Math.PI) h -= 2 * Math.PI;
  while (h - previous < -Math.PI) h += 2 * Math.PI;
  return h;
}

export class Fleet {
  /**
   * @param {import("./basemap.js").BaseMap} map
   * @param {object} options display options shared with the UI (mutated there)
   */
  constructor(map, options, config) {
    this.map = map;
    this.viewer = map.viewer;
    this.options = options;
    this.config = config;
    /** @type {Map<number, VehicleView>} */
    this.vehicles = new Map();
    this.frame = null;
    this.originBase = null;
    this.selected = null;
    /** Every drone selected (the ALL button): outlined in its colour, its sensor shown in
     * "selected" mode. `selected` stays the one the camera, fence and range to B use. */
    this.selectAll = false;
    /** Vehicle the camera is looking through, hidden from its own view. */
    this.fpv = null;
    /** Showing the camera feed: only physical things, drones on their physics (setFeed). */
    this.feed = false;
    this.viewer.entities.collectionChanged.addEventListener((_, added) => {
      for (const e of added) this._feedShow(e);
    });
    this.target = new TargetView(this);
    /** The duel's truck (duelpilot.js) while its driver is sending it, else null:
     * {currentPosition()}. */
    this.duelTruck = null;
    /** Drones out of the duel, destroyed: not drawn, whatever their last report. */
    this.lost = new Set();
    /** The map is the duel's drive (dueldriver.js): no fleet, symbols or plans on it. */
    this.driving = false;
    this.parkLine = this.viewer.entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._parkLine(), false),
        width: 2,
        material: new C.PolylineDashMaterialProperty({
          color: C.Color.WHITE.withAlpha(0.7),
          dashLength: 12,
        }),
        show: new C.CallbackProperty(() => this.options.slots && this._parked(), false),
      },
    });
    for (const m of config.markers ?? []) this._addMarker(m);
    this._addRangeBearing();
    this.leaderRadius = config.leaderRadiusM ?? LEADER_RADIUS_M;
    this._addControlArea();
  }

  isSelected(sysid) {
    return this.selectAll || this.selected === sysid;
  }

  /** Show the camera feed (`on`) or the map. */
  setFeed(on) {
    this.feed = on;
    for (const e of this.viewer.entities.values) this._feedShow(e);
  }

  _feedShow(entity) {
    if (!PHYSICAL_ID.test(entity.id)) entity.show = !this.feed;
  }

  /** Take in every drone's physics, {sysid: {lat, lon, alt, roll, pitch, yaw}}. */
  physics(states) {
    const now = performance.now();
    for (const [sysid, state] of Object.entries(states)) {
      const v = this.vehicles.get(Number(sysid));
      if (!v) continue;
      v.physics = state;
      v.physicsAt = now;
    }
  }

  /** Drone `sysid`'s colour name: the one picked on its card (options.droneColors), else
   * DEFAULT_COLOR. */
  droneColor(sysid) {
    const name = this.options.droneColors?.[sysid];
    return name in DRONE_COLORS ? name : DEFAULT_COLOR;
  }

  droneCss(sysid) {
    return DRONE_COLORS[this.droneColor(sysid)];
  }

  /** The leader's control area: a ring on the ground, LEADER_RADIUS_M round the leader. An
   * armed follower inside it lifts off once the leader is above 5 m. */
  _addControlArea() {
    const shown = () => this.options.controlArea && this.leaderGround() !== undefined;
    const color = (alpha) =>
      new C.ColorMaterialProperty(
        new C.CallbackProperty(
          () => shade(this.droneCss(this.leader), alpha),
          false,
        ),
      );
    this.viewer.entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._controlRing(), false),
        width: 2.5,
        material: color(0.95),
        depthFailMaterial: color(0.35),
        show: new C.CallbackProperty(shown, false),
      },
    });
    this.viewer.entities.add({
      position: new C.CallbackPositionProperty(() => this._controlRingLabel(), false),
      label: {
        text: `${this.leaderRadius} m`,
        font: "600 11px Inter, Arial, sans-serif",
        fillColor: C.Color.WHITE,
        outlineColor: C.Color.BLACK,
        outlineWidth: 3,
        style: C.LabelStyle.FILL_AND_OUTLINE,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        distanceDisplayCondition: new C.DistanceDisplayCondition(0, LABEL_RANGE_M),
        show: new C.CallbackProperty(shown, false),
      },
    });
  }

  /** The leader's position on the ground below it, or undefined without a placed leader. */
  leaderGround() {
    const lead = this.vehicles.get(this.leader);
    const p = lead?.currentPosition();
    if (!p) return undefined;
    const carto = C.Cartographic.fromCartesian(p);
    return C.Cartesian3.fromRadians(carto.longitude, carto.latitude, lead.base + 0.3);
  }

  _controlRing() {
    const center = this.leaderGround();
    if (!center) return [];
    const enu = C.Transforms.eastNorthUpToFixedFrame(center);
    const pts = [];
    for (let i = 0; i <= 72; i++) {
      const a = (i / 72) * 2 * Math.PI;
      const local = new C.Cartesian3(this.leaderRadius * Math.sin(a), this.leaderRadius * Math.cos(a), 0);
      pts.push(C.Matrix4.multiplyByPoint(enu, local, new C.Cartesian3()));
    }
    return pts;
  }

  _controlRingLabel() {
    const center = this.leaderGround();
    if (!center) return undefined;
    const enu = C.Transforms.eastNorthUpToFixedFrame(center);
    return C.Matrix4.multiplyByPoint(enu, new C.Cartesian3(0, -this.leaderRadius, 0), new C.Cartesian3());
  }

  /** ATAK-style range and bearing line from the selected vehicle to point B. */
  _addRangeBearing() {
    const ends = () => {
      const from = this.vehicles.get(this.selected)?.currentPosition();
      const to = (this.duelTruck ?? this.target).currentPosition();
      return from && to ? [from, to] : null;
    };
    const color = C.Color.fromCssColorString("#f2f5f7");
    const show = new C.CallbackProperty(() => this.options.rangeBearing && ends() !== null, false);
    this.viewer.entities.add({
      position: new C.CallbackPositionProperty(() => {
        const e = ends();
        return e ? C.Cartesian3.midpoint(e[0], e[1], new C.Cartesian3()) : undefined;
      }, false),
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => ends() ?? [], false),
        width: 1.5,
        material: new C.PolylineDashMaterialProperty({ color: color.withAlpha(0.8), dashLength: 16 }),
        depthFailMaterial: color.withAlpha(0.3),
        show,
      },
      label: {
        show,
        text: new C.CallbackProperty(() => {
          const e = ends();
          return e ? rangeBearing(e[0], e[1]) : "";
        }, false),
        font: "600 12px Inter, Arial, sans-serif",
        fillColor: C.Color.WHITE,
        outlineColor: C.Color.BLACK,
        outlineWidth: 3,
        style: C.LabelStyle.FILL_AND_OUTLINE,
        showBackground: true,
        backgroundColor: C.Color.fromCssColorString("rgba(10,14,18,0.62)"),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });
  }

  /** Ingest a live frame. */
  update(frame) {
    this.frame = frame;
    this.originBase ??= this.map.groundHeight(frame.origin[0], frame.origin[1]);
    for (const v of frame.vehicles) this._vehicle(v.sysid).update(v, frame.t);
    this.target.update(frame.target, frame.t);
    this.target.checkHits(this.vehicles.values());
  }

  /** Ingest past frames (on page load) for trails only. */
  history(frames) {
    for (const frame of frames) {
      this.originBase ??= this.map.groundHeight(frame.origin[0], frame.origin[1]);
      for (const v of frame.vehicles) {
        const view = this._vehicle(v.sysid);
        view.data ??= v;
        view.placeBase(v);
        view.addTrailPoint(v, frame.t);
      }
      if (frame.target) this.target.addTrailPoint(frame.target, frame.t);
    }
  }

  get leader() {
    return this.frame?.leader ?? null;
  }

  /** Positions of everything on the map, for framing the camera. */
  allPositions() {
    const out = [];
    for (const v of this.vehicles.values()) {
      const p = v.currentPosition();
      if (p) out.push(p);
    }
    const b = this.target.currentPosition();
    if (b) out.push(b);
    return out;
  }

  _vehicle(sysid) {
    let v = this.vehicles.get(sysid);
    if (!v) {
      v = new VehicleView(this, sysid);
      this.vehicles.set(sysid, v);
    }
    return v;
  }

  _parked() {
    return this.frame?.vehicles.some((v) => v.role === "parked" || v.role === "landing") ?? false;
  }

  _parkLine() {
    const slots = [...this.vehicles.values()]
      .filter((v) => v.data?.role === "parked" && v.slotPosition())
      .map((v) => v.slotPosition());
    if (slots.length < 2) return [];
    // Order along the line: project onto the direction between the two farthest slots.
    let best = [slots[0], slots[1]];
    let bestD = -1;
    for (const a of slots)
      for (const b of slots) {
        const d = C.Cartesian3.distanceSquared(a, b);
        if (d > bestD) [bestD, best] = [d, [a, b]];
      }
    const dir = C.Cartesian3.subtract(best[1], best[0], new C.Cartesian3());
    const along = (p) => C.Cartesian3.dot(C.Cartesian3.subtract(p, best[0], new C.Cartesian3()), dir);
    return slots.sort((a, b) => along(a) - along(b));
  }

  _addMarker(m) {
    this.viewer.entities.add({
      id: `marker-${m.name}`,
      position: C.Cartesian3.fromDegrees(m.lon, m.lat),
      billboard: {
        image: pointMarker(m.name.slice(0, 2), MARKER_CSS).toDataURL(),
        heightReference: C.HeightReference.CLAMP_TO_GROUND,
        verticalOrigin: C.VerticalOrigin.CENTER,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
      label: {
        text: `POINT ${m.name.toUpperCase()}`,
        font: "600 12px Inter, Arial, sans-serif",
        fillColor: C.Color.WHITE,
        outlineColor: C.Color.BLACK,
        outlineWidth: 3,
        style: C.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new C.Cartesian2(0, 26),
        heightReference: C.HeightReference.CLAMP_TO_GROUND,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
      },
    });
  }
}

class VehicleView {
  constructor(fleet, sysid) {
    this.fleet = fleet;
    this.sysid = sysid;
    this.data = null;
    /** Ground height at the vehicle's home, metres above the ellipsoid. Null until known. */
    this.base = null;
    this._basePending = false;
    this._heading = undefined;
    this._roll = undefined;
    /** Propeller angle and rate (turns/s), advanced once per rendered frame. */
    this._props = { angle: 0, rate: 0, time: undefined };
    this._lastPrune = 0;
    /** @type {{t: number, lat: number, lon: number, alt: number, cart?: Cesium.Cartesian3}[]} */
    this.trail = [];
    this.position = hold(new C.SampledPositionProperty());
    this.orientation = hold(new C.SampledProperty(C.Quaternion));
    this.headingProp = hold(new C.SampledProperty(Number));
    this.pitchProp = hold(new C.SampledProperty(Number));
    this.rollProp = hold(new C.SampledProperty(Number));
    this._sensorCache = { time: null, value: null };
    /** The copter's latest physics state ({lat, lon, alt, roll, pitch, yaw}) and when it came
     * (performance.now()), while the camera feed is open in the sim. */
    this.physics = null;
    this.physicsAt = 0;

    const opts = fleet.options;
    const placed = () => this.base !== null && !fleet.lost.has(this.sysid);
    const visible = new C.CallbackProperty(() => placed() && fleet.fpv !== this.sysid, false);
    const isLeader = () => fleet.leader === this.sysid;
    const isSelected = () => fleet.isSelected(this.sysid);
    const entities = fleet.viewer.entities;

    const tint = (alpha = 1) =>
      new C.CallbackProperty(() => shade(fleet.droneCss(this.sysid), alpha), false);
    const tintMaterial = (alpha) => new C.ColorMaterialProperty(tint(alpha));

    // The 2525E symbol, marked AUT while the autopilot flies it (controlMode); plain otherwise.
    const symbol = this._symbol(false);
    const autonomous = this._symbol(true);
    // In the camera feed: where the physics has it, at its real size.
    const physical = () => (fleet.feed ? this._physical() : null);
    this.entity = entities.add({
      id: `vehicle-${sysid}`,
      name: callsign(sysid),
      position: new C.CallbackPositionProperty(
        (time, result) => physical()?.pos ?? this.position.getValue(time, result),
        false,
      ),
      orientation: new C.CallbackProperty((time, result) => {
        const p = physical();
        if (!p) return this.orientation.getValue(time, result);
        const hpr = new C.HeadingPitchRoll(p.yaw + MODEL_HEADING_OFFSET, p.pitch, p.roll);
        return C.Transforms.headingPitchRollQuaternion(p.pos, hpr, undefined, undefined, result);
      }, false),
      model: {
        uri: MODEL_URI,
        scale: new C.CallbackProperty(() => (fleet.feed ? 1 : MODEL_SCALE), false),
        minimumPixelSize: new C.CallbackProperty(() => (fleet.feed ? 0 : 44), false),
        maximumScale: 150,
        nodeTransformations: Object.fromEntries(
          Object.entries(PROPS).map(([node, turn]) => [
            node,
            new C.NodeTransformationProperty({
              rotation: new C.CallbackProperty(
                (time, result) =>
                  C.Quaternion.fromAxisAngle(C.Cartesian3.UNIT_Y, turn * this._propAngle(time), result),
                false,
              ),
            }),
          ]),
        ),
        silhouetteColor: new C.CallbackProperty(
          () => shade(isSelected() ? fleet.droneCss(this.sysid) : LEADER_CSS),
          false,
        ),
        silhouetteSize: new C.CallbackProperty(
          () => (fleet.feed ? 0 : isSelected() ? 3 : isLeader() ? 2 : 0),
          false,
        ),
        show: visible,
      },
      billboard: {
        image: new C.CallbackProperty(
          () => (controlMode(this.data) === "AUT" ? autonomous : symbol).url,
          false,
        ),
        width: symbol.width,
        height: symbol.height,
        // Put milsymbol's anchor (the centre of the frame) SYMBOL_LIFT_PX above the drone.
        horizontalOrigin: C.HorizontalOrigin.LEFT,
        verticalOrigin: C.VerticalOrigin.TOP,
        pixelOffset: new C.Cartesian2(-symbol.anchor.x, -symbol.anchor.y - SYMBOL_LIFT_PX),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: new C.CallbackProperty(
          () => opts.labels && placed() && !fleet.feed && fleet.fpv !== this.sysid,
          false,
        ),
      },
      label: {
        text: new C.CallbackProperty(() => this._labelText(), false),
        font: "600 12px Inter, Arial, sans-serif",
        fillColor: C.Color.WHITE,
        outlineColor: C.Color.BLACK,
        outlineWidth: 3,
        style: C.LabelStyle.FILL_AND_OUTLINE,
        showBackground: true,
        backgroundColor: C.Color.fromCssColorString("rgba(10,14,18,0.62)"),
        backgroundPadding: new C.Cartesian2(6, 4),
        horizontalOrigin: C.HorizontalOrigin.LEFT,
        verticalOrigin: C.VerticalOrigin.CENTER,
        pixelOffset: new C.Cartesian2(symbol.width - symbol.anchor.x + 6, -SYMBOL_LIFT_PX),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        distanceDisplayCondition: new C.DistanceDisplayCondition(0, LABEL_RANGE_M),
        show: new C.CallbackProperty(
          () => opts.labels && placed() && !fleet.feed && fleet.fpv !== this.sysid,
          false,
        ),
      },
    });

    this.trailEntity = entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._trailPositions(), false),
        width: 3,
        material: new C.PolylineGlowMaterialProperty({ color: tint(0.9), glowPower: 0.18 }),
        show: new C.CallbackProperty(() => opts.trails && placed(), false),
      },
    });

    this.stalk = entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._stalk(), false),
        width: 1.5,
        material: new C.PolylineDashMaterialProperty({ color: tint(0.8), dashLength: 8 }),
        show: new C.CallbackProperty(() => placed() && (this.data?.alt ?? 0) > 1, false),
      },
    });

    const slotShown = () => opts.slots && placed() && this.slotPosition() !== undefined;
    this.slot = entities.add({
      position: new C.CallbackPositionProperty(() => this.slotPosition(), false),
      point: {
        pixelSize: 11,
        color: C.Color.TRANSPARENT,
        outlineColor: tint(),
        outlineWidth: 2.5,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: new C.CallbackProperty(slotShown, false),
      },
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._slotLink(), false),
        width: 1.5,
        material: new C.PolylineDashMaterialProperty({ color: tint(0.9), dashLength: 6 }),
        show: new C.CallbackProperty(slotShown, false),
      },
    });

    // Formation geometry: a faint line from the leader to this follower's slot draws the ring.
    this.formationLink = entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._formationLink(), false),
        width: 1,
        material: C.Color.WHITE.withAlpha(0.35),
        show: new C.CallbackProperty(
          () => opts.slots && placed() && this.data?.role === "follower",
          false,
        ),
      },
    });

    // The camera's view: a line from the drone along each image corner, fading out over
    // SENSOR_RANGE_M, and the image frame across them. The FPV view sees between them.
    const sensorShown = new C.CallbackProperty(
      () =>
        (opts.sensors === "all" || (opts.sensors === "selected" && isSelected())) &&
        placed() &&
        Boolean(this.data?.armed) &&
        fleet.fpv !== this.sysid,
      false,
    );
    const fade = new FadeMaterialProperty(() => fleet.droneCss(this.sysid), 0.95);
    for (let i = 0; i < 4; i++) {
      entities.add({
        polyline: {
          arcType: C.ArcType.NONE,
          positions: new C.CallbackProperty(() => this._sensor()?.rays[i] ?? [], false),
          width: 1.5,
          material: fade,
          show: sensorShown,
        },
      });
    }
    entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._sensor()?.frame ?? [], false),
        width: 1.5,
        material: tintMaterial(0.95 * (1 - SENSOR_FRAME_AT)),
        show: sensorShown,
      },
    });
  }

  update(v, t) {
    this.data = v;
    this.placeBase(v);
    if (this.base === null) return;
    const when = later(DELAY_S);
    const pos = this._cartesian(v.lat, v.lon, v.alt);
    this.position.addSample(when, pos);
    this._heading = unwrap(v.heading, this._heading);
    this.headingProp.addSample(when, this._heading);
    this.pitchProp.addSample(when, v.pitch);
    this._roll = unwrap(v.roll, this._roll);
    this.rollProp.addSample(when, this._roll);
    const hpr = new C.HeadingPitchRoll(v.heading + MODEL_HEADING_OFFSET, v.pitch, v.roll);
    this.orientation.addSample(when, C.Transforms.headingPitchRollQuaternion(pos, hpr));
    this.addTrailPoint(v, t);
    const now = performance.now();
    if (now - this._lastPrune > KEEP_SAMPLES_S * 1000) {
      this._lastPrune = now;
      const old = olderThan(KEEP_SAMPLES_S);
      this.position.removeSamples(old);
      this.orientation.removeSamples(old);
      this.headingProp.removeSamples(old);
      this.pitchProp.removeSamples(old);
      this.rollProp.removeSamples(old);
    }
  }

  /** @param {number} t coordinator time of the frame, seconds */
  addTrailPoint(v, t) {
    const last = this.trail[this.trail.length - 1];
    if (last && t < last.t) this.trail.length = 0; // a replay restarted
    else if (last && t - last.t < TRAIL_STEP_S) return;
    if (!v.armed && v.alt < 0.5) return;
    if (last && last.lat === v.lat && last.lon === v.lon && last.alt === v.alt) return; // hovering still
    this.trail.push({ t, lat: v.lat, lon: v.lon, alt: v.alt });
    const keep = TRAIL_S / TRAIL_STEP_S;
    if (this.trail.length > keep) this.trail.splice(0, this.trail.length - keep);
  }

  currentPosition() {
    if (this.base === null) return undefined;
    return this.position.getValue(C.JulianDate.now());
  }

  /** Smoothed heading in radians, clockwise from north. */
  currentHeading() {
    return this.headingProp.getValue(C.JulianDate.now()) ?? this.data?.heading ?? 0;
  }

  /** Propeller angle at `time`, radians. The rate eases towards still (disarmed), slow (armed)
   * or a little faster (flying), so the props never jump or stall. */
  _propAngle(time) {
    const p = this._props;
    if (p.time && C.JulianDate.equals(p.time, time)) return p.angle;
    const dt = p.time ? Math.min(Math.max(C.JulianDate.secondsDifference(time, p.time), 0), 0.5) : 0;
    p.time = C.JulianDate.clone(time, p.time);
    const d = this.data;
    const target = !d?.armed ? 0 : d.alt > AIRBORNE_M ? PROP_TURNS_PER_S.flying : PROP_TURNS_PER_S.armed;
    p.rate += (target - p.rate) * (1 - Math.exp(-dt / PROP_SETTLE_S));
    p.angle = (p.angle + 2 * Math.PI * p.rate * dt) % (2 * Math.PI);
    return p.angle;
  }

  /** Smoothed pitch in radians, nose up. */
  currentPitch() {
    return this.pitchProp.getValue(C.JulianDate.now()) ?? this.data?.pitch ?? 0;
  }

  /** Smoothed roll in radians, right wing down. */
  currentRoll() {
    return this.rollProp.getValue(C.JulianDate.now()) ?? this.data?.roll ?? 0;
  }

  /** The FPV camera's axes (sensor.cameraAxes): fixed to the frame, tilted by the Tilt option. */
  cameraAxes() {
    const tilt = C.Math.toRadians(this.fleet.options.tiltDeg);
    return cameraAxes(this.currentHeading(), this.currentPitch(), this.currentRoll(), tilt);
  }

  /** The latest physics with its position, while it is fresh; null otherwise. */
  _physical() {
    const p = this.physics;
    if (!p || this.base === null || performance.now() - this.physicsAt >= PHYSICS_STALE_MS) return null;
    return { ...p, pos: this._cartesian(p.lat, p.lon, p.alt) };
  }

  /** Where the camera is and where it looks. In the camera feed (`feed`), from the drone's
   * physics as it last arrived; otherwise, or without physics, from the smoothed telemetry.
   * `physical` says which. */
  cameraPose(feed) {
    const p = feed ? this._physical() : null;
    if (p) {
      const tilt = C.Math.toRadians(this.fleet.options.tiltDeg);
      return { pos: p.pos, axes: cameraAxes(p.yaw, p.pitch, p.roll, tilt), physical: true };
    }
    const pos = this.currentPosition();
    return pos && { pos, axes: this.cameraAxes(), physical: false };
  }

  /** The latest frame's position and model rotation, Earth-fixed, not smoothed or delayed;
   * undefined until the drone is placed. */
  framePose() {
    const v = this.data;
    if (!v || this.base === null) return undefined;
    const pos = this._cartesian(v.lat, v.lon, v.alt);
    const hpr = new C.HeadingPitchRoll(v.heading + MODEL_HEADING_OFFSET, v.pitch, v.roll);
    return { pos, quat: C.Transforms.headingPitchRollQuaternion(pos, hpr) };
  }

  /** Height above home of the smoothed position. */
  currentAlt() {
    const p = this.currentPosition();
    if (!p || this.base === null) return this.data?.alt ?? 0;
    return C.Cartographic.fromCartesian(p).height - this.base - MODEL_LIFT_M;
  }

  slotPosition() {
    const s = this.data?.slot;
    if (!s || this.base === null) return undefined;
    return this._cartesian(s[0], s[1], this.data.alt);
  }

  _cartesian(lat, lon, alt) {
    return C.Cartesian3.fromDegrees(lon, lat, this.base + Math.max(alt, 0) + MODEL_LIFT_M);
  }

  async placeBase(v) {
    if (this._basePending || this.base !== null) return;
    this._basePending = true;
    const fleet = this.fleet;
    this.base =
      v.alt < AIRBORNE_M ? await fleet.map.groundHeight(v.lat, v.lon) : await fleet.originBase;
  }

  /**
   * The vehicle's 2525 symbol as an image, with its size and anchor in CSS pixels.
   * @param {boolean} autonomous
   */
  _symbol(autonomous) {
    const symbol = new ms.Symbol(sidcUav(autonomous), {
      size: 36,
      uniqueDesignation: callsign(this.sysid),
      infoColor: "#f2f5f7",
      infoOutlineColor: "rgba(0,0,0,0.9)",
      infoOutlineWidth: 4,
      outlineColor: "rgba(0,0,0,0.55)",
      outlineWidth: 3,
      fontfamily: "Arial",
    });
    const size = symbol.getSize();
    return {
      url: symbol.asCanvas(window.devicePixelRatio || 1).toDataURL(),
      width: size.width,
      height: size.height,
      anchor: symbol.getAnchor(),
    };
  }

  _labelText() {
    const v = this.data;
    // Full text for the selected vehicle, the leader and a pilot's drone only; the 2525 symbol
    // carries the callsign for the rest, so a tight formation stays readable.
    const shown =
      this.fleet.selected === this.sysid || this.fleet.leader === this.sysid || v?.role === "pilot";
    if (!v || !shown) return "";
    const role = v.armed ? v.role.toUpperCase() : "ON GROUND";
    return `${role} · ${v.mode}\n${Math.round(v.alt)} m AGL`;
  }

  _trailPositions() {
    if (this.base === null) return [];
    const out = [];
    for (const p of this.trail) {
      p.cart ??= this._cartesian(p.lat, p.lon, p.alt);
      out.push(p.cart);
    }
    const now = this.currentPosition();
    if (now) out.push(now);
    return out;
  }

  _stalk() {
    const p = this.currentPosition();
    if (!p) return [];
    const carto = C.Cartographic.fromCartesian(p);
    return [C.Cartesian3.fromRadians(carto.longitude, carto.latitude, this.base), p];
  }

  _slotLink() {
    const p = this.currentPosition();
    const s = this.slotPosition();
    return p && s ? [p, s] : [];
  }

  _formationLink() {
    const leader = this.fleet.vehicles.get(this.fleet.leader);
    const from = leader?.currentPosition();
    const to = this.slotPosition();
    return from && to ? [from, to] : [];
  }

  /** The camera's view lines and image frame for this frame, shared by their polylines. Same
   * camera as the FPV view: its position, axes and field of view. */
  _sensor() {
    const now = C.JulianDate.now();
    const cache = this._sensorCache;
    if (cache.time && C.JulianDate.equalsEpsilon(cache.time, now, 0.004)) return cache.value;
    cache.time = now;
    cache.value = null;
    const apex = this.currentPosition();
    if (!apex) return null;
    const opts = this.fleet.options;
    const hfov = C.Math.toRadians(opts.hfovDeg);
    const vfov = C.Math.toRadians(opts.vfovDeg);
    const axes = this.cameraAxes();
    const far = viewCorners(apex, axes, hfov, vfov, SENSOR_RANGE_M);
    const near = viewCorners(apex, axes, hfov, vfov, SENSOR_RANGE_M * SENSOR_FRAME_AT);
    cache.value = { rays: far.map((c) => [apex, c]), frame: [...near, near[0]] };
    return cache.value;
  }
}

class TargetView {
  constructor(fleet) {
    this.fleet = fleet;
    this.latest = null;
    this.height = null;
    this._heightAt = null;
    this._pending = false;
    this.trail = [];
    this.position = hold(new C.SampledPositionProperty());
    /** B on the ground, where the truck stands. */
    this.groundPosition = hold(new C.SampledPositionProperty());
    /** sysid -> whether that drone is on the truck, as of its last frame. */
    this._onTruck = new Map();
    /** Landing radius disc, created once B has a position. */
    this.ring = null;
    this._ringUpdated = 0;
    const entities = fleet.viewer.entities;
    const placed = () => this.height !== null && this.latest !== null;
    const color = C.Color.fromCssColorString(TARGET_CSS);
    this.entity = entities.add({
      id: "target-B",
      name: "POINT B",
      position: this.position,
      billboard: {
        image: pointMarker("B", TARGET_CSS).toDataURL(),
        verticalOrigin: C.VerticalOrigin.CENTER,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: new C.CallbackProperty(placed, false),
      },
      label: {
        text: "POINT B",
        font: "600 12px Inter, Arial, sans-serif",
        fillColor: color,
        outlineColor: C.Color.BLACK,
        outlineWidth: 3,
        style: C.LabelStyle.FILL_AND_OUTLINE,
        pixelOffset: new C.Cartesian2(0, 26),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: new C.CallbackProperty(placed, false),
      },
    });
    const truckHeading = new C.HeadingPitchRoll(
      C.Math.toRadians(TRUCK_HEADING_DEG) + MODEL_HEADING_OFFSET,
      0,
      0,
    );
    this.truck = entities.add({
      id: "target-B-truck",
      name: "POINT B",
      position: this.groundPosition,
      orientation: new C.CallbackProperty((time, result) => {
        const p = this.groundPosition.getValue(time);
        return p && C.Transforms.headingPitchRollQuaternion(p, truckHeading, undefined, undefined, result);
      }, false),
      model: {
        uri: TRUCK_URI,
        minimumPixelSize: 32,
        maximumScale: 20,
        show: new C.CallbackProperty(() => placed() && !fleet.duelTruck, false),
      },
    });
    const radius = fleet.config.landRadiusM ?? 25;
    this.radius = radius;
    this.color = color;
    this.ringOutline = entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._circle(radius), false),
        width: 2,
        material: color.withAlpha(0.85),
        depthFailMaterial: color.withAlpha(0.4),
        show: new C.CallbackProperty(placed, false),
      },
    });
    this.trailEntity = entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => this._trail(), false),
        width: 2,
        material: new C.PolylineDashMaterialProperty({ color: color.withAlpha(0.8), dashLength: 10 }),
        depthFailMaterial: color.withAlpha(0.3),
        show: new C.CallbackProperty(() => placed() && fleet.options.trails, false),
      },
    });
  }

  update(target, t) {
    this.latest = target;
    if (!target) return;
    this._refreshHeight(target);
    if (this.height === null) return;
    const pos = C.Cartesian3.fromDegrees(target[1], target[0], this.height + 0.5);
    this.position.addSample(later(DELAY_S), pos);
    const ground = C.Cartesian3.fromDegrees(target[1], target[0], this.height);
    this.groundPosition.addSample(later(DELAY_S), ground);
    this.addTrailPoint(target, t);
    const now = performance.now();
    if (now - this._ringUpdated > 1000) {
      this._ringUpdated = now;
      this._placeRing(pos);
      const old = olderThan(KEEP_SAMPLES_S);
      this.position.removeSamples(old);
      this.groundPosition.removeSamples(old);
    }
  }

  /** Puff smoke where a drone comes onto the truck. A drone already on it when the page opens
   * has not just struck it. */
  checkHits(vehicles) {
    if (!this.latest || this.height === null || this.fleet.duelTruck) return;
    for (const view of vehicles) {
      const v = view.data;
      if (!v || view.base === null) continue;
      const was = this._onTruck.get(view.sysid);
      const on = this._onTruckBox(v, view.base, was ? HIT_MARGIN_M + HIT_RELEASE_M : HIT_MARGIN_M);
      this._onTruck.set(view.sysid, on);
      if (on && was === false) {
        const at = C.Cartesian3.fromDegrees(v.lon, v.lat, view.base + Math.max(v.alt, 0) + MODEL_LIFT_M);
        // When the drawn drone, DELAY_S behind the frames, gets there.
        setTimeout(() => poof(this.fleet.viewer.scene, at), DELAY_S * 1000);
      }
    }
  }

  /** Whether frame vehicle `v`, its home's ground at `base`, is within `pad` of the truck. */
  _onTruckBox(v, base, pad) {
    const [lat, lon] = this.latest;
    const north = (v.lat - lat) * M_PER_DEG_LAT;
    const east = (v.lon - lon) * M_PER_DEG_LAT * Math.cos(C.Math.toRadians(lat));
    const heading = C.Math.toRadians(TRUCK_HEADING_DEG);
    const ahead = north * Math.cos(heading) + east * Math.sin(heading);
    const across = east * Math.cos(heading) - north * Math.sin(heading);
    return (
      Math.abs(ahead) <= TRUCK_SIZE_M.length / 2 + pad &&
      Math.abs(across) <= TRUCK_SIZE_M.width / 2 + pad &&
      base + v.alt - this.height <= TRUCK_SIZE_M.height + pad
    );
  }

  /** The filled disc is ground-clamped geometry, so it moves once a second, not every frame. */
  _placeRing(pos) {
    if (this.ring) {
      this.ring.position.setValue(pos);
      return;
    }
    this.ring = this.fleet.viewer.entities.add({
      position: new C.ConstantPositionProperty(pos),
      ellipse: {
        semiMajorAxis: this.radius,
        semiMinorAxis: this.radius,
        material: this.color.withAlpha(0.14),
        classificationType: this.fleet.map.classification,
        show: new C.CallbackProperty(() => this.latest !== null, false),
      },
    });
  }

  addTrailPoint(target, t) {
    const last = this.trail[this.trail.length - 1];
    if (last && t < last.t) this.trail.length = 0;
    else if (last && t - last.t < 1.0) return;
    this.trail.push({ t, lat: target[0], lon: target[1] });
    if (this.trail.length > 600) this.trail.shift();
  }

  currentPosition() {
    if (this.height === null || this.latest === null) return undefined;
    return this.position.getValue(C.JulianDate.now());
  }

  /** B drives over the ground: resample the ground height when it has moved a few metres. */
  async _refreshHeight(target) {
    if (this._pending) return;
    const here = C.Cartesian3.fromDegrees(target[1], target[0]);
    if (this._heightAt && C.Cartesian3.distance(here, this._heightAt) < 3) return;
    this._pending = true;
    this.height = await this.fleet.map.groundHeight(target[0], target[1]);
    this._heightAt = here;
    this._pending = false;
  }

  _trail() {
    if (this.height === null) return [];
    const pts = this.trail.map((p) => C.Cartesian3.fromDegrees(p.lon, p.lat, this.height + 0.4));
    const now = this.currentPosition();
    if (now) pts.push(now);
    return pts;
  }

  _circle(radius) {
    const center = this.currentPosition();
    if (!center) return [];
    const enu = C.Transforms.eastNorthUpToFixedFrame(center);
    const pts = [];
    for (let i = 0; i <= 64; i++) {
      const a = (i / 64) * 2 * Math.PI;
      const local = new C.Cartesian3(radius * Math.sin(a), radius * Math.cos(a), -0.1);
      pts.push(C.Matrix4.multiplyByPoint(enu, local, new C.Cartesian3()));
    }
    return pts;
  }
}
