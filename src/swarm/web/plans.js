// Each drone's uploaded plan, as the coordinator read it off the vehicle (swarm.mission):
// the mission path and its waypoints, home, ROI, the geofence and rally points.
//
// The look follows ATAK routes: a solid line in the drone's colour with a black under-stroke,
// route-coloured waypoint discs, and points named SP, CP1, CP2... with white labels on a dark
// box. From QGroundControl: the item letter in the disc (T takeoff, L land, H home), green for
// the current item, and orange geofences. Flown legs fade, which neither app does.
//
// Heights come in above home (or above terrain for terrain-frame items) and are placed on
// the same ground height as the drone itself (VehicleView.base), so a drone flying its
// mission sits on the drawn path.

import { planMarker } from "./symbols.js";

const C = Cesium;

/** Commands, as in MAVLink's MAV_CMD. */
const CMD = {
  WAYPOINT: 16,
  LOITER_UNLIM: 17,
  LOITER_TURNS: 18,
  LOITER_TIME: 19,
  RTL: 20,
  LAND: 21,
  TAKEOFF: 22,
  LOITER_TO_ALT: 31,
  SPLINE: 82,
  ROI_LOCATION: 195,
  ROI: 201,
};
const LOITERS = new Set([CMD.LOITER_UNLIM, CMD.LOITER_TURNS, CMD.LOITER_TIME, CMD.LOITER_TO_ALT]);
/** MISSION_STATE from MISSION_CURRENT. */
const MISSION_ACTIVE = 3;
const MISSION_COMPLETE = 5;
/** FENCE_TYPE bits. */
const FENCE_ALT_MAX = 1;
const FENCE_CIRCLE = 2;

/** QGroundControl's geofence orange and breach-return green. */
const FENCE_CSS = "#ffa500";
const BREACH_RETURN_CSS = "#14a63c";
const RALLY_CSS = "#c58cff";
const ROI_CSS = "#ffd84d";
const LABEL_FONT = "600 11px Inter, Arial, sans-serif";
/** Plan labels hide beyond this camera distance; the markers stay. */
const LABEL_RANGE_M = 900;
const FENCE_LABEL_RANGE_M = 4000;
const GROUND_LIFT_M = 0.3;

/**
 * Disc text and ATAK-style point name per nav item. Checkpoints are numbered in order.
 * @returns {{disc: string, name: string, kind: string}}
 */
function naming(item, checkpoint) {
  switch (item.cmd) {
    case CMD.TAKEOFF:
      return { disc: "T", name: "SP", kind: "action" };
    case CMD.LAND:
      return { disc: "L", name: "LZ", kind: "action" };
    default:
      return {
        disc: String(checkpoint),
        name: LOITERS.has(item.cmd) ? `CP${checkpoint} LOITER` : `CP${checkpoint}`,
        kind: LOITERS.has(item.cmd) ? "loiter" : "waypoint",
      };
  }
}

/** An ATAK map label: white text on a 60% black box, below the point. */
const label = (text, pixelY, range = LABEL_RANGE_M) => ({
  text,
  font: LABEL_FONT,
  fillColor: C.Color.WHITE,
  showBackground: true,
  backgroundColor: C.Color.BLACK.withAlpha(0.6),
  backgroundPadding: new C.Cartesian2(4, 2),
  verticalOrigin: pixelY < 0 ? C.VerticalOrigin.BOTTOM : C.VerticalOrigin.TOP,
  pixelOffset: new C.Cartesian2(0, pixelY),
  disableDepthTestDistance: Number.POSITIVE_INFINITY,
  distanceDisplayCondition: new C.DistanceDisplayCondition(0, range),
});

export class PlanLayer {
  /**
   * @param {import("./fleet.js").Fleet} fleet
   * @param {object} options display options shared with the UI (`mission`, `fence`)
   */
  constructor(fleet, options) {
    this.fleet = fleet;
    this.options = options;
    /** @type {Map<number, PlanView>} */
    this.views = new Map();
  }

  /** A plan arrived (on change, and re-sent every few seconds). */
  set(plan) {
    let view = this.views.get(plan.sysid);
    if (!view) {
      view = new PlanView(this, plan.sysid);
      this.views.set(plan.sysid, view);
    }
    return view.set(plan);
  }

  /** Per telemetry frame: build plans whose drone is now placed, follow the current item. */
  update() {
    for (const view of this.views.values()) view.update();
  }

  /** Waypoint positions, for framing the camera. */
  allPositions() {
    return [...this.views.values()].flatMap((v) => v.positions);
  }
}

class PlanView {
  constructor(layer, sysid) {
    this.layer = layer;
    this.sysid = sysid;
    this.fleet = layer.fleet;
    // Drawn in the drone's colour, rebuilt when it changes.
    this.css = this.fleet.droneCss(sysid);
    this.color = C.Color.fromCssColorString(this.css);
    this.plan = null;
    this._key = "";
    this._buildFor = null;
    this._building = false;
    /** Cartesian positions of the nav items, by seq. */
    this.points = new Map();
    this.positions = [];
    this.mission = new C.CustomDataSource(`mission-${sysid}`);
    this.fence = new C.CustomDataSource(`fence-${sysid}`);
    this.fleet.viewer.dataSources.add(this.mission);
    this.fleet.viewer.dataSources.add(this.fence);
  }

  /** @returns {boolean} true when the plan differs from the one shown */
  set(plan) {
    const { t, ...content } = plan;
    const key = JSON.stringify(content);
    if (key === this._key) return false;
    this._key = key;
    this.plan = plan;
    this._buildFor = null; // rebuild on the next update
    return true;
  }

  get vehicle() {
    return this.fleet.vehicles.get(this.sysid);
  }

  update() {
    const opts = this.layer.options;
    // A camera does not see plans (Fleet.feed), nor does the duel's driver (Fleet.driving).
    const map = !this.fleet.feed && !this.fleet.driving;
    this.mission.show = map && Boolean(opts.mission);
    // Every drone carries its own fence around its own home; with drones metres apart they
    // overlap, so the fence is drawn for the selected drone only.
    this.fence.show = map && Boolean(opts.fence) && this.fleet.selected === this.sysid;
    const css = this.fleet.droneCss(this.sysid);
    if (css !== this.css) {
      this.css = css;
      this.color = C.Color.fromCssColorString(css);
      this._buildFor = null;
    }
    const base = this.vehicle?.base;
    if (this.plan && base !== null && base !== undefined && this._buildFor !== this._key) {
      this._build(base);
    }
  }

  /** Where the drone is in its mission: legs before the current item are flown. */
  _legState(seq) {
    const v = this.vehicle?.data;
    if (!v) return "next";
    if (v.mission_state === MISSION_COMPLETE) return "done";
    const flying = v.mode === "AUTO" || v.mission_state === MISSION_ACTIVE;
    if (!flying || v.wp === null || v.wp === undefined) return "next";
    if (seq < v.wp) return "done";
    return seq === v.wp ? "active" : "next";
  }

  async _build(base) {
    if (this._building) return;
    this._building = true;
    const key = this._key;
    const plan = this.plan;
    try {
      const heightOf = async (lat, lon, alt, terrain) =>
        (terrain ? await this.fleet.map.groundHeight(lat, lon) : base) + Math.max(alt, 0);
      const nav = plan.items.filter((i) => i.nav && i.lat !== null);
      const cart = await Promise.all(
        nav.map(async (i) => C.Cartesian3.fromDegrees(i.lon, i.lat, await heightOf(i.lat, i.lon, i.alt, i.terrain))),
      );
      if (key !== this._key) return; // a newer plan arrived meanwhile
      this.mission.entities.removeAll();
      this.fence.entities.removeAll();
      this.points = new Map(nav.map((i, k) => [i.seq, cart[k]]));
      this.positions = cart;
      this._drawMission(plan, nav, cart, base);
      this._drawFence(plan, base);
      this._buildFor = key;
    } finally {
      this._building = false;
    }
  }

  _drawMission(plan, nav, cart, base) {
    const entities = this.mission.entities;
    const home = plan.home ? C.Cartesian3.fromDegrees(plan.home[1], plan.home[0], base + GROUND_LIFT_M) : null;
    const ground = (p) => {
      const c = C.Cartographic.fromCartesian(p);
      return C.Cartesian3.fromRadians(c.longitude, c.latitude, base + GROUND_LIFT_M);
    };

    if (home) {
      // A takeoff from home labels the spot already (SP); drones start metres apart, so
      // fewer labels there keeps it readable.
      const startsHere = nav[0]?.cmd === CMD.TAKEOFF;
      entities.add({
        position: home,
        billboard: this._billboard(planMarker("H", this.css, "home")),
        label: startsHere ? undefined : label("HOME", 15),
      });
    }

    // Legs: from home (or the previous nav item) to each nav item, styled by progress.
    // RTL climbs to RTL_ALT_M first if lower, flies home at that height, then lands.
    let from = home;
    let fromAlt = 0;
    const groundTrack = home ? [home] : [];
    nav.forEach((item, k) => {
      const to = cart[k];
      if (from && item.cmd === CMD.RTL) {
        const climb = Math.max(fromAlt, plan.rtl_alt ?? 0);
        const up = (p) => {
          const c = C.Cartographic.fromCartesian(p);
          return C.Cartesian3.fromRadians(c.longitude, c.latitude, base + climb);
        };
        this._leg(entities, item.seq, [from, up(from), up(to), to]);
      } else if (from) {
        this._leg(entities, item.seq, [from, to]);
      }
      from = to;
      fromAlt = item.alt ?? 0;
      groundTrack.push(ground(to));
    });
    if (groundTrack.length > 1) {
      entities.add({
        polyline: {
          arcType: C.ArcType.NONE,
          positions: groundTrack,
          width: 1.5,
          material: new C.PolylineDashMaterialProperty({ color: this.color.withAlpha(0.45), dashLength: 10 }),
        },
      });
    }

    // Markers, with a drop line to the ground under each so heights read in 3D. RTL has no
    // marker of its own (as in QGC): its leg ends at home.
    let checkpoint = 0;
    nav.forEach((item, k) => {
      if (item.cmd === CMD.RTL) return;
      if (item.cmd !== CMD.TAKEOFF && item.cmd !== CMD.LAND) checkpoint += 1;
      const p = cart[k];
      const at = ground(p);
      const { disc, name, kind } = naming(item, checkpoint);
      const state = () => this._legState(item.seq);
      const height = Math.round(item.alt ?? 0);
      const sameSpot = C.Cartesian3.distance(p, at) < 1;
      if (!sameSpot) {
        entities.add({
          polyline: {
            arcType: C.ArcType.NONE,
            positions: [at, p],
            width: 1,
            material: new C.PolylineDashMaterialProperty({ color: this.color.withAlpha(0.6), dashLength: 6 }),
          },
        });
      }
      const images = {
        next: planMarker(disc, this.css, kind),
        active: planMarker(disc, this.css, kind, "active"),
        done: planMarker(disc, this.css, kind, "done"),
      };
      const heightText = item.cmd === CMD.LAND ? "" : ` · ${height} m${item.terrain ? " AGL" : ""}`;
      entities.add({
        position: p,
        billboard: {
          ...this._billboard(images.next),
          image: new C.CallbackProperty(() => images[state()].url, false),
          width: new C.CallbackProperty(() => images[state()].size, false),
          height: new C.CallbackProperty(() => images[state()].size, false),
        },
        label: label(`${name}${heightText}`, 15),
      });
    });

    // From the drone to the item it is flying to, while the mission runs.
    const vehicle = this.vehicle;
    entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => {
          const wp = vehicle?.data?.wp;
          const target = wp !== undefined && wp !== null ? this.points.get(wp) : undefined;
          const here = vehicle?.currentPosition();
          return target && here && this._legState(wp) === "active" ? [here, target] : [];
        }, false),
        width: 10,
        material: new C.PolylineArrowMaterialProperty(C.Color.fromCssColorString("#ffffff").withAlpha(0.9)),
      },
    });

    for (const item of plan.items) {
      if ((item.cmd === CMD.ROI || item.cmd === CMD.ROI_LOCATION) && item.lat !== null) {
        entities.add({
          position: C.Cartesian3.fromDegrees(item.lon, item.lat, base + GROUND_LIFT_M),
          billboard: this._billboard(planMarker("", ROI_CSS, "roi")),
          label: label("ROI", 15),
        });
      }
    }
    for (const [lat, lon, alt] of plan.rally ?? []) {
      entities.add({
        position: C.Cartesian3.fromDegrees(lon, lat, base + Math.max(alt, 0)),
        billboard: this._billboard(planMarker("R", RALLY_CSS, "rally")),
        label: label(`RALLY · ${Math.round(alt)} m`, 15),
      });
    }
  }

  _leg(entities, seq, positions) {
    const width = new C.CallbackProperty(() => (this._legState(seq) === "active" ? 4 : 3), false);
    const color = new C.CallbackProperty(() => {
      const state = this._legState(seq);
      return state === "done" ? this.color.withAlpha(0.35) : this.color.withAlpha(state === "active" ? 1 : 0.85);
    }, false);
    entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions,
        width,
        material: new C.PolylineOutlineMaterialProperty({
          color,
          outlineColor: C.Color.BLACK.withAlpha(0.6),
          outlineWidth: 1,
        }),
        depthFailMaterial: new C.ColorMaterialProperty(this.color.withAlpha(0.25)),
      },
    });
  }

  _drawFence(plan, base) {
    const entities = this.fence.entities;
    const fp = plan.fence_params;
    const enabled = Boolean(fp?.enabled);
    const ceiling = fp && fp.types & FENCE_ALT_MAX && fp.alt_max ? fp.alt_max : null;
    const top = base + (ceiling ?? 0);
    const state = enabled ? "ON" : "OFF";
    // As QGC: an inclusion fence is an outline (here a faint wall up to the ceiling), an
    // exclusion fence is filled at 20%. Dashed and dimmer while the fence is off.
    const wall = (css, inclusion) => {
      const c = C.Color.fromCssColorString(css);
      return {
        material: c.withAlpha((inclusion ? 0.06 : 0.2) * (enabled ? 1 : 0.5)),
        outline: true,
        outlineColor: c.withAlpha(enabled ? 0.9 : 0.45),
        height: base,
        extrudedHeight: ceiling ? top : undefined,
        numberOfVerticalLines: 0,
      };
    };
    const ring = (positions, css) => {
      const c = C.Color.fromCssColorString(css);
      entities.add({
        polyline: {
          arcType: C.ArcType.NONE,
          positions,
          width: 2,
          material: enabled
            ? new C.ColorMaterialProperty(c.withAlpha(0.9))
            : new C.PolylineDashMaterialProperty({ color: c.withAlpha(0.6), dashLength: 14 }),
        },
      });
    };

    if (fp && fp.types & FENCE_CIRCLE && fp.radius && plan.home) {
      const [lat, lon] = plan.home;
      const centre = C.Cartesian3.fromDegrees(lon, lat, base);
      entities.add({ position: centre, ellipse: { semiMajorAxis: fp.radius, semiMinorAxis: fp.radius, ...wall(FENCE_CSS, true) } });
      ring(circlePoints(lat, lon, fp.radius, base + GROUND_LIFT_M), FENCE_CSS);
      if (ceiling) ring(circlePoints(lat, lon, fp.radius, top), FENCE_CSS);
      const edge = C.Cartesian3.fromDegrees(...offsetDeg(lat, lon, fp.radius, 0).reverse(), (ceiling ? top : base) + 2);
      entities.add({
        position: edge,
        label: label(
          `FENCE ${state} · R ${Math.round(fp.radius)} m${ceiling ? ` · MAX ${Math.round(ceiling)} m` : ""}`,
          -12,
          FENCE_LABEL_RANGE_M,
        ),
      });
    }
    for (const shape of plan.fence ?? []) {
      const css = FENCE_CSS;
      if (shape.kind === "circle" && shape.radius) {
        const [lat, lon] = shape.points[0];
        entities.add({
          position: C.Cartesian3.fromDegrees(lon, lat, base),
          ellipse: { semiMajorAxis: shape.radius, semiMinorAxis: shape.radius, ...wall(css, shape.inclusion) },
        });
        ring(circlePoints(lat, lon, shape.radius, base + GROUND_LIFT_M), css);
      } else if (shape.kind === "polygon" && shape.points.length >= 3) {
        const flat = shape.points.flatMap(([lat, lon]) => [lon, lat]);
        entities.add({
          polygon: {
            hierarchy: C.Cartesian3.fromDegreesArray(flat),
            perPositionHeight: false,
            ...wall(css, shape.inclusion),
          },
        });
        const closed = [...shape.points, shape.points[0]].map(([lat, lon]) =>
          C.Cartesian3.fromDegrees(lon, lat, base + GROUND_LIFT_M),
        );
        ring(closed, css);
      }
    }
    if (plan.fence_return) {
      const [lat, lon] = plan.fence_return;
      entities.add({
        position: C.Cartesian3.fromDegrees(lon, lat, base + GROUND_LIFT_M),
        billboard: this._billboard(planMarker("B", BREACH_RETURN_CSS, "rally")),
        label: label("BREACH RETURN", 15),
      });
    }
  }

  _billboard(marker) {
    return {
      image: marker.url,
      width: marker.size,
      height: marker.size,
      verticalOrigin: C.VerticalOrigin.CENTER,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    };
  }
}

/** [lat, lon] a distance and true bearing away. Flat earth, fine for a fence. */
function offsetDeg(lat, lon, distance, bearingRad) {
  const r = 6378137;
  const dLat = (distance * Math.cos(bearingRad)) / r;
  const dLon = (distance * Math.sin(bearingRad)) / (r * Math.cos(C.Math.toRadians(lat)));
  return [lat + C.Math.toDegrees(dLat), lon + C.Math.toDegrees(dLon)];
}

function circlePoints(lat, lon, radius, height, steps = 72) {
  const out = [];
  for (let k = 0; k <= steps; k++) {
    const [a, b] = offsetDeg(lat, lon, radius, (2 * Math.PI * k) / steps);
    out.push(C.Cartesian3.fromDegrees(b, a, height));
  }
  return out;
}
