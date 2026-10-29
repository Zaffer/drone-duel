// The Map/COP view's drawing (DESIGN.md §11): the vehicle as a ground station sees it, and its
// uploaded plan. Everything here comes from telemetry and the plan channel, never from the true
// pose (DESIGN.md §2.2), so the same layer can one day watch a real aircraft.
//
// The vehicle: its MIL-STD-2525E symbol (milsymbol) a little above it, marked AUT while the
// autopilot flies it; its mode and height beside it; a trail of where telemetry has put it; and
// a dashed stalk down to its home's ground, so heights read in 3D. Positions come at 10 Hz and
// are eased from one to the next, a sample behind.
//
// The plan, as ATAK draws routes and QGroundControl its items: a solid route in the pilot's
// colour with a dark under-stroke, waypoint discs with QGC's item text (number, T takeoff, L
// land, H home), a dashed ground track and a drop line under each point; the geofence in the
// caution colour, inclusion polygons as faint walls, exclusion ones filled; rally points R and
// the breach return B. Heights: AMSL as telemetry gives them, plus geo.mapOffset; items above
// home take home's height from telemetry (alt_amsl - alt_rel).

import { effect } from "alien-signals";
import * as C from "cesium";
import ms from "milsymbol";
import type { PlanItem, PlanMsg, TelemetryMsg } from "../net/protocol.ts";
import { prefs } from "../state/prefs.ts";
import { plan, status, telemetry } from "../state/session.ts";
import { mapOffset } from "../world/geo.ts";
import type { World } from "../world/world.ts";

/** MAV_CMD, as MAVLink numbers them. */
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
  FENCE_RETURN: 5000,
  FENCE_POLY_IN: 5001,
  FENCE_POLY_OUT: 5002,
  FENCE_CIRCLE_IN: 5003,
  FENCE_CIRCLE_OUT: 5004,
} as const;
const NAV = new Set<number>([
  CMD.WAYPOINT,
  CMD.LOITER_UNLIM,
  CMD.LOITER_TURNS,
  CMD.LOITER_TIME,
  CMD.RTL,
  CMD.LAND,
  CMD.TAKEOFF,
  CMD.LOITER_TO_ALT,
  CMD.SPLINE,
]);
const LOITERS = new Set<number>([
  CMD.LOITER_UNLIM,
  CMD.LOITER_TURNS,
  CMD.LOITER_TIME,
  CMD.LOITER_TO_ALT,
]);
/** MAV_FRAME: heights above sea level, and above the terrain (the rest: above home). */
const FRAME_AMSL = new Set([0, 5]);
const FRAME_TERRAIN = new Set([10, 11]);
/** Modes in which the autopilot, not the pilot, flies it (ArduCopter's and INAV's names). */
const AUTONOMOUS = new Set([
  "AUTO",
  "GUIDED",
  "RTL",
  "SMART_RTL",
  "AUTO_RTL",
  "LAND",
  "CIRCLE",
  "FOLLOW",
  "BRAKE",
  "AVOID_ADSB",
  "GUIDED_NOGPS",
  "RTH",
  "NAV RTH",
  "WP",
  "NAV WP",
  "MISSION",
  "LAUNCH",
]);

/** The trail: a point every so often, s, kept this long, s; a jump further than this, m, is a
 * respawn. */
const TRAIL = { everyS: 0.5, seconds: 120, jumpM: 300 };
const SYMBOL_LIFT_PX = 44;
const SYMBOL_SIZE = 34;
const LABEL_RANGE_M = 2500;
const PLAN_LABEL_RANGE_M = 1200;
const GROUND_LIFT_M = 0.3;
/** Above this height above home the stalk shows. */
const STALK_FROM_M = 1;

interface Palette {
  ember: string;
  tide: string;
  sun: string;
  sky: string;
  text: string;
  muted: string;
  bg: string;
}

function palette(): Palette {
  const css = getComputedStyle(document.documentElement);
  const v = (name: string, fallback: string): string =>
    css.getPropertyValue(name).trim() || fallback;
  return {
    ember: v("--ember", "#ff6b35"),
    tide: v("--tide", "#22c7b8"),
    sun: v("--sun", "#ffc845"),
    sky: v("--sky", "#6ea8ff"),
    text: v("--text", "#eef0f7"),
    muted: v("--muted", "#8e93ad"),
    bg: v("--bg", "#0d0e15"),
  };
}

/**
 * MIL-STD-2525E code for a friendly military unmanned aircraft (CoT a-f-A-M-F-Q, as in ATAK),
 * marked AUT (common sector 2 modifier 110, autonomous control) when `autonomous`.
 */
export function sidcUav(autonomous: boolean): string {
  return [
    "13", // version: 2525E
    "0", // context: reality
    "3", // standard identity: friend
    "01", // symbol set: air
    "0", // status: present
    "0", // headquarters, task force, dummy: none
    "00", // amplifier: none
    "110300", // entity: military, unmanned aircraft
    "00", // sector 1 modifier: none
    autonomous ? "10" : "00", // sector 2 modifier
    "0", // sector 1 modifier set: the symbol set's own
    autonomous ? "1" : "0", // sector 2 modifier set: common
    "0", // frame shape: the standard identity's
    "0000000",
  ].join("");
}

/** Whether the autopilot flies it in `mode`. */
export const isAutonomous = (mode: string): boolean => AUTONOMOUS.has(mode.toUpperCase());

interface SymbolImage {
  url: string;
  width: number;
  height: number;
  anchor: { x: number; y: number };
}

function symbolImage(autonomous: boolean, name: string): SymbolImage {
  const symbol = new ms.Symbol(sidcUav(autonomous), {
    size: SYMBOL_SIZE,
    uniqueDesignation: name,
    infoColor: "#eef0f7",
    infoOutlineColor: "rgba(0,0,0,0.9)",
    infoOutlineWidth: 4,
    outlineColor: "rgba(0,0,0,0.55)",
    outlineWidth: 3,
    fontfamily: "Inter, Arial, sans-serif",
  });
  const size = symbol.getSize();
  return {
    url: symbol.asCanvas(window.devicePixelRatio || 1).toDataURL(),
    width: size.width,
    height: size.height,
    anchor: symbol.getAnchor(),
  };
}

type MarkerKind = "waypoint" | "action" | "loiter" | "home" | "rally" | "roi";

/** A plan point's marker: an ATAK route disc in `color` with QGC's item text inside. */
function planMarker(
  text: string,
  color: string,
  kind: MarkerKind,
  bg: string,
): { url: string; size: number } {
  const ratio = window.devicePixelRatio || 1;
  const size = kind === "home" ? 22 : 20;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size * ratio;
  const ctx = canvas.getContext("2d")!;
  ctx.scale(ratio, ratio);
  const c = size / 2;
  ctx.beginPath();
  if (kind === "roi") {
    ctx.moveTo(c, 2);
    ctx.lineTo(size - 2, c);
    ctx.lineTo(c, size - 2);
    ctx.lineTo(2, c);
    ctx.closePath();
  } else {
    ctx.arc(c, c, c - 2, 0, Math.PI * 2);
  }
  // Home is hollow, so it reads as a place, not a point to fly to.
  ctx.fillStyle = kind === "home" ? bg : color;
  ctx.globalAlpha = kind === "home" ? 0.85 : 1;
  ctx.fill();
  ctx.globalAlpha = 1;
  ctx.lineWidth = kind === "home" ? 3 : 1.5;
  ctx.strokeStyle = kind === "home" ? color : "rgba(0,0,0,0.85)";
  ctx.stroke();
  if (kind === "loiter") {
    ctx.beginPath();
    ctx.arc(c, c, c - 5, 0, Math.PI * 2);
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(255,255,255,0.7)";
    ctx.stroke();
  }
  if (text) {
    ctx.fillStyle = kind === "home" ? color : "#ffffff";
    ctx.font = `700 ${text.length > 1 ? 10 : 12}px Inter, Arial, sans-serif`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, c, c + 0.5);
  }
  return { url: canvas.toDataURL(), size };
}

/** A map label: light text on a dark box, below the point (`pixelY` > 0) or above. */
function label(
  text: string,
  pixelY: number,
  range = PLAN_LABEL_RANGE_M,
): C.LabelGraphics.ConstructorOptions {
  return {
    text,
    font: "600 11px Inter, Arial, sans-serif",
    fillColor: C.Color.WHITE,
    showBackground: true,
    backgroundColor: C.Color.BLACK.withAlpha(0.6),
    backgroundPadding: new C.Cartesian2(4, 2),
    verticalOrigin: pixelY < 0 ? C.VerticalOrigin.BOTTOM : C.VerticalOrigin.TOP,
    pixelOffset: new C.Cartesian2(0, pixelY),
    disableDepthTestDistance: Number.POSITIVE_INFINITY,
    distanceDisplayCondition: new C.DistanceDisplayCondition(0, range),
  };
}

/** Home's height above sea level: from telemetry (where it is less its height above home), or
 * the mission's home item, or the pad's ground. */
export function homeAmsl(
  t: Pick<TelemetryMsg, "alt_amsl" | "alt_rel"> | null,
  p: Pick<PlanMsg, "mission"> | null,
  pad: number | null,
): number | null {
  if (t && t.alt_amsl !== null && t.alt_rel !== null) return t.alt_amsl - t.alt_rel;
  const home = p?.mission.find((i) => i.seq === 0);
  if (home && FRAME_AMSL.has(home.frame) && (home.lat !== 0 || home.lon !== 0)) return home.alt;
  return pad;
}

/** An item's height above sea level, home at `home`. */
export function itemAmsl(item: Pick<PlanItem, "alt" | "frame">, home: number): number {
  return FRAME_AMSL.has(item.frame) ? item.alt : home + Math.max(item.alt, 0);
}

const finite = (c: C.Cartesian3): boolean =>
  Number.isFinite(c.x) && Number.isFinite(c.y) && Number.isFinite(c.z);

/** `pts` safe for a polyline: finite, no point repeating the one before; none at all when that
 * leaves fewer than two. Cesium throws on worse, and a throw in its render loop stops it. */
export function cleanLine(pts: readonly C.Cartesian3[]): C.Cartesian3[] {
  const out: C.Cartesian3[] = [];
  for (const p of pts) {
    if (!finite(p)) return [];
    const last = out[out.length - 1];
    if (!last || C.Cartesian3.distanceSquared(last, p) > 1e-6) out.push(p);
  }
  return out.length >= 2 ? out : [];
}

interface Sample {
  pos: C.Cartesian3;
  at: number;
  /** Home's ground, map height. */
  ground: number;
  t: TelemetryMsg;
}

export class MapLayer {
  private readonly source = new C.CustomDataSource("map-cop");
  private readonly planSource = new C.CustomDataSource("map-cop-plan");
  private readonly colors = palette();
  private on = false;
  private prev: Sample | null = null;
  private cur: Sample | null = null;
  private trail: { pos: C.Cartesian3; at: number }[] = [];
  private trailAt = 0;
  private now = new C.Cartesian3();
  private placed = false;
  private symbols: { name: string; plain: SymbolImage; aut: SymbolImage } | null = null;
  private planKey = "";
  private planRef: PlanMsg | null = null;
  private planVersion = 0;
  private planJson = "";
  private readonly stop: () => void;

  constructor(private readonly world: World) {
    void world.viewer.dataSources.add(this.source);
    void world.viewer.dataSources.add(this.planSource);
    this.source.show = false;
    this.planSource.show = false;
    this.addVehicle();
    this.stop = effect(() => {
      const t = telemetry();
      this.sample(t);
    });
  }

  setOn(on: boolean): void {
    this.on = on;
    this.source.show = on;
    this.planSource.show = on;
    if (on) this.update(performance.now());
  }

  /** Each animation frame while on: the vehicle eased between its last two samples, the trail,
   * and the plan rebuilt when it, home's height or the map's offset changed. */
  update(now: number): void {
    if (!this.on) return;
    const cur = this.cur;
    this.placed = cur !== null;
    if (cur) {
      const prev = this.prev;
      if (prev) {
        const span = Math.max(cur.at - prev.at, 1);
        const k = Math.min(Math.max((now - cur.at) / span, 0), 1);
        C.Cartesian3.lerp(prev.pos, cur.pos, k, this.now);
      } else {
        C.Cartesian3.clone(cur.pos, this.now);
      }
    }
    this.updatePlan();
  }

  dispose(): void {
    this.stop();
    this.world.viewer.dataSources.remove(this.source, true);
    this.world.viewer.dataSources.remove(this.planSource, true);
  }

  // -- the vehicle ---------------------------------------------------------------------------

  private sample(t: TelemetryMsg | null): void {
    const placed =
      t !== null &&
      [t.lat, t.lon, t.alt_amsl].every((x) => x !== null && Number.isFinite(x)) &&
      (t.alt_rel === null || Number.isFinite(t.alt_rel));
    if (!t || !placed || t.lat === null || t.lon === null || t.alt_amsl === null) {
      this.prev = this.cur = null;
      this.trail = [];
      return;
    }
    const at = performance.now();
    const offset = mapOffset();
    const pos = C.Cartesian3.fromDegrees(t.lon, t.lat, t.alt_amsl + offset);
    const ground = t.alt_amsl - (t.alt_rel ?? 0) + offset;
    if (this.cur && C.Cartesian3.distance(this.cur.pos, pos) > TRAIL.jumpM) {
      this.prev = null;
      this.trail = [];
    } else {
      this.prev = this.cur;
    }
    this.cur = { pos, at, ground, t };
    if (at - this.trailAt >= TRAIL.everyS * 1000) {
      this.trailAt = at;
      this.trail.push({ pos, at });
      while (this.trail.length && at - this.trail[0]!.at > TRAIL.seconds * 1000) this.trail.shift();
    }
  }

  private symbolFor(t: TelemetryMsg | undefined): SymbolImage {
    const name = prefs().player.name.toUpperCase();
    if (this.symbols?.name !== name) {
      this.symbols = { name, plain: symbolImage(false, name), aut: symbolImage(true, name) };
    }
    return t?.armed && isAutonomous(t.mode) ? this.symbols.aut : this.symbols.plain;
  }

  private addVehicle(): void {
    const c = this.colors;
    const entities = this.source.entities;
    const first = this.symbolFor(undefined);
    const placed = new C.CallbackProperty(() => this.placed, false);
    const position = new C.CallbackPositionProperty(
      () => (this.placed ? this.now : undefined),
      false,
    );
    const imperial = (): boolean => prefs().hud.units === "imperial";
    entities.add({
      id: "map-vehicle",
      position,
      billboard: {
        image: new C.CallbackProperty(() => this.symbolFor(this.cur?.t).url, false),
        width: new C.CallbackProperty(() => this.symbolFor(this.cur?.t).width, false),
        height: new C.CallbackProperty(() => this.symbolFor(this.cur?.t).height, false),
        // milsymbol's anchor (the frame's centre) SYMBOL_LIFT_PX above the vehicle.
        horizontalOrigin: C.HorizontalOrigin.LEFT,
        verticalOrigin: C.VerticalOrigin.TOP,
        pixelOffset: new C.CallbackProperty(() => {
          const s = this.symbolFor(this.cur?.t);
          return new C.Cartesian2(-s.anchor.x, -s.anchor.y - SYMBOL_LIFT_PX);
        }, false),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: placed,
      },
      point: {
        pixelSize: 7,
        color: C.Color.fromCssColorString(c.ember),
        outlineColor: C.Color.BLACK.withAlpha(0.7),
        outlineWidth: 2,
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        show: placed,
      },
      label: {
        text: new C.CallbackProperty(() => {
          const t = this.cur?.t;
          if (!t) return "";
          const h = (t.alt_rel ?? 0) * (imperial() ? 3.28084 : 1);
          const state = t.armed ? t.mode : `${t.mode} · DISARMED`;
          return `${state}\n${Math.round(h)} ${imperial() ? "ft" : "m"}`;
        }, false),
        font: "600 12px Inter, Arial, sans-serif",
        fillColor: C.Color.fromCssColorString(c.text),
        showBackground: true,
        backgroundColor: C.Color.fromCssColorString(c.bg).withAlpha(0.7),
        backgroundPadding: new C.Cartesian2(6, 4),
        horizontalOrigin: C.HorizontalOrigin.LEFT,
        verticalOrigin: C.VerticalOrigin.CENTER,
        pixelOffset: new C.Cartesian2(first.width - first.anchor.x + 6, -SYMBOL_LIFT_PX),
        disableDepthTestDistance: Number.POSITIVE_INFINITY,
        distanceDisplayCondition: new C.DistanceDisplayCondition(0, LABEL_RANGE_M),
        show: placed,
      },
    });
    // Polylines use arcType NONE: GEODESIC miscounts zero-length segments (a hover) and draws a
    // V from the vehicle back into its trail.
    entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => {
          const pts = this.trail.map((p) => p.pos);
          if (this.placed) pts.push(this.now);
          return cleanLine(pts);
        }, false),
        width: 3,
        material: new C.PolylineGlowMaterialProperty({
          color: C.Color.fromCssColorString(c.sky).withAlpha(0.9),
          glowPower: 0.18,
        }),
        show: placed,
      },
    });
    entities.add({
      polyline: {
        arcType: C.ArcType.NONE,
        positions: new C.CallbackProperty(() => {
          const cur = this.cur;
          if (!this.placed || !cur) return [];
          const carto = C.Cartographic.fromCartesian(this.now);
          if (!carto) return [];
          return cleanLine([
            C.Cartesian3.fromRadians(carto.longitude, carto.latitude, cur.ground),
            this.now,
          ]);
        }, false),
        width: 1.5,
        material: new C.PolylineDashMaterialProperty({
          color: C.Color.fromCssColorString(c.ember).withAlpha(0.8),
          dashLength: 8,
        }),
        show: new C.CallbackProperty(
          () => this.placed && (this.cur?.t.alt_rel ?? 0) > STALK_FROM_M,
          false,
        ),
      },
    });
  }

  // -- the plan ------------------------------------------------------------------------------

  private updatePlan(): void {
    const p = plan();
    const origin = status().origin;
    const home = homeAmsl(this.cur?.t ?? null, p, origin ? origin[2] : null);
    const offset = mapOffset();
    if (p !== this.planRef) {
      // A plan is re-sent now and then: only a different one is drawn again.
      this.planRef = p;
      const json = JSON.stringify(p);
      if (json !== this.planJson) {
        this.planJson = json;
        this.planVersion += 1;
      }
    }
    // Home's height to the decimetre: telemetry's own wander should not rebuild it.
    const key =
      p && home !== null ? `${this.planVersion}|${home.toFixed(1)}|${offset.toFixed(1)}` : "";
    if (key === this.planKey) return;
    this.planKey = key;
    this.planSource.entities.removeAll();
    if (p && home !== null && Number.isFinite(home) && Number.isFinite(offset)) {
      // Items with no finite place are left out: a NaN position throws inside Cesium.
      const ok = (i: PlanItem): boolean => [i.lat, i.lon, i.alt].every(Number.isFinite);
      this.drawPlan(
        {
          ...p,
          mission: p.mission.filter(ok),
          fence: p.fence.filter(ok),
          rally: p.rally.filter(ok),
        },
        home,
        offset,
      );
    }
  }

  private drawPlan(p: PlanMsg, home: number, offset: number): void {
    const c = this.colors;
    const entities = this.planSource.entities;
    const route = C.Color.fromCssColorString(c.ember);
    const at = (lat: number, lon: number, amsl: number): C.Cartesian3 =>
      C.Cartesian3.fromDegrees(lon, lat, amsl + offset);
    const ground = (lat: number, lon: number): C.Cartesian3 => at(lat, lon, home + GROUND_LIFT_M);
    const billboard = (m: {
      url: string;
      size: number;
    }): C.BillboardGraphics.ConstructorOptions => ({
      image: m.url,
      width: m.size,
      height: m.size,
      verticalOrigin: C.VerticalOrigin.CENTER,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
    });

    const homeItem = p.mission.find((i) => i.seq === 0 && (i.lat !== 0 || i.lon !== 0));
    let from: { lat: number; lon: number; amsl: number } | null = homeItem
      ? { lat: homeItem.lat, lon: homeItem.lon, amsl: home }
      : null;
    const homePlace = from;
    if (homePlace) {
      entities.add({
        position: ground(homePlace.lat, homePlace.lon),
        billboard: billboard(planMarker("H", c.ember, "home", c.bg)),
        label: label("HOME", 15),
      });
    }

    const groundTrack: C.Cartesian3[] = homePlace ? [ground(homePlace.lat, homePlace.lon)] : [];
    let checkpoint = 0;
    const nav = p.mission.filter((i) => i.seq > 0 && NAV.has(i.command));
    for (const item of nav) {
      const placedHere = item.lat !== 0 || item.lon !== 0;
      let to: { lat: number; lon: number; amsl: number } | null;
      if (item.command === CMD.RTL) {
        to = homePlace;
      } else if (placedHere) {
        to = { lat: item.lat, lon: item.lon, amsl: itemAmsl(item, home) };
      } else {
        // Takeoff or land where it is: straight up or down from the last point.
        to = from
          ? {
              lat: from.lat,
              lon: from.lon,
              amsl: item.command === CMD.LAND ? home : itemAmsl(item, home),
            }
          : null;
      }
      if (!to) continue;
      if (from) {
        const pts =
          item.command === CMD.RTL
            ? [
                at(from.lat, from.lon, from.amsl),
                at(to.lat, to.lon, from.amsl),
                ground(to.lat, to.lon),
              ]
            : [at(from.lat, from.lon, from.amsl), at(to.lat, to.lon, to.amsl)];
        entities.add({
          polyline: {
            arcType: C.ArcType.NONE,
            positions: cleanLine(pts),
            width: 3,
            material: new C.PolylineOutlineMaterialProperty({
              color: route.withAlpha(0.9),
              outlineColor: C.Color.BLACK.withAlpha(0.6),
              outlineWidth: 1,
            }),
            depthFailMaterial: new C.ColorMaterialProperty(route.withAlpha(0.25)),
          },
        });
      }
      from = to;
      groundTrack.push(ground(to.lat, to.lon));
      if (item.command === CMD.RTL) continue;
      // The marker, a drop line to the ground under it, its name and height.
      const isAction = item.command === CMD.TAKEOFF || item.command === CMD.LAND;
      if (!isAction) checkpoint += 1;
      const disc =
        item.command === CMD.TAKEOFF ? "T" : item.command === CMD.LAND ? "L" : String(checkpoint);
      const name =
        item.command === CMD.TAKEOFF
          ? "SP"
          : item.command === CMD.LAND
            ? "LZ"
            : `CP${checkpoint}${LOITERS.has(item.command) ? " LOITER" : ""}`;
      const kind: MarkerKind = isAction
        ? "action"
        : LOITERS.has(item.command)
          ? "loiter"
          : "waypoint";
      const pos = at(to.lat, to.lon, to.amsl);
      const foot = ground(to.lat, to.lon);
      if (C.Cartesian3.distance(pos, foot) > 1) {
        entities.add({
          polyline: {
            arcType: C.ArcType.NONE,
            positions: cleanLine([foot, pos]),
            width: 1,
            material: new C.PolylineDashMaterialProperty({
              color: route.withAlpha(0.6),
              dashLength: 6,
            }),
          },
        });
      }
      const terrain = FRAME_TERRAIN.has(item.frame);
      const height = FRAME_AMSL.has(item.frame) ? item.alt - home : item.alt;
      const heightText =
        item.command === CMD.LAND ? "" : ` · ${Math.round(height)} m${terrain ? " AGL" : ""}`;
      entities.add({
        position: pos,
        billboard: billboard(planMarker(disc, c.ember, kind, c.bg)),
        label: label(`${name}${heightText}`, 15),
      });
    }
    if (groundTrack.length > 1) {
      entities.add({
        polyline: {
          arcType: C.ArcType.NONE,
          positions: cleanLine(groundTrack),
          width: 1.5,
          material: new C.PolylineDashMaterialProperty({
            color: route.withAlpha(0.45),
            dashLength: 10,
          }),
        },
      });
    }

    for (const item of p.mission) {
      if (
        (item.command === CMD.ROI || item.command === CMD.ROI_LOCATION) &&
        (item.lat !== 0 || item.lon !== 0)
      ) {
        entities.add({
          position: ground(item.lat, item.lon),
          billboard: billboard(planMarker("", c.sun, "roi", c.bg)),
          label: label("ROI", 15),
        });
      }
    }
    for (const r of p.rally) {
      entities.add({
        position: at(r.lat, r.lon, itemAmsl(r, home)),
        billboard: billboard(planMarker("R", c.tide, "rally", c.bg)),
        label: label(`RALLY · ${Math.round(FRAME_AMSL.has(r.frame) ? r.alt - home : r.alt)} m`, 15),
      });
    }
    this.drawFence(p.fence, home, offset);
  }

  /** The geofence: polygons from runs of their vertices, circles' centres (the plan carries no
   * radius), the breach return point. */
  private drawFence(items: PlanItem[], home: number, offset: number): void {
    const c = this.colors;
    const entities = this.planSource.entities;
    const fence = C.Color.fromCssColorString(c.sun);
    const base = home + offset;
    for (const poly of fencePolygons(items)) {
      if (!poly.points.every(([lat, lon]) => Number.isFinite(lat) && Number.isFinite(lon)))
        continue;
      const inclusion = poly.command === CMD.FENCE_POLY_IN;
      const flat = poly.points.flatMap(([lat, lon]) => [lon, lat]);
      entities.add({
        polygon: {
          hierarchy: C.Cartesian3.fromDegreesArray(flat),
          perPositionHeight: false,
          height: base,
          material: fence.withAlpha(inclusion ? 0.06 : 0.2),
          outline: false,
        },
      });
      const closed = [...poly.points, poly.points[0]!].map(([lat, lon]) =>
        C.Cartesian3.fromDegrees(lon, lat, base + GROUND_LIFT_M),
      );
      entities.add({
        polyline: {
          arcType: C.ArcType.NONE,
          positions: cleanLine(closed),
          width: 2,
          material: new C.ColorMaterialProperty(fence.withAlpha(0.9)),
        },
      });
    }
    for (const item of items) {
      const where = C.Cartesian3.fromDegrees(item.lon, item.lat, base + GROUND_LIFT_M);
      if (item.command === CMD.FENCE_CIRCLE_IN || item.command === CMD.FENCE_CIRCLE_OUT) {
        entities.add({
          position: where,
          point: {
            pixelSize: 8,
            color: C.Color.TRANSPARENT,
            outlineColor: fence,
            outlineWidth: 2,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: label(item.command === CMD.FENCE_CIRCLE_IN ? "FENCE CIRCLE" : "NO-FLY CIRCLE", 12),
        });
      } else if (item.command === CMD.FENCE_RETURN) {
        entities.add({
          position: where,
          billboard: {
            image: planMarker("B", c.tide, "rally", c.bg).url,
            width: 20,
            height: 20,
            verticalOrigin: C.VerticalOrigin.CENTER,
            disableDepthTestDistance: Number.POSITIVE_INFINITY,
          },
          label: label("BREACH RETURN", 15),
        });
      }
    }
  }
}

/** The fence's polygons: each run of vertices of one kind (inclusion or exclusion), in order. A
 * run under three vertices is no polygon. */
export function fencePolygons(
  items: readonly PlanItem[],
): { command: number; points: [number, number][] }[] {
  const out: { command: number; points: [number, number][] }[] = [];
  let run: { command: number; points: [number, number][] } | null = null;
  for (const item of items) {
    if (item.command !== CMD.FENCE_POLY_IN && item.command !== CMD.FENCE_POLY_OUT) {
      run = null;
      continue;
    }
    if (!run || run.command !== item.command) {
      run = { command: item.command, points: [] };
      out.push(run);
    }
    run.points.push([item.lat, item.lon]);
  }
  return out.filter((r) => r.points.length >= 3);
}
