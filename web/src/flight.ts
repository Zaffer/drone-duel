// A flight: one place flown, from the menu's Fly to Quit (DESIGN.md §6.4, §11). It asks the
// server for the vehicle, measures where the map's ground meets the sim's, draws the vehicle,
// runs the view's camera, the camera feed, the HUD's minimap and dials, and the dynamics overlay
// each frame, sends the pilot's sticks to the firmware and the map's ground to the physics.
// The flight owns the place's local frame and its ground (engine/), built once the pad's height on
// the map is known and kept while the place is: the truck drives on it, a mode stands on it, and
// the aircraft's physics is told the ground under it from it. The mode (modes/*) rides on top,
// through `mode`. A mode that owns the view (Shoot Down: the
// player is on the ground, the vehicle the target) places the camera itself, and nobody here flies
// the vehicle: no rig, no camera feed, no operator, no sticks, no motor in the player's ears.

import { effect, signal } from "alien-signals";
import { rotorNorm, truckEngine } from "./audio/levels.ts";
import { sounds } from "./audio/sounds.ts";
import { DynamicsOverlay } from "./dynamics/overlay.ts";
import { LocalFrame } from "./engine/frame.ts";
import { Ground } from "./engine/ground.ts";
import { loop } from "./engine/loop.ts";
import { Minimap } from "./hud/minimap.ts";
import { perf } from "./hud/perf.ts";
import { toFirmware } from "./input/mapping.ts";
import { radio } from "./input/radio.ts";
import type { RcMsg, Vehicle } from "./net/protocol.ts";
import { link, type Pose } from "./net/server.ts";
import { GOGGLES_M, Operator, operatorSpot } from "./people/operator.ts";
import {
  content,
  vehicle as currentVehicle,
  dynamicsOn,
  paused,
  toast,
  vehicleId,
  view,
  wind,
} from "./state/app.ts";
import { inputSource } from "./state/input.ts";
import { prefs, type ViewId } from "./state/prefs.ts";
import { phase, sessionStarting, status } from "./state/session.ts";
import { makeBody } from "./vehicles/make.ts";
import { truckControls } from "./vehicles/truck/controls.ts";
import { Truck } from "./vehicles/truck/truck.ts";
import type { Body } from "./vehicles/types.ts";
import { CameraFeed } from "./video/feed.ts";
import { MapLayer } from "./views/mapview.ts";
import { makeRigs, type Rig, type RigContext } from "./views/rigs.ts";
import { mapOffset, moveBy } from "./world/geo.ts";
import type { World } from "./world/world.ts";

/** What a mode adds to a flight (modes/*). A mode with systems of its own joins the frame loop
 * itself (engine/loop.ts); `tick` is run in the flight's place in it. */
export interface ModeRuntime {
  tick?(now: number, pose: Pose | null): void;
  /** Marks for the minimap (gates, the other player, the truck). */
  marks(): Parameters<Minimap["setMarks"]>[0];
  dispose(): void;
}

const GROUND_MS = 50;
const GROUND_LEAD_S = 0.3;

export class Flight {
  readonly body = signal<Body | null>(null);
  /** The pad as the sim has it: lat, lon, ground m above sea level, heading. */
  readonly pad = signal<RigContext["pad"] | null>(null);
  /** The place's ground, with its local frame at the pad (engine/ground.ts), once known. */
  readonly ground = signal<Ground | null>(null);
  private groundAt = "";
  mode: ModeRuntime | null = null;
  /** The pilot standing behind the pad (not for the truck, which they would be driving). */
  operator: Operator | null = null;
  /** The truck starts this far ahead of the pad (a duel's driver), m. */
  truckAhead = 0;
  private readonly abort = new AbortController();
  private readonly rigs = makeRigs();
  private rig: Rig | null = null;
  private feed: CameraFeed;
  private minimap: Minimap;
  private mapLayer: MapLayer;
  private dynamics: DynamicsOverlay;
  private seq = 0;
  private bodyKey = "";
  private groundBusy = false;
  private groundTold = 0;
  private lastFrame = 0;
  private disposers: (() => void)[] = [];

  readonly ownsView: boolean;

  constructor(
    readonly world: World,
    hud: { feed: HTMLElement; minimap: HTMLElement; dynamics: HTMLElement },
    opts: { ownsView?: boolean } = {},
  ) {
    this.ownsView = opts.ownsView ?? false;
    this.feed = new CameraFeed(world, hud.feed);
    this.minimap = new Minimap(hud.minimap);
    this.mapLayer = new MapLayer(world);
    this.dynamics = new DynamicsOverlay(world, hud.dynamics);
    const signal = this.abort.signal;
    world.mouseCamera = false;

    loop.each("flight", "place", (now) => this.frame(now), signal);
    radio.samples().subscribe((ch) => this.sticks(ch), { signal });
    loop.each("ground report", "sim", (now) => this.reportGround(now), signal);
    let dynamicsData: Record<string, unknown> | null = null;
    link.on("dynamics").subscribe((d) => (dynamicsData = d.data), { signal });
    this.dynamicsData = () => dynamicsData;
    link.on("physics").subscribe(
      (p) => {
        if (p.phase === "crashed") sounds.crash();
      },
      { signal },
    );

    this.disposers.push(
      effect(() => this.onStatus()),
      effect(() => this.setView(view())),
      effect(() => {
        const system = prefs().video.system;
        this.feed.setSystem(system);
        link.send({ ch: "video", system });
      }),
      effect(() => {
        const on = dynamicsOn() && (currentVehicle()?.dynamics_overlay ?? false);
        this.dynamics.setOn(on);
        link.send({ ch: "dynamics", on });
      }),
    );
  }

  private dynamicsData: () => Record<string, unknown> | null;

  /** Start a vehicle at a location, on a pad (0 player 1, 1 player 2). */
  start(locationId: string, vehicle: string, pad = 0): void {
    vehicleId(vehicle);
    link.send({ ch: "video", system: prefs().video.system });
    link.send({ ch: "session", action: "start", location: locationId, vehicle, pad });
    sessionStarting(locationId, vehicle, pad);
  }

  switchVehicle(vehicle: string): void {
    vehicleId(vehicle);
    const s = status();
    link.send({ ch: "session", action: "switch", location: null, vehicle, pad: s.pad });
    sessionStarting(s.location, vehicle, s.pad);
  }

  reset(): void {
    const b = this.body();
    if (b instanceof Truck) b.respawn();
    else
      link.send({
        ch: "session",
        action: "reset",
        location: null,
        vehicle: null,
        pad: status().pad,
      });
  }

  throw(): void {
    link.send({ ch: "session", action: "throw", location: null, vehicle: null, pad: 0 });
  }

  /** The vehicle is destroyed (a crash or a strike in a duel): its physics stops dead. */
  destroy(): void {
    link.send({ ch: "session", action: "destroy", location: null, vehicle: null, pad: 0 });
  }

  pose(now = performance.now()): Pose | null {
    return this.body()?.poseAt(now) ?? null;
  }

  dispose(): void {
    this.abort.abort();
    for (const d of this.disposers) d();
    this.mode?.dispose();
    this.mode = null;
    this.rig?.leave(this.rigContext()!);
    this.body()?.dispose();
    this.body(null);
    this.operator?.dispose();
    this.operator = null;
    this.feed.dispose();
    this.minimap.dispose();
    this.mapLayer.dispose();
    this.dynamics.dispose();
    this.ground()?.dispose();
    this.ground(null);
    sounds.stopMotor();
    this.world.mouseCamera = true;
    link.send({ ch: "dynamics", on: false });
    link.send({ ch: "session", action: "stop", location: null, vehicle: null, pad: 0 });
  }

  // The vehicle's body follows the session: a new one for a new vehicle, once the pad's ground
  // on the map is known (mapOffset).

  private onStatus(): void {
    const s = status();
    if (!s.vehicle || !s.origin || s.state === "idle") return;
    const key = `${s.location}/${s.vehicle}/${s.pad}`;
    if (key === this.bodyKey) return;
    this.bodyKey = key;
    const spec = content()?.vehicles[s.vehicle];
    if (!spec) return;
    let [lat, lon, alt] = s.origin;
    if (spec.physics.kind === "browser-truck" && this.truckAhead) {
      const hd = (s.heading_deg * Math.PI) / 180;
      [lat, lon] = moveBy(lat, lon, Math.cos(hd) * this.truckAhead, Math.sin(hd) * this.truckAhead);
    }
    const pad = { lat, lon, alt, heading_deg: s.heading_deg };
    void this.spawnBody(key, spec, pad, s.origin);
  }

  private async spawnBody(
    key: string,
    spec: Vehicle,
    pad: RigContext["pad"],
    origin: [number, number, number],
  ): Promise<void> {
    this.body()?.dispose();
    this.body(null);
    this.operator?.dispose();
    this.operator = null;
    // Where the map's ground meets the sim's: the location's pad.
    const h = await this.world.groundHeightSettled(origin[0], origin[1]);
    if (this.bodyKey !== key || this.abort.signal.aborted) return;
    mapOffset(h - origin[2]);
    const place = status().location ?? "";
    if (!this.ground() || this.groundAt !== place) {
      this.ground()?.dispose();
      this.ground(null);
      const ground = await Ground.create(this.world, new LocalFrame(origin[0], origin[1], h));
      if (this.bodyKey !== key || this.abort.signal.aborted) {
        ground.dispose();
        return;
      }
      this.groundAt = place;
      this.ground(ground);
    }
    const ground = this.ground()!;
    if (spec.kind !== "ground" && !this.ownsView) {
      const at = { lat: origin[0], lon: origin[1], heading_deg: pad.heading_deg };
      const spot = await operatorSpot(this.world, at, mapOffset());
      if (this.bodyKey !== key || this.abort.signal.aborted) return;
      this.operator = new Operator(this.world, spot);
    }
    if (pad.lat !== origin[0] || pad.lon !== origin[1]) {
      pad.alt = (await this.world.groundHeightSettled(pad.lat, pad.lon)) - mapOffset();
    }
    this.pad(pad);
    const body = await makeBody(this.world, spec, pad, ground);
    if (this.bodyKey !== key || this.abort.signal.aborted) {
      body.dispose();
      return;
    }
    if (body instanceof Truck) {
      let last: string | null = null;
      body.onNotice = (text) => {
        if (text && text !== last) toast(text, "info", 4000);
        last = text;
      };
    }
    this.body(body);
    this.rig?.leave(this.rigContext()!);
    this.rig = null;
    this.setView(view());
  }

  private rigContext(): RigContext | null {
    const v = currentVehicle();
    const pad = this.pad();
    if (!v || !pad) return null;
    const p = prefs();
    const camera = p.video.tiltDeg === null ? v.camera : { ...v.camera, tilt_deg: p.video.tiltDeg };
    return {
      world: this.world,
      camera,
      pad,
      chaseDistance: p.view.chaseDistance,
      fovDeg: p.view.fovDeg,
      // The vehicle's size against a 5" quad's: the chase arm grows with it.
      scale: { multirotor: 1, fixedwing: 1.3, ground: 2.6 }[v.kind],
    };
  }

  private setView(id: ViewId): void {
    if (this.ownsView) {
      this.feed.setOn(false);
      this.mapLayer.setOn(false);
      this.body()?.setVisible(true);
      this.world.mouseCamera = false;
      return;
    }
    const ctx = this.rigContext();
    if (!ctx) return;
    this.rig?.leave(ctx);
    const rig = this.rigs[id];
    this.rig = rig;
    rig.enter(ctx, this.pose());
    this.world.mouseCamera = rig.mouse;
    this.body()?.setVisible(rig.showVehicle);
    this.operator?.setVisible(!rig.fromPilot);
    this.feed.setOn(id === "fpv");
    this.mapLayer.setOn(id === "map");
  }

  // Each frame.

  private frame(now: number): void {
    const dt = this.lastFrame ? Math.min(0.1, (now - this.lastFrame) / 1000) : 0;
    this.lastFrame = now;
    const body = this.body();
    body?.draw(now);
    const pose = body?.poseAt(now) ?? null;
    const ground = this.ground();
    if (ground) {
      ground.focus("vehicle", pose ? ground.frame.fromSim(pose.lat, pose.lon, pose.alt) : null);
    }
    perf.frame(now, body?.ahead?.(now) ?? 0);
    const ctx = this.rigContext();
    if (pose && ctx && this.rig && !paused()) this.rig.update(ctx, pose, dt);
    const pad = this.pad();
    const pilot = this.operator?.spot ?? pad;
    if (pilot) this.feed.setPilot(pilot.lat, pilot.lon, pilot.alt + GOGGLES_M);
    this.feed.frame(now, pose);
    this.mode?.tick?.(now, pose);
    this.minimap.setMarks([
      ...(pad ? [{ lat: pad.lat, lon: pad.lon, kind: "pad" as const }] : []),
      ...(this.mode?.marks() ?? []),
    ]);
    this.minimap.update(pose);
    this.mapLayer.update(now);
    this.dynamics.update(this.dynamicsData(), pose);
    this.motorSound(pose);
  }

  private motorSound(pose: Pose | null): void {
    if (this.ownsView) return;
    const v = currentVehicle();
    if (!v || !pose || phase() === "crashed") {
      sounds.stopMotor();
      return;
    }
    if (v.kind === "multirotor") {
      const each = pose.act.map((w) => rotorNorm("multirotor", w));
      const mean = (pose.act[0] + pose.act[1] + pose.act[2] + pose.act[3]) / 4;
      const norm = rotorNorm("multirotor", mean);
      sounds.motor("multirotor", norm, norm, each);
    } else if (v.kind === "fixedwing") {
      const norm = rotorNorm("fixedwing", pose.act[2]);
      sounds.motor("fixedwing", norm, norm);
    } else {
      const e = truckEngine(pose.act[0], pose.act[2]);
      sounds.motor("ground", e.load, e.rpm);
    }
  }

  // The pilot's sticks: to the firmware (aircraft), or the truck's controls.

  private sticks(ch: number[]): void {
    const v = currentVehicle();
    const body = this.body();
    if (!v || this.ownsView) return;
    if (body instanceof Truck) {
      if (paused()) return;
      const pads = navigator
        .getGamepads()
        .filter((p): p is Gamepad => !!p && p.mapping === "standard");
      body.setControls(truckControls(ch, radio.held, pads[0] ?? null));
      return;
    }
    // The server's radio flies from the server (rc/pilot.py), never through this page's busy
    // thread (DESIGN.md §12); the page sends only its own controller's sticks. The session
    // throws a hand-launched wing on its launch switch's flick up, whoever flies it.
    if (inputSource() === "server") return;
    // Pausing cannot stop the firmware, so the radio keeps flying it: going quiet here would be a
    // lost link, and the firmware's failsafe would disarm it.
    if (v.firmware.kind === "none" || status().state !== "ready") return;
    link.send({ ch: "rc", seq: ++this.seq, us: toFirmware(ch, v.firmware.kind) as RcMsg["us"] });
  }

  // The map's ground under the vehicle, a little ahead of it, for its physics: from the ground's
  // colliders, or, where none are built (the map view, far off), Cesium's lookup.

  private reportGround(now: number): void {
    const body = this.body();
    if (!body || body instanceof Truck || now - this.groundTold < GROUND_MS) return;
    const pose = body.poseAt(now);
    if (!pose) return;
    this.groundTold = now;
    const R = 6378137;
    const lat = pose.lat + ((pose.v[0] * GROUND_LEAD_S) / R) * (180 / Math.PI);
    const lon =
      pose.lon +
      ((pose.v[1] * GROUND_LEAD_S) / (R * Math.cos((pose.lat * Math.PI) / 180))) * (180 / Math.PI);
    const ground = this.ground();
    if (ground) {
      // Down from just over it: a tree's crown it flies under is not its ground.
      const p = ground.frame.fromSim(lat, lon, pose.alt);
      const h = ground.covers(p) ? ground.heightAt(p[0], p[1], p[2] + 1) : undefined;
      if (h !== undefined) {
        const at = ground.frame.toGeo([p[0], p[1], h]);
        link.send({ ch: "ground", lat, lon, alt: at.alt });
        return;
      }
    }
    if (this.groundBusy) return;
    this.groundBusy = true;
    void this.world
      .groundHeight(lat, lon)
      .then((h) => {
        if (h !== undefined) link.send({ ch: "ground", lat, lon, alt: h - mapOffset() });
      })
      .finally(() => {
        this.groundBusy = false;
      });
  }
}

/** Send the wind tab's wind to the physics. */
export function sendWind(gustNow = false): void {
  const w = wind();
  if (!w) return;
  link.send({ ch: "wind", wind: w, gust_now: gustNow });
}
