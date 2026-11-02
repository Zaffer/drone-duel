// A flight: one scenario flown, from the menu's Fly to Quit (DESIGN.md §6.4, §11). It asks the
// server for the vehicle, measures where the map's ground meets the sim's, draws the vehicle,
// runs the view's camera, the camera feed, the HUD's minimap and dials, and the dynamics overlay
// each frame, sends the pilot's sticks to the firmware and the map's ground to the physics.
// The mode (modes/*) rides on top, through `mode`.

import { effect, signal } from "alien-signals";
import { rotorNorm, truckEngine } from "./audio/levels.ts";
import { sounds } from "./audio/sounds.ts";
import { DynamicsOverlay } from "./dynamics/overlay.ts";
import { Minimap } from "./hud/minimap.ts";
import { switchPos } from "./input/mapping.ts";
import { radio } from "./input/radio.ts";
import type { RcMsg, Vehicle } from "./net/protocol.ts";
import { link, type Pose } from "./net/server.ts";
import { GOGGLES_M, Operator, operatorSpot } from "./people/operator.ts";
import {
  content,
  vehicle as currentVehicle,
  dynamicsOn,
  paused,
  scenario,
  toast,
  vehicleId,
  view,
  wind,
} from "./state/app.ts";
import { toUs } from "./state/input.ts";
import { prefs, type ViewId } from "./state/prefs.ts";
import { phase, status } from "./state/session.ts";
import { every, frames } from "./streams/time.ts";
import { makeBody } from "./vehicles/make.ts";
import { truckControls } from "./vehicles/truck/controls.ts";
import { Truck } from "./vehicles/truck/truck.ts";
import type { Body } from "./vehicles/types.ts";
import { CameraFeed } from "./video/feed.ts";
import { MapLayer } from "./views/mapview.ts";
import { makeRigs, type Rig, type RigContext } from "./views/rigs.ts";
import { mapOffset, moveBy } from "./world/geo.ts";
import type { World } from "./world/world.ts";

/** What a mode adds to a flight (modes/*). */
export interface ModeRuntime {
  tick(now: number, pose: Pose | null): void;
  /** Marks for the minimap (gates, the other player, the truck). */
  marks(): Parameters<Minimap["setMarks"]>[0];
  dispose(): void;
}

const GROUND_MS = 50;
const GROUND_LEAD_S = 0.3;

/** The firmware's own channel conventions on the way out: ArduPilot reads pitch forward as a
 * low pulse, INAV as a high one (as Betaflight). */
export function toFirmware(ch: readonly number[], spec: Vehicle): number[] {
  const us = ch.slice(0, 16).map(toUs);
  while (us.length < 8) us.push(1000);
  if (spec.firmware.kind === "ardupilot-copter" && ch[1] !== undefined) us[1] = toUs(-ch[1]);
  return us;
}

export class Flight {
  readonly body = signal<Body | null>(null);
  /** The pad as the sim has it: lat, lon, ground m above sea level, heading. */
  readonly pad = signal<RigContext["pad"] | null>(null);
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
  private launchHigh = true;
  private lastFrame = 0;
  private disposers: (() => void)[] = [];

  constructor(
    readonly world: World,
    hud: { feed: HTMLElement; minimap: HTMLElement; dynamics: HTMLElement },
  ) {
    this.feed = new CameraFeed(world, hud.feed);
    this.minimap = new Minimap(hud.minimap);
    this.mapLayer = new MapLayer(world);
    this.dynamics = new DynamicsOverlay(world, hud.dynamics);
    const signal = this.abort.signal;
    world.mouseCamera = false;

    frames().subscribe((now) => this.frame(now), { signal });
    radio.samples().subscribe((ch) => this.sticks(ch), { signal });
    every(GROUND_MS).subscribe(() => void this.reportGround(), { signal });
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

  /** Start the scenario with a vehicle on a pad (0 player 1, 1 player 2). */
  start(scenarioId: string, vehicle: string, pad = 0): void {
    vehicleId(vehicle);
    link.send({ ch: "video", system: prefs().video.system });
    link.send({ ch: "session", action: "start", scenario: scenarioId, vehicle, pad });
  }

  switchVehicle(vehicle: string): void {
    vehicleId(vehicle);
    link.send({ ch: "session", action: "switch", scenario: null, vehicle, pad: status().pad });
  }

  reset(): void {
    const b = this.body();
    if (b instanceof Truck) b.respawn();
    else
      link.send({
        ch: "session",
        action: "reset",
        scenario: null,
        vehicle: null,
        pad: status().pad,
      });
  }

  throw(): void {
    link.send({ ch: "session", action: "throw", scenario: null, vehicle: null, pad: 0 });
  }

  /** The vehicle is destroyed (a crash or a strike in a duel): its physics stops dead. */
  destroy(): void {
    link.send({ ch: "session", action: "destroy", scenario: null, vehicle: null, pad: 0 });
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
    sounds.stopMotor();
    this.world.mouseCamera = true;
    link.send({ ch: "dynamics", on: false });
    link.send({ ch: "session", action: "stop", scenario: null, vehicle: null, pad: 0 });
  }

  // The vehicle's body follows the session: a new one for a new vehicle, once the pad's ground
  // on the map is known (mapOffset).

  private onStatus(): void {
    const s = status();
    if (!s.vehicle || !s.origin || s.state === "idle") return;
    const key = `${s.scenario}/${s.vehicle}/${s.pad}`;
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
    // Where the map's ground meets the sim's: the scenario's pad.
    const h = await this.world.groundHeightSettled(origin[0], origin[1]);
    if (this.bodyKey !== key || this.abort.signal.aborted) return;
    mapOffset(h - origin[2]);
    if (spec.kind !== "ground") {
      const at = { lat: origin[0], lon: origin[1], heading_deg: pad.heading_deg };
      const spot = await operatorSpot(this.world, at, mapOffset());
      if (this.bodyKey !== key || this.abort.signal.aborted) return;
      this.operator = new Operator(this.world, spot);
    }
    if (pad.lat !== origin[0] || pad.lon !== origin[1]) {
      pad.alt = (await this.world.groundHeightSettled(pad.lat, pad.lon)) - mapOffset();
    }
    this.pad(pad);
    const body = await makeBody(this.world, spec, pad);
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
    radio.publish();
    const body = this.body();
    body?.draw(now);
    const pose = body?.poseAt(now) ?? null;
    const ctx = this.rigContext();
    if (pose && ctx && this.rig && !paused()) this.rig.update(ctx, pose, dt);
    const pad = this.pad();
    const pilot = this.operator?.spot ?? pad;
    if (pilot) this.feed.setPilot(pilot.lat, pilot.lon, pilot.alt + GOGGLES_M);
    this.feed.frame(now, pose);
    this.mode?.tick(now, pose);
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
    if (!v || paused()) return;
    if (body instanceof Truck) {
      const pads = navigator
        .getGamepads()
        .filter((p): p is Gamepad => !!p && p.mapping === "standard");
      body.setControls(truckControls(ch, radio.held, pads[0] ?? null));
      return;
    }
    if (v.firmware.kind === "none" || status().state !== "ready") return;
    link.send({ ch: "rc", seq: ++this.seq, us: toFirmware(ch, v) as RcMsg["us"] });
    // A hand-launched wing is thrown on its launch switch's flick up.
    const launch = v.aux?.launch;
    if (launch) {
      const high = switchPos(ch[launch - 1]) === 2;
      if (high && !this.launchHigh) this.throw();
      this.launchHigh = high;
    }
  }

  // The map's ground under the vehicle, a little ahead of it, for its physics.

  private async reportGround(): Promise<void> {
    const body = this.body();
    const pose = body?.poseAt(performance.now());
    if (!body || !pose || this.groundBusy || body instanceof Truck) return;
    this.groundBusy = true;
    try {
      const R = 6378137;
      const lat = pose.lat + ((pose.v[0] * GROUND_LEAD_S) / R) * (180 / Math.PI);
      const lon =
        pose.lon +
        ((pose.v[1] * GROUND_LEAD_S) / (R * Math.cos((pose.lat * Math.PI) / 180))) *
          (180 / Math.PI);
      const h = await this.world.groundHeight(lat, lon, [
        ...body.primitives,
        ...(this.operator?.primitives ?? []),
        ...this.dynamics.primitives,
      ]);
      if (h !== undefined) link.send({ ch: "ground", lat, lon, alt: h - mapOffset() });
    } finally {
      this.groundBusy = false;
    }
  }
}

/** Send the wind tab's wind to the physics. */
export function sendWind(gustNow = false): void {
  const w = wind();
  if (!w) return;
  link.send({
    ch: "wind",
    speed_ms: w.speed_ms,
    from_deg: w.from_deg,
    turbulence: w.turbulence,
    gusts: w.gusts,
    gust_now: gustNow,
  });
}

export { scenario };
