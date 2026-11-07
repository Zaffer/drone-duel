// The truck (DESIGN.md §6.3): the old drive (drivesession.js) behind the Body interface. Rapier
// runs it in the page, in the place's ground (engine/ground.ts): its local frame, its colliders
// made of the 3D tiles drawn near it, its physics world stepped at a fixed 60 Hz. Its pose is in the
// sim's heights (map height less geo.mapOffset), as the aircraft's are; the views ask for it
// like any other. No cameras here.
//
// It waits for ground under its start before it drops onto its wheels ("placing"), rights
// itself after lying on its side or roof a while, goes back to its last good spot if it falls
// out of the world, and, with an `outside` fence, back inside after a few seconds out.

import * as C from "cesium";
import { sounds } from "../../audio/sounds.ts";
import type { Ground, GroundStats } from "../../engine/ground.ts";
import type { Vehicle } from "../../net/protocol.ts";
import type { Pose } from "../../net/server.ts";
import { mapOffset } from "../../world/geo.ts";
import type { World } from "../../world/world.ts";
import { type Body, extrapolate } from "../types.ts";
import { Car } from "./car.ts";
import { type TruckControls, truckInput } from "./controls.ts";
import {
  bodyRates,
  chassisToNed,
  enuToNed,
  headingOfYaw,
  type Quat,
  type Vec3,
  type XYZ,
  yawOfHeading,
  yawQuat,
} from "./frame.ts";

export type { TruckControls } from "./controls.ts";

/** The truck waits for ground under it, looking this far down from this high. */
const GROUND_PROBE = { up: 60, down: 200 };
/** It rights itself after lying on its side or roof this long. */
const AUTO_RIGHT_MS = 2500;
/** Outside the fence (`outside`), it goes back inside after this long. */
const OUTSIDE_FENCE_S = 5;
/** Its level speed changing this much in a frame, m/s, is a bump. */
const BUMP_DV_MS = 2.5;
/** No bump this long after it is put down: its speed jumps to nothing. */
const BUMP_QUIET_MS = 500;
/** A fall this far below the last good spot is out of the world. */
const FALL_M = 40;

export interface TruckStart {
  lat: number;
  lon: number;
  /** Sim height, m above sea level (used only if the map has no ground there yet). */
  alt: number;
  heading_deg: number;
}

/** A spot in the local frame: x east, y north, z up (m), yaw radians from east, anticlockwise. */
interface Spot {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

interface Sim {
  car: Car;
  ground: Ground;
  localToFixed: C.Matrix4;
  /** Stops its driving on each physics step. */
  stopDriving: () => void;
}

const scratchRot = new C.Matrix3();
const scratchLocal = new C.Matrix4();
const scratchQ = new C.Quaternion();
const scratchFixed = new C.Cartesian3();
const scratchCarto = new C.Cartographic();

export class Truck implements Body {
  model: C.Model | null = null;
  /** Resolves once Rapier and the local frame are set up (it then waits for ground). */
  readonly ready: Promise<void>;
  /** A message for the player, or null to clear it ("Loading the ground…", the fence). */
  onNotice: ((text: string | null) => void) | null = null;
  /** The fence: whether a point is outside it; null for none. */
  outside: ((lat: number, lon: number) => boolean) | null = null;
  /** Times it fell out of the world and was put back. */
  falls = 0;

  private sim: Sim | null = null;
  private disposed = false;
  private visible = true;
  private readonly matrix = new C.Matrix4();
  /** Where it goes once there is ground under it; null once on its wheels. */
  private placing: Spot | null;
  private input: TruckControls = { throttle: 0, steer: 0, handbrake: false };
  private simTime = 0;
  private pose: Pose | null = null;
  private notice: string | null = null;
  private lastGood: Spot | null = null;
  private lastGoodAt = 0;
  private stuckSince: number | null = null;
  private outsideSince: number | null = null;
  private lastInside: Spot | null = null;
  private fenceSecond: number | null = null;
  private vel: { x: number; y: number } | null = null;
  private quietUntil = 0;

  constructor(
    private readonly world: World,
    readonly spec: Vehicle,
    readonly start: TruckStart,
    private readonly ground: Ground,
  ) {
    this.placing = { x: 0, y: 0, z: 0, yaw: yawOfHeading(start.heading_deg) };
    truckInput.rearm();
    this.ready = this.init();
    void C.Model.fromGltfAsync({ url: spec.model, modelMatrix: this.matrix })
      .then((model) => {
        if (this.disposed) {
          model.destroy();
          return;
        }
        this.model = model;
        model.show = false;
        world.add(model);
      })
      .catch((err: unknown) => console.error(`model ${spec.model}`, err));
  }

  /** A truck with its physics ready. */
  static async create(
    world: World,
    spec: Vehicle,
    start: TruckStart,
    ground: Ground,
  ): Promise<Truck> {
    const truck = new Truck(world, spec, start, ground);
    await truck.ready;
    return truck;
  }

  private async init(): Promise<void> {
    await this.world.ready;
    const { lat, lon, alt } = this.start;
    const groundH = (await this.world.groundHeight(lat, lon)) ?? alt + mapOffset();
    if (this.disposed) return;
    const ground = this.ground;
    const [x, y, z] = ground.frame.fromMap(lat, lon, groundH);
    this.home = { x, y, z, yaw: yawOfHeading(this.start.heading_deg) };
    if (this.placing) this.placing = { ...this.home };
    const car = new Car(ground.R, ground.physics);
    car.body.setEnabled(false);
    const stopDriving = ground.onStep((h) => this.step(h));
    this.sim = { car, ground, localToFixed: ground.frame.toFixedM, stopDriving };
    this.say("Loading the ground…");
  }

  /** Where it starts, local. */
  private home: Spot = { x: 0, y: 0, z: 0, yaw: 0 };

  /** A physics step's driving, before it. */
  private step(h: number): void {
    const sim = this.sim;
    if (!sim || this.placing) return;
    const input = this.input;
    // The chassis steers +1 to the left.
    sim.car.drive(h, { throttle: input.throttle, steer: -input.steer, handbrake: input.handbrake });
    this.simTime += h;
  }

  // -- the Body interface ----------------------------------------------------------------

  poseAt(now: number): Pose | null {
    const p = this.pose ?? this.startPose(now);
    return extrapolate(p, (now - p.at) / 1000);
  }

  draw(now: number): void {
    const sim = this.sim;
    if (!sim) return;
    const { car } = sim;
    if (this.placing) this.tryPlace(sim, this.placing, now);
    if (!this.placing) {
      const t = car.body.translation();
      sim.ground.focus("truck", [t.x, t.y, t.z]);
      this.guardFall(sim, t, now);
      this.bump(sim, now);
    }
    this.pose = this.makePose(sim, now);
    if (!this.placing) this.guardFence(sim, now);
    this.drawModel(sim);
  }

  setVisible(on: boolean): void {
    this.visible = on;
    if (this.model) this.model.show = on && this.sim !== null && !this.placing;
  }

  dispose(): void {
    this.disposed = true;
    if (this.model) {
      this.world.remove(this.model);
      this.model = null;
    }
    const sim = this.sim;
    this.sim = null;
    if (sim) {
      sim.stopDriving();
      sim.car.remove(sim.ground.physics);
      sim.ground.focus("truck", null);
    }
    this.say(null);
  }

  // -- driving ---------------------------------------------------------------------------

  /** This frame's controls (controls.ts); `reset` puts it back on its wheels. */
  setControls(c: TruckControls): void {
    this.input = c;
    if (c.reset) this.reset();
  }

  /** The controls it drives on now. */
  get controls(): TruckControls {
    return this.input;
  }

  /** Back on its wheels where it is, facing the same way. */
  reset(): void {
    const sim = this.sim;
    if (!sim || this.placing) return;
    const t = sim.car.body.translation();
    sim.car.place({ x: t.x, y: t.y, z: t.z + 1.5 }, sim.car.yaw);
    this.quiet();
  }

  /** Back to the start, once there is ground there. */
  respawn(): void {
    this.placeAt({ ...this.home });
  }

  /** On its wheels, on the ground (false while it waits for ground). */
  get placed(): boolean {
    return this.sim !== null && this.placing === null;
  }

  /** Forward speed, m/s; negative backwards. */
  get speed(): number {
    return this.placed ? this.sim!.car.speed : 0;
  }

  /** Heading of the nose, degrees true. */
  get heading(): number {
    const sim = this.sim;
    if (!sim) return this.start.heading_deg;
    return headingOfYaw(this.placing ? this.placing.yaw : sim.car.yaw);
  }

  get wheelsOnGround(): number {
    return this.placed ? this.sim!.car.wheelsOnGround : 0;
  }

  /** The ground's colliders: how many, and whether they are still settling. */
  get groundStats(): GroundStats | null {
    return this.sim?.ground.stats ?? null;
  }

  /** Draw the colliders' triangles over the map (the old drive's G key), or stop. */
  setGroundDebug(on: boolean): void {
    this.sim?.ground.setDebug(on);
  }

  // -- inside ----------------------------------------------------------------------------

  private say(text: string | null): void {
    if (text === this.notice) return;
    this.notice = text;
    this.onNotice?.(text);
  }

  /** Put it at `spot` once there is ground there. */
  private placeAt(spot: Spot): void {
    this.placing = spot;
    truckInput.rearm();
    this.sim?.car.body.setEnabled(false);
    if (this.model) this.model.show = false;
    if (this.sim) this.say("Loading the ground…");
  }

  /** Drop it onto the ground at `spot` once its colliders are built. */
  private tryPlace(sim: Sim, spot: Spot, now: number): void {
    const { x, y, yaw } = spot;
    sim.ground.focus("truck", [x, y, spot.z]);
    const hit = sim.ground.cast([x, y, spot.z + GROUND_PROBE.up], [0, 0, -1], GROUND_PROBE.down);
    // Wait for the ground to settle at its finest too, or it lands on a coarse tile.
    if (hit === null || sim.ground.settling) return;
    sim.car.place({ x, y, z: hit.point[2] + 0.4 }, yaw);
    sim.car.body.setEnabled(true);
    this.placing = null;
    this.say(null);
    this.quiet(now);
  }

  /** It has just been put down: its jump in speed is no bump. */
  private quiet(now = performance.now()): void {
    this.vel = null;
    this.quietUntil = now + BUMP_QUIET_MS;
  }

  /** Keep the last upright spot; put it back there if it falls out of the world. Right it when
   * it has lain on its side or roof a while. */
  private guardFall(sim: Sim, t: XYZ, now: number): void {
    const car = sim.car;
    const upZ = car.uprightness;
    if (upZ < 0.4 && Math.abs(car.speed) < 1) {
      this.stuckSince ??= now;
      if (now - this.stuckSince > AUTO_RIGHT_MS) {
        this.stuckSince = null;
        this.reset();
      }
    } else {
      this.stuckSince = null;
    }
    if (car.wheelsOnGround >= 3 && upZ > 0.8 && now - this.lastGoodAt > 500) {
      this.lastGood = { x: t.x, y: t.y, z: t.z, yaw: car.yaw };
      this.lastGoodAt = now;
    }
    const good = this.lastGood;
    if (good && t.z < good.z - FALL_M) {
      this.falls++;
      this.placeAt(good);
    }
  }

  /** A bump when it stops short on something. */
  private bump(sim: Sim, now: number): void {
    const v = sim.car.body.linvel();
    const prev = this.vel;
    this.vel = { x: v.x, y: v.y };
    if (!prev || now < this.quietUntil) return;
    const dv = Math.hypot(v.x - prev.x, v.y - prev.y);
    if (dv > BUMP_DV_MS) sounds.bump((dv - BUMP_DV_MS) / 8);
  }

  /** Outside the fence, it goes back in after OUTSIDE_FENCE_S, turned round. */
  private guardFence(sim: Sim, now: number): void {
    const pose = this.pose;
    if (!this.outside || !pose) return;
    const t = sim.car.body.translation();
    if (!this.outside(pose.lat, pose.lon)) {
      if (this.outsideSince !== null) this.say(null);
      this.outsideSince = null;
      this.fenceSecond = null;
      if (sim.car.wheelsOnGround >= 3)
        this.lastInside = { x: t.x, y: t.y, z: t.z, yaw: sim.car.yaw };
      return;
    }
    this.outsideSince ??= now;
    const left = OUTSIDE_FENCE_S - (now - this.outsideSince) / 1000;
    if (left > 0) {
      const second = Math.ceil(left);
      this.say(`Outside the fence: back inside in ${second} s`);
      if (second !== this.fenceSecond) sounds.fence();
      this.fenceSecond = second;
      return;
    }
    this.outsideSince = null;
    this.fenceSecond = null;
    const back = this.lastInside ?? { ...this.home, yaw: sim.car.yaw };
    this.placeAt({ ...back, yaw: back.yaw + Math.PI });
  }

  private drawModel(sim: Sim): void {
    const model = this.model;
    if (!model) return;
    model.show = this.visible && !this.placing;
    if (this.placing) return;
    const t = sim.car.body.translation();
    const q = sim.car.body.rotation();
    // Cesium turns the model's glTF front (+Z) and up (+Y) onto +x and +z, as the chassis.
    scratchQ.x = q.x;
    scratchQ.y = q.y;
    scratchQ.z = q.z;
    scratchQ.w = q.w;
    const local = C.Matrix4.fromRotationTranslation(
      C.Matrix3.fromQuaternion(scratchQ, scratchRot),
      new C.Cartesian3(t.x, t.y, t.z),
      scratchLocal,
    );
    C.Matrix4.multiply(sim.localToFixed, local, this.matrix);
    model.modelMatrix = this.matrix;
  }

  /** Its pose now, from the physics (or where it waits for ground). */
  private makePose(sim: Sim, now: number): Pose {
    const spot = this.placing;
    const body = sim.car.body;
    const t = spot ? { x: spot.x, y: spot.y, z: spot.z } : body.translation();
    const rotation = spot ? yawQuat(spot.yaw) : body.rotation();
    const zero = { x: 0, y: 0, z: 0 };
    const v = spot ? zero : body.linvel();
    const w = spot ? zero : body.angvel();
    C.Matrix4.multiplyByPoint(sim.localToFixed, new C.Cartesian3(t.x, t.y, t.z), scratchFixed);
    const carto = C.Cartographic.fromCartesian(scratchFixed, C.Ellipsoid.WGS84, scratchCarto);
    const steer = spot ? 0 : -sim.car.steer;
    return {
      t: this.simTime,
      lat: C.Math.toDegrees(carto.latitude),
      lon: C.Math.toDegrees(carto.longitude),
      alt: carto.height - mapOffset(),
      q: chassisToNed(rotation),
      v: enuToNed(v),
      rates: bodyRates(rotation, w),
      act: [spot ? 0 : sim.car.speed, steer, this.input.throttle, 0],
      at: now,
    };
  }

  /** Before the physics is up: at the start, at rest. */
  private startPose(now: number): Pose {
    const { lat, lon, alt, heading_deg } = this.start;
    const q: Quat = chassisToNed(yawQuat(yawOfHeading(heading_deg)));
    const zero: Vec3 = [0, 0, 0];
    return { t: 0, lat, lon, alt, q, v: zero, rates: zero, act: [0, 0, 0, 0], at: now };
  }
}
