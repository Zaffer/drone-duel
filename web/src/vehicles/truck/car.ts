// The truck's physics: a Rapier ray-cast vehicle (a box chassis on four sprung wheel rays) in
// the local east-north-up frame, ported unchanged from the old drive (car.js). Chassis frame:
// +x forward, +y left, +z up, its origin on the ground under the middle of the truck at rest.
// No Cesium here: truck.ts draws it.

import type RAPIER from "@dimforge/rapier3d-compat";
import { uprightness, type XYZ, yawOf, yawQuat } from "./frame.ts";

type Rapier = typeof RAPIER;

export const MASS_KG = 2200;
/** The body's box: the truck's 4.9 x 2.3 m, from the axles' height to the roof. At the axles'
 * height, lower than the real truck's, so it slides before it rolls. */
export const BODY = { halfX: 2.45, halfY: 1.1, bottom: 0.45, top: 1.75 };
/** The truck's size, mirrors included (tools/models/truckmodel.py). */
export const TRUCK_SIZE_M = { length: 4.9, width: 2.3, height: 1.75 };
/** Rounded edges ride over kerbs and the spikes that photogrammetry makes of posts. */
const BODY_ROUNDING_M = 0.25;
const CENTRE_OF_MASS = { x: 0.2, y: 0, z: 0.35 };
/** Wheel centres at rest, chassis frame, measured on the model (truckmodel.py). */
const AXLES_X = { front: 1.72, rear: -1.32 };
const TRACK_HALF_M = 0.86;
const WHEEL_RADIUS_M = 0.325;
const SUSPENSION_REST_M = 0.35;
/** Spring rate per kilogram of chassis per wheel. Four springs carry it 9.81 / (4 k) down. */
const SUSPENSION_STIFFNESS = 30;
const SAG_M = 9.81 / (4 * SUSPENSION_STIFFNESS);
/** Newtons per driven wheel at full throttle; all four are driven. */
const ENGINE_N = 2600;
const REVERSE_N = 1400;
export const TOP_SPEED_MS = 33;
export const TOP_REVERSE_MS = 8;
/** Brake impulse per wheel per step, N s. */
const BRAKE = 55;
const HANDBRAKE = 120;
const ROLLING_BRAKE = 2.5;
export const STEER_MAX_RAD = 0.6;
/** Steering lock halves at this speed. */
const STEER_HALF_MS = 14;
const STEER_RATE_RAD_S = 2.2;

/** One step's inputs, the chassis' way: steer +1 to the left. */
export interface DriveInput {
  throttle: number;
  steer: number;
  handbrake: boolean;
}

/** The wheels' force and brake for `throttle` at `speed` m/s: throttle against the motion
 * brakes, and once stopped, drives the other way. */
export function driveForce(throttle: number, speed: number): { force: number; brake: number } {
  let force = 0;
  let brake = ROLLING_BRAKE;
  if (throttle > 0) {
    if (speed < -0.5) brake = BRAKE;
    else force = ENGINE_N * throttle * Math.max(0, 1 - speed / TOP_SPEED_MS);
  } else if (throttle < 0) {
    if (speed > 0.5) brake = BRAKE * -throttle;
    else force = REVERSE_N * throttle * Math.max(0, 1 + speed / TOP_REVERSE_MS);
  }
  return { force, brake };
}

/** The front wheels' lock at `speed` m/s, rad: halving by STEER_HALF_MS. */
export const steerLock = (speed: number): number =>
  STEER_MAX_RAD / (1 + Math.abs(speed) / STEER_HALF_MS);

export class Car {
  readonly body: RAPIER.RigidBody;
  readonly vehicle: RAPIER.DynamicRayCastVehicleController;
  /** The front wheels' angle, rad, + left. */
  steer = 0;

  constructor(
    private readonly R: Rapier,
    world: RAPIER.World,
  ) {
    const body = world.createRigidBody(
      R.RigidBodyDesc.dynamic().setCcdEnabled(true).setCanSleep(false).setAngularDamping(0.5),
    );
    const halfZ = (BODY.top - BODY.bottom) / 2;
    const { halfX: hx, halfY: hy } = BODY;
    const inertia = {
      x: (MASS_KG / 3) * (hy * hy + halfZ * halfZ),
      y: (MASS_KG / 3) * (hx * hx + halfZ * halfZ),
      z: (MASS_KG / 3) * (hx * hx + hy * hy),
    };
    world.createCollider(
      R.ColliderDesc.roundCuboid(
        hx - BODY_ROUNDING_M,
        hy - BODY_ROUNDING_M,
        halfZ - BODY_ROUNDING_M,
        BODY_ROUNDING_M,
      )
        .setTranslation(0, 0, BODY.bottom + halfZ)
        .setMassProperties(MASS_KG, CENTRE_OF_MASS, inertia, { x: 0, y: 0, z: 0, w: 1 })
        .setFriction(0.4)
        .setRestitution(0.1),
      body,
    );
    this.body = body;

    const vehicle = world.createVehicleController(body);
    vehicle.indexUpAxis = 2;
    vehicle.setIndexForwardAxis = 0;
    // The wheel's top of travel: its rest centre, raised by the spring's sag at rest.
    const mountZ = WHEEL_RADIUS_M + SUSPENSION_REST_M - SAG_M;
    for (const x of [AXLES_X.front, AXLES_X.rear]) {
      for (const y of [TRACK_HALF_M, -TRACK_HALF_M]) {
        const down = { x: 0, y: 0, z: -1 };
        const axle = { x: 0, y: -1, z: 0 };
        vehicle.addWheel({ x, y, z: mountZ }, down, axle, SUSPENSION_REST_M, WHEEL_RADIUS_M);
        const i = vehicle.numWheels() - 1;
        vehicle.setWheelSuspensionStiffness(i, SUSPENSION_STIFFNESS);
        vehicle.setWheelSuspensionCompression(i, 4.4);
        vehicle.setWheelSuspensionRelaxation(i, 2.3);
        vehicle.setWheelMaxSuspensionTravel(i, 0.3);
        vehicle.setWheelMaxSuspensionForce(i, MASS_KG * 40);
        vehicle.setWheelFrictionSlip(i, 1.2);
        vehicle.setWheelSideFrictionStiffness(i, 1.0);
      }
    }
    this.vehicle = vehicle;
  }

  /** Put the car at rest at local `position`, heading `yaw` (radians from east, anticlockwise). */
  place(position: XYZ, yaw: number): void {
    this.body.setTranslation(position, true);
    this.body.setRotation(yawQuat(yaw), true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.steer = 0;
  }

  /** Forward speed, m/s; negative backwards. */
  get speed(): number {
    return this.vehicle.currentVehicleSpeed();
  }

  /** Heading of the nose, radians from east, anticlockwise. */
  get yaw(): number {
    return yawOf(this.body.rotation());
  }

  /** The chassis' up axis' vertical component: 1 upright, -1 on its roof. */
  get uprightness(): number {
    return uprightness(this.body.rotation());
  }

  get wheelsOnGround(): number {
    let n = 0;
    for (let i = 0; i < 4; i++) if (this.vehicle.wheelIsInContact(i)) n++;
    return n;
  }

  /** Drive inputs for one physics step of `dt` s. */
  drive(dt: number, { throttle, steer, handbrake }: DriveInput): void {
    const v = this.speed;
    const target = steer * steerLock(v);
    const step = STEER_RATE_RAD_S * dt;
    this.steer += Math.max(-step, Math.min(step, target - this.steer));
    const { force, brake } = driveForce(throttle, v);
    const vehicle = this.vehicle;
    for (let i = 0; i < 4; i++) {
      const front = i < 2;
      vehicle.setWheelSteering(i, front ? this.steer : 0);
      vehicle.setWheelEngineForce(i, force);
      vehicle.setWheelBrake(i, handbrake && !front ? HANDBRAKE : brake);
    }
    vehicle.updateVehicle(dt, this.R.QueryFilterFlags.EXCLUDE_DYNAMIC);
  }

  /** Out of `world` (the ground's, which outlives the truck). */
  remove(world: RAPIER.World): void {
    world.removeVehicleController(this.vehicle);
    world.removeRigidBody(this.body);
  }
}
