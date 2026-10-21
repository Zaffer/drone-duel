// The car: a Rapier ray-cast vehicle (a box chassis on four sprung wheel rays) in the local
// east-north-up frame, drawn as the COP's military truck. Chassis frame: +x forward, +y left,
// +z up, its origin on the ground under the middle of the truck at rest.

const C = Cesium;

const TRUCK_URI = "/models/military-truck.glb";
const MASS_KG = 2200;
/** The body's box: the truck's 4.9 x 2.3 m, from the axles' height to the roof. */
const BODY = { halfX: 2.45, halfY: 1.1, bottom: 0.45, top: 1.75 };
/** At the axles' height, lower than the real truck's, so it slides before it rolls. */
/** Rounded edges ride over kerbs and the spikes that photogrammetry makes of posts. */
const BODY_ROUNDING_M = 0.25;
const CENTRE_OF_MASS = { x: 0.2, y: 0, z: 0.35 };
/** Wheel centres at rest, chassis frame, measured on the model (swarm.truckmodel). */
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
const TOP_SPEED_MS = 33;
const TOP_REVERSE_MS = 8;
/** Brake impulse per wheel per step, N s. */
const BRAKE = 55;
const HANDBRAKE = 120;
const ROLLING_BRAKE = 2.5;
const STEER_MAX_RAD = 0.6;
/** Steering lock halves at this speed. */
const STEER_HALF_MS = 14;
const STEER_RATE_RAD_S = 2.2;

export class Car {
  /**
   * @param {*} R the RAPIER module
   * @param {*} world RAPIER.World in the local frame
   * @param {Cesium.Scene} scene
   * @param {Cesium.Matrix4} localToFixed the local frame's east-north-up matrix
   */
  constructor(R, world, scene, localToFixed) {
    this.R = R;
    this.world = world;
    this.localToFixed = localToFixed;
    this.steer = 0;

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

    this.modelMatrix = new C.Matrix4();
    this.model = null;
    this.scene = scene;
    this._destroyed = false;
    C.Model.fromGltfAsync({ url: TRUCK_URI, modelMatrix: this.modelMatrix }).then((model) => {
      if (this._destroyed) model.destroy();
      else this.model = scene.primitives.add(model);
    });
  }

  /** Take the truck off the map. Its body goes with the world. */
  destroy() {
    this._destroyed = true;
    if (this.model) this.scene.primitives.remove(this.model);
    this.model = null;
  }

  /** Put the car at rest at local `position` heading `heading` (radians from east, CCW). */
  place(position, heading) {
    this.body.setTranslation(position, true);
    this.body.setRotation({ x: 0, y: 0, z: Math.sin(heading / 2), w: Math.cos(heading / 2) }, true);
    this.body.setLinvel({ x: 0, y: 0, z: 0 }, true);
    this.body.setAngvel({ x: 0, y: 0, z: 0 }, true);
    this.steer = 0;
  }

  /** Forward speed, m/s; negative backwards. */
  get speed() {
    return this.vehicle.currentVehicleSpeed();
  }

  /** Heading of the nose, radians from east, CCW. */
  get heading() {
    const q = this.body.rotation();
    return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
  }

  /** The chassis' up axis' vertical component: 1 upright, -1 on its roof. */
  get uprightness() {
    const q = this.body.rotation();
    return 1 - 2 * (q.x * q.x + q.y * q.y);
  }

  get wheelsOnGround() {
    let n = 0;
    for (let i = 0; i < 4; i++) if (this.vehicle.wheelIsInContact(i)) n++;
    return n;
  }

  /**
   * Drive inputs for one physics step: throttle and steer -1..1, handbrake true/false.
   * Throttle back while moving forward brakes, then reverses once stopped.
   */
  drive(dt, { throttle, steer, handbrake }) {
    const v = this.speed;
    const lock = STEER_MAX_RAD / (1 + Math.abs(v) / STEER_HALF_MS);
    const target = steer * lock;
    const step = STEER_RATE_RAD_S * dt;
    this.steer += Math.max(-step, Math.min(step, target - this.steer));

    let force = 0;
    let brake = ROLLING_BRAKE;
    if (throttle > 0) {
      if (v < -0.5) brake = BRAKE;
      else force = ENGINE_N * throttle * Math.max(0, 1 - v / TOP_SPEED_MS);
    } else if (throttle < 0) {
      if (v > 0.5) brake = BRAKE * -throttle;
      else force = REVERSE_N * throttle * Math.max(0, 1 + v / TOP_REVERSE_MS);
    }
    const vehicle = this.vehicle;
    for (let i = 0; i < 4; i++) {
      const front = i < 2;
      vehicle.setWheelSteering(i, front ? this.steer : 0);
      vehicle.setWheelEngineForce(i, force);
      vehicle.setWheelBrake(i, handbrake && !front ? HANDBRAKE : brake);
    }
    vehicle.updateVehicle(dt, this.R.QueryFilterFlags.EXCLUDE_DYNAMIC);
  }

  /** Move the drawn truck to the body. */
  draw() {
    const t = this.body.translation();
    const q = this.body.rotation();
    // Cesium turns the model's glTF front (+Z) and up (+Y) onto +x and +z, as the chassis.
    const local = C.Matrix4.fromRotationTranslation(
      C.Matrix3.fromQuaternion(new C.Quaternion(q.x, q.y, q.z, q.w), scratchRot),
      new C.Cartesian3(t.x, t.y, t.z),
      scratchLocal,
    );
    C.Matrix4.multiply(this.localToFixed, local, this.modelMatrix);
    if (this.model) this.model.modelMatrix = this.modelMatrix;
  }
}

const scratchRot = new C.Matrix3();
const scratchLocal = new C.Matrix4();
