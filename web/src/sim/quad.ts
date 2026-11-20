// The quad as its firmware flies it, in the page: drone_duels.physics.quad.model and esc,
// line for line, around the plant in the simulator's module (SkyFlow-Dynamics' racer5in:
// web/wasm/plant.c). Those modules' docstrings have the reasons and the sources; the numbers
// here are theirs, and web/test/sim-quad.test.ts holds this to a flight the Python model flew.
//
// `step` takes one firmware frame: the motors' outputs in, the IMU and the true state out, in
// ArduPilot's frames (NED from the start pad, body FRD), as the Python model's Fdm.
//
// The ground: touching it, the quad is held level, keeping its heading, and slides to a stop;
// up and down it sits on its legs, a critically damped spring. It leaves when its thrust lifts
// it off them. Down faster than CRASH_SPEED, or on its side or back, it has crashed: its
// motors stop until a reset.
//
// Left out: hits and a payload (Shoot Down, which the web version does not have).

import {
  LocalFrame,
  type Quat,
  rotate,
  tilt,
  unrotate,
  type Vec,
  yawOf,
  yawQuat,
} from "./frames.ts";
import type { Sim } from "./module.ts";
import { seeded, Weather, type Wind } from "./weather.ts";

/** What the page's plant needs of an airframe (drone_duels.webdata.airframe_data). */
export interface Airframe {
  /** SkyFlow-Dynamics' flat parameter vector. */
  params: number[];
  mass: number;
  grav: number;
  /** The slowest a powered rotor turns, and the fastest, rad/s. */
  rotor_min: number;
  rotor_max: number;
  ct2: number;
  cq2: number;
}

/** One frame of the model: the Python model's Fdm. */
export interface Fdm {
  /** Sim time, s. */
  t: number;
  /** Body rates, FRD, rad/s. */
  gyro: Vec;
  /** Specific force, body FRD, m/s². */
  accel: Vec;
  /** NED from the start pad, m. */
  position: Vec;
  /** Body FRD to NED. */
  quat: Quat;
  /** NED, m/s. */
  velocity: Vec;
  batteryV: number;
  batteryA: number;
}

export type Phase = "ground" | "flying" | "crashed";

const N = 4;
// SkyFlow's flat state: x_W, v_W, q_wxyz, w_B, rotor speeds; W north-west-up, B forward-left-up.
const X = 0;
const V = 3;
const Q = 6;
const W = 10;
const ROTORS = 13;
/** diag(1, -1, -1): NWU ⇄ NED and FLU ⇄ FRD. */
const FLIP = [1, -1, -1] as const;

/** For each of racer5in's rotors (front left, front right, rear right, rear left), the index
 * of the ArduPilot motor (from 0: front right, rear left, front left, rear right) driving it. */
export const MOTOR_OF_ROTOR = [2, 0, 3, 1] as const;
/** For each ArduPilot motor (from 0), racer5in's rotor. */
const ROTOR_OF_MOTOR = [1, 3, 0, 2] as const;
/** ArduPilot's motor i is Betaflight's AP_FROM_BF[i] (rear right, front right, rear left,
 * front left): drone_duels.bridges.betaflight. */
export const AP_FROM_BF = [1, 2, 3, 0] as const;

/** The plant's step rate at least, Hz. */
const SUBSTEP_HZ = 1200;
/** How high the centre of mass is when the legs just touch the ground, m. */
const CONTACT_HEIGHT = 0.03;
/** The legs' natural frequency on the ground, Hz, critically damped. */
const CONTACT_HZ = 4;
/** Into the ground faster than this, m/s, and it has crashed. */
const CRASH_SPEED = 8;
/** Touching the ground tilted further than this, it is on its side or back: crashed. */
const CRASH_TILT_DEG = 75;
/** Sliding friction against the ground, as a share of its weight. */
const FRICTION = 0.5;
/** The turbulence's scale across the airframe, m: racer5in's motor-to-motor diagonal. */
const WIND_SPAN = 0.26;

// The ESCs and the pack (drone_duels.physics.quad.esc).
const PWM_MIN = 1000;
const PWM_MAX = 2000;
/** The throttle curve's k: SkyDreamer's identified value for a 5 inch racer. */
const CURVE_K = 0.5;
const CELLS = 6;
const CELL_FULL = 4.2;
/** A LiPo cell's resting voltage against the charge left. */
const LIPO_CELL: readonly (readonly [number, number])[] = [
  [0.0, 3.27],
  [0.05, 3.61],
  [0.1, 3.69],
  [0.2, 3.73],
  [0.3, 3.77],
  [0.4, 3.8],
  [0.5, 3.84],
  [0.6, 3.87],
  [0.7, 3.95],
  [0.8, 4.02],
  [0.9, 4.11],
  [1.0, 4.2],
];
const CAPACITY_MAH = 1300;
const PACK_RESISTANCE = 0.035;
const EFFICIENCY = 0.75;
const AVIONICS_W = 5;

/** A cell's resting voltage with `charge` (0 to 1) left. */
export function cellVoltage(charge: number): number {
  const c = Math.max(0, Math.min(1, charge));
  for (let i = 1; i < LIPO_CELL.length; i++) {
    const [c0, v0] = LIPO_CELL[i - 1]!;
    const [c1, v1] = LIPO_CELL[i]!;
    if (c <= c1) return v0 + ((v1 - v0) * (c - c0)) / (c1 - c0);
  }
  return CELL_FULL;
}

/** The pack, drained and sagging by the rotors' power. */
export class Battery {
  usedMah = 0;
  voltage = CELLS * CELL_FULL;
  current = 0;

  get resting(): number {
    return CELLS * cellVoltage(1 - this.usedMah / CAPACITY_MAH);
  }

  /** `dt` s at rotor speeds `speeds[from..from+4]` (rad/s). */
  step(frame: Airframe, speeds: ArrayLike<number>, from: number, dt: number): void {
    let shaft = 0;
    for (let i = 0; i < N; i++) shaft += frame.cq2 * Math.abs(speeds[from + i]!) ** 3;
    const power = shaft / EFFICIENCY + AVIONICS_W;
    const ocv = this.resting;
    const disc = ocv * ocv - 4 * PACK_RESISTANCE * power;
    this.voltage = disc > 0 ? (ocv + Math.sqrt(disc)) / 2 : ocv / 2;
    this.current = power / this.voltage;
    this.usedMah = Math.min(CAPACITY_MAH, this.usedMah + (this.current * dt) / 3.6);
  }
}

export class QuadModel {
  readonly local: LocalFrame;
  readonly weather: Weather;
  /** The ground under the quad, m above sea level. */
  groundAlt: number;
  t = 0;
  phase: Phase = "ground";
  battery = new Battery();
  /** SkyFlow's flat state. */
  s: Float64Array;
  last: Fdm | null = null;
  private readonly cmd = new Float64Array(N);

  constructor(
    private readonly sim: Sim,
    readonly frame: Airframe,
    /** The start pad: latitude, longitude, the ground's height above sea level, m. */
    readonly origin: readonly [number, number, number],
    readonly headingDeg: number,
    wind: Partial<Wind> = {},
    seed?: number,
  ) {
    if (frame.params.length !== sim.paramSize) {
      throw new Error("the airframe's parameters are not this plant's");
    }
    sim.params.set(frame.params);
    this.local = new LocalFrame(origin[0], origin[1]);
    this.weather = new Weather(wind, WIND_SPAN, seed === undefined ? Math.random : seeded(seed));
    this.groundAlt = origin[2];
    this.s = this.startState();
  }

  /** Back to the origin, still, on the ground, a fresh pack. */
  reset(): void {
    this.t = 0;
    this.groundAlt = this.origin[2];
    this.phase = "ground";
    this.battery = new Battery();
    this.s = this.startState();
    this.last = null;
  }

  /** The ground under the quad is at `alt` m above sea level. On the ground, the quad moves
   * with it; flying, it meets it where it is. */
  setGround(alt: number): void {
    if (this.phase !== "flying") this.s[X + 2]! += alt - this.groundAlt;
    this.groundAlt = alt;
  }

  setWind(wind: Partial<Wind>, gustNow = false): void {
    this.weather.update(wind, gustNow);
  }

  /** The ground's NED down from the origin, m. */
  private groundDown(): number {
    return this.origin[2] - this.groundAlt;
  }

  /** At the origin, facing the start's heading, settled on its legs. */
  private startState(): Float64Array {
    const sag = this.frame.grav / (2 * Math.PI * CONTACT_HZ) ** 2;
    const s = new Float64Array(this.sim.stateSize);
    s[X + 2] = -(this.groundDown() - CONTACT_HEIGHT + sag);
    s.set(skyflowQuat(yawQuat(this.headingDeg)), Q);
    return s;
  }

  /** protocol.POSE_FIELDS: t, lat, lon, alt, q (NED ← FRD), v NED, rates FRD, and the rotor
   * speeds of motors 1-4 (ArduPilot's quad X), rad/s. Into `out` (18 doubles). */
  pose(out: Float64Array): boolean {
    const fdm = this.last;
    if (!fdm) return false;
    const [lat, lon] = this.local.toLatLon(fdm.position[0], fdm.position[1]);
    out[0] = fdm.t;
    out[1] = lat;
    out[2] = lon;
    out[3] = this.origin[2] - fdm.position[2];
    out.set(fdm.quat, 4);
    out.set(fdm.velocity, 8);
    out.set(fdm.gyro, 11);
    for (let m = 0; m < N; m++) out[14 + m] = this.s[ROTORS + ROTOR_OF_MOTOR[m]!]!;
    return true;
  }

  /** On by `dt` s with ArduPilot's motor outputs `pwm` (µs, motor 1 first). */
  step(dt: number, pwm: ArrayLike<number>): Fdm {
    const { frame, sim } = this;
    const s = this.s;
    const substeps = Math.max(1, Math.round(dt * SUBSTEP_HZ));
    // The ESCs: each rotor's commanded speed, 0 for one unpowered.
    const top =
      frame.rotor_max * Math.min(1, Math.max(0, this.battery.voltage / (CELLS * CELL_FULL)));
    for (let rotor = 0; rotor < N; rotor++) {
      const width = this.phase === "crashed" ? 0 : (pwm[MOTOR_OF_ROTOR[rotor]!] ?? 0);
      const u = Math.min(1, Math.max(0, (width - PWM_MIN) / (PWM_MAX - PWM_MIN)));
      this.cmd[rotor] = u > 0 ? sim.throttleToSpeed(u, frame.rotor_min, top, CURVE_K) : 0;
    }

    const quat = nedQuat(s, Q);
    const height = this.groundDown() - -s[X + 2]!;
    const wind = this.weather.wind;
    const air = Math.hypot(s[V]! - wind[0], -s[V + 1]! - wind[1], -s[V + 2]! - wind[2]);
    this.weather.step(dt, Math.max(0, height), air);

    const u = sim.inputs;
    u.fill(0);
    u.set(this.cmd, 0);
    for (let i = 0; i < 3; i++) u[N + i] = FLIP[i]! * this.weather.wind[i]!;
    sim.state.set(s);
    sim.advance(dt / substeps, substeps, frame.rotor_max);
    const nxt = sim.state.slice();
    const imu = sim.imu.slice();
    this.battery.step(frame, nxt, ROTORS, dt);
    this.t += dt;

    let gyro: Vec = [imu[3]!, -imu[4]!, -imu[5]!];
    let accel: Vec = [imu[0]!, -imu[1]!, -imu[2]!];
    if (this.phase === "flying") {
      this.s = nxt;
      if (this.height(nxt) <= 0) {
        const downSpeed = -nxt[V + 2]!;
        const over = (tilt(nedQuat(nxt, Q)) * 180) / Math.PI;
        this.phase = downSpeed > CRASH_SPEED || over > CRASH_TILT_DEG ? "crashed" : "ground";
      }
    } else {
      const [held, felt] = this.contact(s, nxt, imu, quat, dt);
      if (this.phase === "ground" && this.height(held) > 0 && held[V + 2]! > 0) {
        this.phase = "flying"; // its thrust has lifted it off its legs
        this.s = nxt;
      } else {
        this.s = held;
        accel = felt;
        gyro = [0, 0, 0];
      }
    }
    const now = this.s;
    this.last = {
      t: this.t,
      gyro,
      accel,
      position: [now[X]!, -now[X + 1]!, -now[X + 2]!],
      quat: nedQuat(now, Q),
      velocity: [now[V]!, -now[V + 1]!, -now[V + 2]!],
      batteryV: this.battery.voltage,
      batteryA: this.battery.current,
    };
    return this.last;
  }

  /** How far above where its legs touch the ground it is, m (below: they give). */
  private height(s: Float64Array): number {
    return s[X + 2]! - (CONTACT_HEIGHT - this.groundDown());
  }

  /** A frame on the ground from `before` (attitude `quat`), the plant having taken it to `nxt`
   * with the IMU `imu` there: level, its heading kept, sliding to a stop, up and down on its
   * legs, the rotors as the plant turned them. The state after, and the accelerometer's
   * reading. */
  private contact(
    before: Float64Array,
    nxt: Float64Array,
    imu: Float64Array,
    quat: Quat,
    dt: number,
  ): [Float64Array, Vec] {
    const g = this.frame.grav;
    // What the thrust, drag and gravity alone do, NED, from the plant.
    const force = rotate(nedQuat(nxt, Q), [imu[0]!, -imu[1]!, -imu[2]!]);
    const freeUp = -(force[2] + g);
    // The legs: a critically damped spring, which only pushes.
    const omega = 2 * Math.PI * CONTACT_HZ;
    let h = this.height(before);
    let vUp = before[V + 2]!;
    const push = h < 0 ? Math.max(0, -omega * omega * h - 2 * omega * vUp) : 0;
    const aUp = freeUp + push;
    vUp += aUp * dt;
    h += vUp * dt;
    // Sliding friction, horizontally.
    let vn = before[V]!;
    let ve = -before[V + 1]!;
    const speed = Math.hypot(vn, ve);
    const slowed = Math.max(0, speed - FRICTION * g * dt);
    const k = speed > 0 ? slowed / speed : 0;
    const an = (vn * (k - 1)) / dt;
    const ae = (ve * (k - 1)) / dt;
    vn *= k;
    ve *= k;
    const level = yawQuat((yawOf(quat) * 180) / Math.PI);
    const held = before.slice();
    held[X] = before[X]! + vn * dt;
    held[X + 1] = before[X + 1]! - ve * dt;
    held[X + 2] = CONTACT_HEIGHT - this.groundDown() + h;
    held[V] = vn;
    held[V + 1] = -ve;
    held[V + 2] = vUp;
    held.set(skyflowQuat(level), Q);
    held.fill(0, W, W + 3);
    for (let i = 0; i < N; i++) held[ROTORS + i] = nxt[ROTORS + i]!;
    return [held, unrotate(level, [an, ae, -aUp - g])];
  }
}

/** q (W ← B), SkyFlow's, at `s[at..]`, as q (NED ← FRD). */
function nedQuat(s: Float64Array, at: number): Quat {
  return [s[at]!, s[at + 1]!, -s[at + 2]!, -s[at + 3]!];
}

/** q (NED ← FRD) as q (W ← B). */
function skyflowQuat(q: Quat): Quat {
  return [q[0], q[1], -q[2], -q[3]];
}
