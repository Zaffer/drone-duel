// The TBS Chupito's flight model: Nanawing's "v2" model (narenana's sim.narenana.com), ported
// as it is for a first demo (wing.js). Nanawing's source is not public; this follows its
// deployed bundle, step for step.
//
// It is an arcade model, not a rigid body. The forces move the aircraft: lift along the body's
// up, drag against the velocity, thrust along the nose, gravity. The sticks set the body's rates
// directly, as a flight controller in rate mode would, lagged a little and weakened below
// `vRef`. Weathervaning, a pull back to trim, and a turn assist that swings the velocity round
// in a bank are added to the rates. No inertia, moments or wind.
//
// Nanawing's frame, kept so the port reads against it: x east, y up, z north (Babylon's,
// left-handed). The body: +x the right wing, +y up, +z the nose. Quaternions [x, y, z, w] turn
// the body into the world. Angles in radians, rates in rad/s.

const DEG = Math.PI / 180;

/** Nanawing's Chupito (its flightConfig: shared defaults, then the Chupito's own). */
export const CHUPITO = {
  mass: 0.7,
  gravity: 9.81,
  airDensity: 1.225,
  wingArea: 0.132,
  wingSpan: 0.8,
  CL0: 0.15,
  CLalpha: 4.8,
  CLmax: 0.9,
  alphaStall: 16 * DEG,
  /** Past the stall, lift falls as exp(-stallSharpness * radians past it). */
  stallSharpness: 5,
  CD0: 0.12,
  inducedDragK: 0.09,
  /** Newtons, whatever the airspeed. */
  maxThrust: 14.71,
  /** Throttle travel per second. */
  throttleSlew: 3,
  pitchRateMax: 380 * DEG,
  rollRateMax: 360 * DEG,
  /** Full stick authority from this airspeed, m/s; `authMin` of it when slower. */
  vRef: 10,
  authMin: 0.25,
  /** Seconds the body's rates take to follow the sticks. */
  pitchResponse: 0.08,
  rollResponse: 0.07,
  yawStability: 4.2,
  pitchStability: 1.4,
  trimAoA: -0.5 * DEG,
  turnCoordination: 0.6,
  stickExpo: 0.05,
  /** Pitch rate fades to nothing over `alphaLimitBand` up to these angles of attack. */
  alphaLimit: 13 * DEG,
  alphaLimitNeg: -14.5 * DEG,
  alphaLimitBand: 6 * DEG,
  /** Keys ramp a stick in `rampSeconds`, let it go in `releaseSeconds`, then add `expo`. */
  keyboard: {
    pitch: { rampSeconds: 0.2, releaseSeconds: 0.08, expo: 0.25 },
    roll: { rampSeconds: 0.45, releaseSeconds: 0.08, expo: 0.65 },
  },
  /** Thrown from this high, this fast, climbing at `launchPitch` with the nose `launchAoA` above. */
  launchSpeed: 12.5,
  launchPitch: 8 * DEG,
  launchAoA: 7 * DEG,
  launchHeight: 2.2,
  launchThrottle: 0.65,
  groundClearance: 0.08,
  /** A touchdown inside all of these is a landing; else a crash. */
  landingMaxDescent: 1.3,
  landingMaxGroundSpeed: 14,
  landingMaxBank: 8 * DEG,
  landingMaxPitch: 12 * DEG,
  landingMaxPitchDown: 4 * DEG,
};

/** Nanawing's physics step. */
export const STEP_S = 1 / 120;

const RIGHT = [1, 0, 0];
const UP = [0, 1, 0];
const NOSE = [0, 0, 1];
/** The turn assist works fully with the wing's up this far from vertical, not at all past 85°. */
const COS_ASSIST_FULL = Math.cos(60 * DEG);
const COS_ASSIST_NONE = Math.cos(85 * DEG);
/** It stops holding the climb or sink as the wing's load reaches this many g. */
const ASSIST_HOLD_LOAD = 1.5;
/** No alpha limit below this airspeed, m/s. */
const ALPHA_LIMIT_FROM = 3;
/** Slower than this times the stall speed: stalled (for the HUD). */
const STALL_WARNING = 1.25;

const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const length = (a) => Math.hypot(a[0], a[1], a[2]);

/** `v` turned by quaternion `q`. */
export function rotate(q, v) {
  const [x, y, z, w] = q;
  const ux = 2 * (y * v[2] - z * v[1]);
  const uy = 2 * (z * v[0] - x * v[2]);
  const uz = 2 * (x * v[1] - y * v[0]);
  return [
    v[0] + w * ux + y * uz - z * uy,
    v[1] + w * uy + z * ux - x * uz,
    v[2] + w * uz + x * uy - y * ux,
  ];
}

/** `q` turned further by `angle` about the body's own x, y or z (gl-matrix's quat.rotateX/Y/Z). */
function turnX(q, angle) {
  const s = Math.sin(angle / 2);
  const c = Math.cos(angle / 2);
  const [x, y, z, w] = q;
  return [x * c + w * s, y * c + z * s, z * c - y * s, w * c - x * s];
}
function turnY(q, angle) {
  const s = Math.sin(angle / 2);
  const c = Math.cos(angle / 2);
  const [x, y, z, w] = q;
  return [x * c - z * s, y * c + w * s, z * c + x * s, w * c - y * s];
}
function turnZ(q, angle) {
  const s = Math.sin(angle / 2);
  const c = Math.cos(angle / 2);
  const [x, y, z, w] = q;
  return [x * c + y * s, y * c - x * s, z * c + w * s, w * c - z * s];
}
function normalize(q) {
  const n = Math.hypot(...q) || 1;
  return q.map((v) => v / n);
}

/** Facing `heading` (radians from north, clockwise), level. */
export const headingQuat = (heading) => [0, Math.sin(heading / 2), 0, Math.cos(heading / 2)];

/** Bank: negative with the right wing down. */
export function bankOf(q) {
  return Math.atan2(rotate(q, RIGHT)[1], rotate(q, UP)[1]);
}

/** The nose's angle above the horizon. */
export function pitchOf(q) {
  return Math.asin(clamp(rotate(q, NOSE)[1], -1, 1));
}

/** The nose's heading, radians from north, clockwise. */
export function headingOf(q) {
  const nose = rotate(q, NOSE);
  return Math.atan2(nose[0], nose[2]);
}

/** The stall speed, m/s: level flight at CLmax. */
export function stallSpeed(cfg = CHUPITO) {
  return Math.sqrt((2 * cfg.mass * cfg.gravity) / (cfg.airDensity * cfg.wingArea * cfg.CLmax));
}

/** Lift coefficient at angle of attack `alpha`. */
export function liftCoefficient(alpha, cfg = CHUPITO) {
  const a = Math.abs(alpha);
  if (a <= cfg.alphaStall) return clamp(cfg.CL0 + cfg.CLalpha * alpha, -cfg.CLmax, cfg.CLmax);
  const side = alpha < 0 ? -1 : 1;
  const atStall = clamp(cfg.CL0 + cfg.CLalpha * side * cfg.alphaStall, -cfg.CLmax, cfg.CLmax);
  return atStall * Math.exp(-cfg.stallSharpness * (a - cfg.alphaStall));
}

/** Nanawing's stick shaping: (1 - expo) x + expo x³. */
export const expo = (x, e) => (1 - e) * x + e * x * x * x;

/** In the hand at `position`, facing `heading`. */
export function ready(position, heading) {
  return {
    position: [...position],
    velocity: [0, 0, 0],
    orientation: headingQuat(heading),
    rates: [0, 0, 0],
    throttle: 0,
    airspeed: 0,
    phase: "READY",
    stalled: false,
  };
}

/** Thrown from where it is held. */
export function launch(s, cfg = CHUPITO) {
  const nose = rotate(s.orientation, NOSE);
  const up = rotate(s.orientation, UP);
  const c = Math.cos(cfg.launchPitch);
  const k = Math.sin(cfg.launchPitch);
  const dir = [nose[0] * c + up[0] * k, nose[1] * c + up[1] * k, nose[2] * c + up[2] * k];
  const n = length(dir);
  const velocity = dir.map((v) => (v / n) * cfg.launchSpeed);
  return {
    ...s,
    velocity,
    orientation: normalize(turnX(s.orientation, -(cfg.launchPitch + cfg.launchAoA))),
    rates: [0, 0, 0],
    throttle: cfg.launchThrottle,
    airspeed: cfg.launchSpeed,
    phase: "FLYING",
  };
}

/**
 * One step in flight. `input`: pitch (+ nose up), roll (+ right wing down), -1..1, and throttle
 * 0..1.
 */
export function step(s, input, dt, cfg = CHUPITO) {
  const slew = cfg.throttleSlew * dt;
  const throttle = clamp(s.throttle + clamp(input.throttle - s.throttle, -slew, slew), 0, 1);
  const nose = rotate(s.orientation, NOSE);
  const up = rotate(s.orientation, UP);
  const speed = length(s.velocity);
  const alpha = Math.atan2(-dot(s.velocity, up), dot(s.velocity, nose));
  const q = 0.5 * cfg.airDensity * speed * speed;
  const cl = liftCoefficient(alpha, cfg);
  const cd = cfg.CD0 + cfg.inducedDragK * cl * cl;

  const force = up.map((v) => v * q * cfg.wingArea * cl);
  if (speed > 1e-6) for (let i = 0; i < 3; i++) force[i] -= (s.velocity[i] / speed) * q * cfg.wingArea * cd;
  for (let i = 0; i < 3; i++) force[i] += nose[i] * throttle * cfg.maxThrust;
  force[1] -= cfg.mass * cfg.gravity;
  const velocity = s.velocity.map((v, i) => v + (force[i] * dt) / cfg.mass);

  // The turn assist: in a bank, swing the velocity round as far as the lift falls short of a
  // level turn's, yaw and pitch the body with it, and hold the climb or sink.
  let yawAssist = 0;
  let pitchAssist = 0;
  const bank = bankOf(s.orientation);
  const newSpeed = length(velocity);
  if (newSpeed > 6 && Math.abs(bank) > 0.04) {
    const upright = clamp((up[1] - COS_ASSIST_NONE) / (COS_ASSIST_FULL - COS_ASSIST_NONE), 0, 1);
    const k = clamp(Math.abs(bank) / 0.15, 0, 1) * cfg.turnCoordination * upright;
    if (k > 0) {
      const b = Math.min(Math.abs(bank), 1.3);
      const load = ((q * cfg.wingArea * cl) / (cfg.mass * cfg.gravity)) * Math.cos(b);
      const shortfall = clamp(1 - load, 0, 1);
      const hold = clamp((ASSIST_HOLD_LOAD - load) / (ASSIST_HOLD_LOAD - 1), 0, 1);
      const turnRate = ((cfg.gravity * Math.tan(b)) / newSpeed) * k * shortfall;
      const side = bank < 0 ? 1 : -1;
      yawAssist = turnRate * Math.cos(b) * side;
      pitchAssist = turnRate * Math.sin(b);
      const a = turnRate * dt * side;
      const [vx, , vz] = velocity;
      velocity[0] = vx * Math.cos(a) + vz * Math.sin(a);
      velocity[2] = -vx * Math.sin(a) + vz * Math.cos(a);
      velocity[1] -= velocity[1] * k * hold * clamp(1.5 * dt, 0, 1);
    }
  }
  const position = s.position.map((p, i) => p + velocity[i] * dt);

  const authority = clamp(speed / cfg.vRef, cfg.authMin, 1);
  let pitchCmd = input.pitch * cfg.pitchRateMax * authority;
  if (speed > ALPHA_LIMIT_FROM) {
    if (pitchCmd > 0) pitchCmd *= clamp((cfg.alphaLimit - alpha) / cfg.alphaLimitBand, 0, 1);
    else if (pitchCmd < 0) pitchCmd *= clamp((alpha - cfg.alphaLimitNeg) / cfg.alphaLimitBand, 0, 1);
  }
  const rollCmd = input.roll * cfg.rollRateMax * authority;
  const pitchRate = s.rates[1] + (pitchCmd - s.rates[1]) * clamp(dt / cfg.pitchResponse, 0, 1);
  const rollRate = s.rates[0] + (rollCmd - s.rates[0]) * clamp(dt / cfg.rollResponse, 0, 1);

  // Weathervane into the sideslip; pitch towards the trim angle of attack.
  let yawStab = 0;
  let pitchStab = 0;
  if (newSpeed > 0.001) {
    const right = rotate(s.orientation, RIGHT);
    const slip = Math.asin(clamp(dot(velocity, right) / newSpeed, -1, 1));
    const aoa = Math.atan2(-dot(velocity, up), dot(velocity, nose));
    yawStab = cfg.yawStability * slip * authority;
    pitchStab = -cfg.pitchStability * (aoa - cfg.trimAoA) * authority;
  }
  const yawRate = yawStab + yawAssist;
  let o = turnX(s.orientation, -(pitchRate + pitchStab + pitchAssist) * dt);
  o = turnZ(o, -rollRate * dt);
  o = turnY(o, yawRate * dt);

  const stalled =
    speed < stallSpeed(cfg) * STALL_WARNING || (speed > ALPHA_LIMIT_FROM && alpha > cfg.alphaStall);
  return {
    position,
    velocity,
    orientation: normalize(o),
    rates: [rollRate, pitchRate, yawRate],
    throttle,
    airspeed: newSpeed,
    phase: s.phase,
    stalled,
  };
}

/** On the ground at height `groundY`: landed if gentle enough, else crashed. */
export function touchdown(s, groundY, cfg = CHUPITO) {
  const descent = -s.velocity[1];
  const groundSpeed = Math.hypot(s.velocity[0], s.velocity[2]);
  const pitch = pitchOf(s.orientation);
  const landed =
    descent <= cfg.landingMaxDescent &&
    groundSpeed <= cfg.landingMaxGroundSpeed &&
    Math.abs(bankOf(s.orientation)) <= cfg.landingMaxBank &&
    pitch <= cfg.landingMaxPitch &&
    pitch >= -cfg.landingMaxPitchDown;
  const position = [s.position[0], groundY + cfg.groundClearance, s.position[2]];
  return {
    ...s,
    position,
    velocity: [0, 0, 0],
    rates: [0, 0, 0],
    airspeed: 0,
    phase: landed ? "LANDED" : "CRASHED",
    impact: { descent, groundSpeed },
  };
}

/** Whether a step's result `next` has reached the ground at height `groundY`. */
export function reachedGround(next, groundY, dt, cfg = CHUPITO) {
  const floor = groundY + cfg.groundClearance;
  return next.position[1] <= floor || next.position[1] - Math.max(0, -next.velocity[1]) * dt <= floor;
}
