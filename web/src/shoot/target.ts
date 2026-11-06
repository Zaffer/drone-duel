// What a bullet can hit on the 5" quad, and what it does to each part (DESIGN.md §7, Shoot
// Down). The parts are laid out as the drawn model (web/public/models/fpv-drone.glb, a stretched
// X): the shooter aims at what they see. Body frame forward, right, down, metres, from the pose's
// point (the model's origin, between the motors, under the frame).
//
// Each part is a box, a vertical cylinder (motors) or a disc (propellers). A bullet goes through
// a quad: it loses a share of its momentum in each part it meets (that share is the push the
// quad gets), and carries on. A propeller disc is mostly air: a bullet crossing it meets a blade
// with the chance the three blades cover at that radius, 3c / 2πr (they are frozen for the
// microseconds it takes: a 5" prop turns under a degree in that time).

import type { HitMsg } from "../net/protocol.ts";
import type { Vec3 } from "../world/geo.ts";

export type Part = HitMsg["part"];

/** ArduPilot's quad X: motor 1 front right, 2 rear left, 3 front left, 4 rear right. */
const CORNERS: readonly { motor: 1 | 2 | 3 | 4; fwd: number; right: number }[] = [
  { motor: 1, fwd: 1, right: 1 },
  { motor: 2, fwd: -1, right: -1 },
  { motor: 3, fwd: 1, right: -1 },
  { motor: 4, fwd: -1, right: 1 },
];

/** The model's motor hubs, m: forward and right of the centre (fpv-drone.glb's prop nodes). */
const HUB_FWD = 0.0868;
const HUB_RIGHT = 0.1051;
/** The props' radius (5"), the motor bells' (a 23 mm stator: 29 mm bell), m. */
const PROP_R = 0.0635;
const BELL_R = 0.0145;
/** The ESC sleeves' centres on the arms, m forward and right. */
const ESC_FWD = 0.05;
const ESC_RIGHT = 0.051;
/** EST: a blade's chord, m (a 5" three-blade's at mid-span). */
const CHORD = 0.012;

/** A share of the bullet's momentum each part takes from it, as it goes through (EST: from
 * what is in the way; a motor's steel and copper or a pack's cells stop far more of it than
 * carbon plate or a polycarbonate blade). */
export const MOMENTUM_SHARE: Record<Part, number> = {
  prop: 0.01,
  arm: 0.04,
  motor: 0.35,
  battery: 0.4,
  stack: 0.06,
  camera: 0.1,
  frame: 0.03,
};

type Shape =
  | { kind: "box"; min: Vec3; max: Vec3 }
  | { kind: "cyl"; at: [number, number]; r: number; down: [number, number] }
  | { kind: "disc"; at: [number, number]; r: number; inner: number; down: number };

export interface Zone {
  part: Part;
  motor: number | null;
  shape: Shape;
}

const box = (min: Vec3, max: Vec3): Shape => ({ kind: "box", min, max });

/** The arms run from the frame's sides to the motors. An arm is a thin bar: a box along it, in
 * the arm's own frame (`arm`). */
interface Arm {
  motor: number;
  from: [number, number];
  to: [number, number];
}

export const ARM_WIDTH = 0.008;
/** The arms and plates' depth, down from the top of the arms. */
const ARM_DOWN: [number, number] = [-0.005, 0];

export const ZONES: Zone[] = [
  // The pack, strapped on top.
  { part: "battery", motor: null, shape: box([-0.027, -0.012, -0.049], [0.027, 0.012, -0.022]) },
  // The flight controller and its receiver and VTX, between the plates.
  { part: "stack", motor: null, shape: box([-0.02, -0.013, -0.019], [0.02, 0.013, -0.004]) },
  // The camera at the nose.
  { part: "camera", motor: null, shape: box([0.062, -0.008, -0.02], [0.078, 0.008, -0.004]) },
  // The plates round them: top and bottom.
  { part: "frame", motor: null, shape: box([-0.075, -0.014, -0.022], [0.08, 0.014, -0.019]) },
  { part: "frame", motor: null, shape: box([-0.075, -0.014, -0.004], [0.08, 0.014, 0]) },
  ...CORNERS.flatMap((c): Zone[] => [
    {
      part: "motor",
      motor: c.motor,
      shape: {
        kind: "cyl",
        at: [c.fwd * HUB_FWD, c.right * HUB_RIGHT],
        r: BELL_R,
        down: [-0.027, -0.005],
      },
    },
    // Each motor's own ESC, in a sleeve half way along its arm (the model's): hit, that motor
    // stops.
    {
      part: "motor",
      motor: c.motor,
      shape: box(
        [c.fwd * ESC_FWD - 0.008, c.right * ESC_RIGHT - 0.008, -0.008],
        [c.fwd * ESC_FWD + 0.008, c.right * ESC_RIGHT + 0.008, 0.003],
      ),
    },
    {
      part: "prop",
      motor: c.motor,
      shape: {
        kind: "disc",
        at: [c.fwd * HUB_FWD, c.right * HUB_RIGHT],
        r: PROP_R,
        inner: BELL_R,
        down: -0.021,
      },
    },
  ]),
];

export const ARMS: Arm[] = CORNERS.map((c) => ({
  motor: c.motor,
  from: [c.fwd * 0.025, c.right * 0.014],
  to: [c.fwd * HUB_FWD, c.right * HUB_RIGHT],
}));

export interface Strike {
  part: Part;
  motor: number | null;
  /** Along the segment, 0..1. */
  s: number;
  /** Where, body frame. */
  at: Vec3;
}

/** Where the segment `a` → `b` (body frame) enters each part it meets, nearest first. Blades are
 * drawn with `rand`. */
export function strikes(a: Vec3, b: Vec3, rand: () => number = Math.random): Strike[] {
  const d: Vec3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  // Nothing within reach: a quick sphere test first.
  if (!nearSegment(a, d, 0.2)) return [];
  const out: Strike[] = [];
  for (const z of ZONES) {
    const s = enter(z.shape, a, d);
    if (s === null) continue;
    const at: Vec3 = [a[0] + d[0] * s, a[1] + d[1] * s, a[2] + d[2] * s];
    if (z.shape.kind === "disc") {
      const r = Math.hypot(at[0] - z.shape.at[0], at[1] - z.shape.at[1]);
      if (rand() > Math.min(1, (3 * CHORD) / (2 * Math.PI * r))) continue;
    }
    out.push({ part: z.part, motor: z.motor, s, at });
  }
  for (const arm of ARMS) {
    const s = enterArm(arm, a, d);
    if (s !== null) {
      out.push({
        part: "arm",
        motor: arm.motor,
        s,
        at: [a[0] + d[0] * s, a[1] + d[1] * s, a[2] + d[2] * s],
      });
    }
  }
  return out.sort((x, y) => x.s - y.s);
}

function nearSegment(a: Vec3, d: Vec3, r: number): boolean {
  const dd = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
  const t = dd > 0 ? Math.max(0, Math.min(1, -(a[0] * d[0] + a[1] * d[1] + a[2] * d[2]) / dd)) : 0;
  return Math.hypot(a[0] + d[0] * t, a[1] + d[1] * t, a[2] + d[2] * t) < r;
}

/** Where along the segment (0..1) it enters `shape`, or null. */
function enter(shape: Shape, a: Vec3, d: Vec3): number | null {
  if (shape.kind === "box") return slab(shape.min, shape.max, a, d);
  if (shape.kind === "disc") {
    if (d[2] === 0) return null;
    const s = (shape.down - a[2]) / d[2];
    if (s < 0 || s > 1) return null;
    const x = a[0] + d[0] * s - shape.at[0];
    const y = a[1] + d[1] * s - shape.at[1];
    const r = Math.hypot(x, y);
    return r <= shape.r && r >= shape.inner ? s : null;
  }
  // A vertical cylinder: its side, then its caps.
  const [cx, cy] = shape.at;
  const ox = a[0] - cx;
  const oy = a[1] - cy;
  let best: number | null = null;
  const qa = d[0] * d[0] + d[1] * d[1];
  if (qa > 0) {
    const qb = 2 * (ox * d[0] + oy * d[1]);
    const qc = ox * ox + oy * oy - shape.r * shape.r;
    const disc = qb * qb - 4 * qa * qc;
    if (disc >= 0) {
      const s = (-qb - Math.sqrt(disc)) / (2 * qa);
      const z = a[2] + d[2] * s;
      if (s >= 0 && s <= 1 && z >= shape.down[0] && z <= shape.down[1]) best = s;
    }
  }
  for (const zc of shape.down) {
    if (d[2] === 0) break;
    const s = (zc - a[2]) / d[2];
    if (s < 0 || s > 1) continue;
    if (Math.hypot(ox + d[0] * s, oy + d[1] * s) <= shape.r && (best === null || s < best))
      best = s;
  }
  // Starting inside it.
  if (
    best === null &&
    Math.hypot(ox, oy) <= shape.r &&
    a[2] >= shape.down[0] &&
    a[2] <= shape.down[1]
  ) {
    best = 0;
  }
  return best;
}

function slab(min: Vec3, max: Vec3, a: Vec3, d: Vec3): number | null {
  let lo = 0;
  let hi = 1;
  for (let i = 0; i < 3; i++) {
    if (Math.abs(d[i]!) < 1e-12) {
      if (a[i]! < min[i]! || a[i]! > max[i]!) return null;
      continue;
    }
    let t0 = (min[i]! - a[i]!) / d[i]!;
    let t1 = (max[i]! - a[i]!) / d[i]!;
    if (t0 > t1) [t0, t1] = [t1, t0];
    lo = Math.max(lo, t0);
    hi = Math.min(hi, t1);
    if (lo > hi) return null;
  }
  return lo;
}

/** An arm: a box in a frame along it (from `from` to `to`, its width across, ARM_DOWN deep). */
function enterArm(arm: Arm, a: Vec3, d: Vec3): number | null {
  const ux = arm.to[0] - arm.from[0];
  const uy = arm.to[1] - arm.from[1];
  const len = Math.hypot(ux, uy);
  const ax: [number, number] = [ux / len, uy / len];
  const across: [number, number] = [-ax[1], ax[0]];
  const local = (p: Vec3, origin: boolean): Vec3 => {
    const x = p[0] - (origin ? arm.from[0] : 0);
    const y = p[1] - (origin ? arm.from[1] : 0);
    return [x * ax[0] + y * ax[1], x * across[0] + y * across[1], p[2]];
  };
  return slab(
    [0, -ARM_WIDTH / 2, ARM_DOWN[0]],
    [len, ARM_WIDTH / 2, ARM_DOWN[1]],
    local(a, true),
    local(d, false),
  );
}
