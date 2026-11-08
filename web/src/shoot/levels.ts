// Shoot Down's levels (DESIGN.md §7.1): how the drone flies at each. On ArduCopter the page flies
// it, sending a target a few times a second (protocol.TargetMsg "go"): a point on a path round
// where it hovers, with the path's velocity there as the feed-forward, so the position
// controller follows the path rather than chasing a point. ArduCopter's own limits (its WP_ACC,
// its lean angle) shape what it actually flies, as they would a real one's. On Betaflight the
// path is a mission of waypoints along it (`pathWaypoints`, TargetMsg "path"), sent once;
// Betaflight's autopilot flies it round and round at its own speed (quad5.cli's
// ap_max_velocity), cutting its corners.
//
// The paths are in the place's local frame (east, north, up), about the hover point, laid out
// by the shooter's view of it: `across` is to their right, `away` away from them. Each starts at
// the hover point and eases up to its speed over RAMP_S (from the hover at full speed, the
// position controller lagged and then overshot: 8.9 m/s on the 6 m/s line, measured 2026-11-07).

import type { Vec3 } from "../world/geo.ts";

export type LevelId = "hover" | "line" | "circle" | "eight";

/** How a firmware flies a path: on targets the page sends (ArduCopter), as a mission
 * (Betaflight), or not at all (it only hovers). */
export type Flies = "targets" | "mission" | "hover";

export interface Level {
  id: LevelId;
  name: string;
  brief: string;
  /** What it needs: a firmware that flies a path (on targets or as a mission). */
  needs: "nothing" | "path";
}

/** The line's half length, m, and top speed, m/s (EST: a quad crossing a field). */
const LINE_M = 25;
const LINE_MS = 6;
/** The circle's radius, m, and speed, m/s. */
const CIRCLE_M = 20;
const CIRCLE_MS = 7;
/** The figure eight's half width, m, and top speed (where its loops cross), m/s. */
const EIGHT_M = 25;
const EIGHT_MS = 8;
/** A path comes up to its speed over this long, s. */
const RAMP_S = 4;

export const LEVELS: readonly Level[] = [
  { id: "hover", name: "Hover", brief: "It holds over the pad.", needs: "nothing" },
  {
    id: "line",
    name: "Back and forth",
    brief: `Side to side across your view, ${2 * LINE_M} m.`,
    needs: "path",
  },
  {
    id: "circle",
    name: "Circle",
    brief: `A ${2 * CIRCLE_M} m circle.`,
    needs: "path",
  },
  {
    id: "eight",
    name: "Figure eight",
    brief: `A figure eight ${2 * EIGHT_M} m across.`,
    needs: "path",
  },
];

/** Where the drone should be `t` s into a path level, and going how fast: local frame. */
export interface PathPoint {
  at: Vec3;
  vel: Vec3;
}

/** The path's point `t` s in: about `hover`, `across` and `away` unit vectors (horizontal). */
export function pathAt(
  id: Exclude<LevelId, "hover">,
  t: number,
  hover: Vec3,
  across: Vec3,
  away: Vec3,
): PathPoint {
  // The path's own clock: easing in, tau = t² / 2T, its rate t / T; then t - T/2.
  const r = Math.min(1, t / RAMP_S);
  t = t < RAMP_S ? (t * t) / (2 * RAMP_S) : t - RAMP_S / 2;
  // Offsets along across and away, and their rates.
  let x = 0;
  let y = 0;
  let vx = 0;
  let vy = 0;
  if (id === "line") {
    const w = LINE_MS / LINE_M;
    x = LINE_M * Math.sin(w * t);
    vx = LINE_MS * Math.cos(w * t);
  } else if (id === "circle") {
    // Round a centre CIRCLE_M beyond the hover point, through it, first to the right.
    const w = CIRCLE_MS / CIRCLE_M;
    x = CIRCLE_M * Math.sin(w * t);
    y = CIRCLE_M * (1 - Math.cos(w * t));
    vx = CIRCLE_MS * Math.cos(w * t);
    vy = CIRCLE_MS * Math.sin(w * t);
  } else {
    // Gerono's lemniscate: x = A sin θ, y = A sin θ cos θ; fastest, A ω √2, at the crossing.
    const w = EIGHT_MS / (EIGHT_M * Math.SQRT2);
    x = EIGHT_M * Math.sin(w * t);
    y = (EIGHT_M / 2) * Math.sin(2 * w * t);
    vx = EIGHT_M * w * Math.cos(w * t);
    vy = EIGHT_M * w * Math.cos(2 * w * t);
  }
  return {
    at: [hover[0] + across[0] * x + away[0] * y, hover[1] + across[1] * x + away[1] * y, hover[2]],
    vel: [r * (across[0] * vx + away[0] * vy), r * (across[1] * vx + away[1] * vy), 0],
  };
}

/** A mission's waypoints, at most (protocol.PATH_POINTS: Betaflight's). */
export const PATH_POINTS = 30;
/** Waypoints a lap: the circle's and the figure eight's. */
const CIRCLE_POINTS = 10;
const EIGHT_POINTS = 15;

/** The path as a mission of waypoints round `hover` (the same shapes as `pathAt`, laid out the
 * same way), PATH_POINTS of them: whole laps, so that only the last leg, where the firmware
 * slows to stop, comes round once a mission. The line goes end to end; the circle and the
 * eight start one step on from the hover point. */
export function pathWaypoints(
  id: Exclude<LevelId, "hover">,
  hover: Vec3,
  across: Vec3,
  away: Vec3,
): Vec3[] {
  const out: Vec3[] = [];
  const at = (x: number, y: number): Vec3 => [
    hover[0] + across[0] * x + away[0] * y,
    hover[1] + across[1] * x + away[1] * y,
    hover[2],
  ];
  for (let k = 1; k <= PATH_POINTS; k++) {
    if (id === "line") {
      out.push(at(k % 2 === 1 ? LINE_M : -LINE_M, 0));
    } else if (id === "circle") {
      const th = (2 * Math.PI * k) / CIRCLE_POINTS;
      out.push(at(CIRCLE_M * Math.sin(th), CIRCLE_M * (1 - Math.cos(th))));
    } else {
      const th = (2 * Math.PI * k) / EIGHT_POINTS;
      out.push(at(EIGHT_M * Math.sin(th), (EIGHT_M / 2) * Math.sin(2 * th)));
    }
  }
  return out;
}

/** The levels a drone can fly: paths need a firmware that flies one (on targets or as a
 * mission). */
export function levelsFor(flies: Flies): Level[] {
  return LEVELS.filter((l) => l.needs === "nothing" || flies !== "hover");
}
