// A vehicle as the page sees it (DESIGN.md §6.3): something with a true pose, drawn on the map.
// The aircraft's pose comes from their physics on the server; the truck's from its own physics in
// the page. Views, modes and the HUD use only this.

import type * as C from "cesium";
import type { Vehicle } from "../net/protocol.ts";
import type { Pose } from "../net/server.ts";
import { moveBy } from "../world/geo.ts";

export interface Body {
  readonly spec: Vehicle;
  /** The true pose at `now` (performance.now(), ms): the aircraft's from their timeline, the
   * truck's carried on from its latest; null until the first one. Heights are the sim's (above sea
   * level); add geo.mapOffset to draw them. */
  poseAt(now: number): Pose | null;
  /** How far past its newest pose `poseAt(now)` carries it on, s (the performance panel). */
  ahead?(now: number): number;
  /** Draw it for the frame at `now` (performance.now()). */
  draw(now: number): void;
  /** The model, once loaded (for hiding it in FPV, and its nodes). */
  readonly model: C.Model | null;
  /** Show or hide the model (hidden in its own FPV view). */
  setVisible(on: boolean): void;
  dispose(): void;
}

/** A pose `dt` seconds on, along its velocity (attitude held): the truck's, whose poses the page
 * makes itself, stamped as they are made. */
export function extrapolate(p: Pose, dt: number): Pose {
  if (dt <= 0) return p;
  const d = Math.min(dt, 0.1);
  const [lat, lon] = moveBy(p.lat, p.lon, p.v[0] * d, p.v[1] * d);
  return { ...p, lat, lon, alt: p.alt - p.v[2] * d };
}
