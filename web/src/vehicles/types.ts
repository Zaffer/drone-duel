// A vehicle as the page sees it (DESIGN.md §6.3): something with a true pose, drawn on the map.
// The aircraft's pose comes from their physics on the server; the truck's from its own physics in
// the page. Views, modes and the HUD use only this.

import type * as C from "cesium";
import type { Vehicle } from "../net/protocol.ts";
import type { Pose } from "../net/server.ts";

export interface Body {
  readonly spec: Vehicle;
  /** The true pose now, extrapolated from the latest; null until the first one. Heights are the
   * sim's (above sea level); add geo.mapOffset to draw them. */
  poseAt(now: number): Pose | null;
  /** Draw it for the frame at `now` (performance.now()). */
  draw(now: number): void;
  /** Its primitives, which ground sampling must look through. */
  readonly primitives: object[];
  /** The model, once loaded (for hiding it in FPV, and its nodes). */
  readonly model: C.Model | null;
  /** Show or hide the model (hidden in its own FPV view). */
  setVisible(on: boolean): void;
  dispose(): void;
}

/** A pose `dt` seconds on, along its velocity (attitude held). */
export function extrapolate(p: Pose, dt: number): Pose {
  if (dt <= 0) return p;
  const R = 6378137;
  const d = Math.min(dt, 0.1);
  return {
    ...p,
    lat: p.lat + ((p.v[0] * d) / R) * (180 / Math.PI),
    lon: p.lon + ((p.v[1] * d) / (R * Math.cos((p.lat * Math.PI) / 180))) * (180 / Math.PI),
    alt: p.alt - p.v[2] * d,
  };
}
