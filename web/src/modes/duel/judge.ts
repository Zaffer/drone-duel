// Whether an aircraft strikes the truck (DESIGN.md §7): pure maths, tested in test/judge.test.ts.
// The truck is a box round its model (4.9 × 2.3 m, 3 m tall) on the pose the driver's page sends,
// whose origin is on the ground under the middle of the truck.

import { eulerOf, neFrom, type Quat } from "../../world/geo.ts";

export const TRUCK_HALF_LENGTH = 2.45;
export const TRUCK_HALF_WIDTH = 1.15;
export const TRUCK_HEIGHT = 3.0;
/** An aircraft's own size: this close to the box counts. */
export const STRIKE_MARGIN_M = 0.5;

/** Map heights (above the ellipsoid) for both. */
export function strikes(
  truck: { lat: number; lon: number; h: number; q: Quat },
  aircraft: { lat: number; lon: number; h: number },
): boolean {
  const [n, e] = neFrom(truck.lat, truck.lon, aircraft.lat, aircraft.lon);
  const heading = eulerOf(truck.q).heading;
  const along = n * Math.cos(heading) + e * Math.sin(heading);
  const across = -n * Math.sin(heading) + e * Math.cos(heading);
  const up = aircraft.h - truck.h;
  const m = STRIKE_MARGIN_M;
  return (
    Math.abs(along) <= TRUCK_HALF_LENGTH + m &&
    Math.abs(across) <= TRUCK_HALF_WIDTH + m &&
    up >= -m &&
    up <= TRUCK_HEIGHT + m
  );
}

/** The duel's match, as the host runs it and sends it. */
export interface Match {
  phase: "live" | "swap" | "over";
  turn: number;
  turns: number;
  /** When the turn (or the swap) ends, by the host's clock (Date.now()). */
  endsAt: number;
  scores: { host: number; guest: number };
  lives: number;
  minutes: number;
}

export const pilotOf = (turn: number): "host" | "guest" => (turn % 2 === 0 ? "host" : "guest");

/** The winner of a finished match, or null for a draw. */
export function winner(m: Match): "host" | "guest" | null {
  if (m.scores.host === m.scores.guest) return null;
  return m.scores.host > m.scores.guest ? "host" : "guest";
}
