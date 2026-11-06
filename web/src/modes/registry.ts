// The game's modes, in the main menu's order (its scenarios) (DESIGN.md §7). A new mode is a ModeDef here.

import type { Location, Vehicle } from "../net/protocol.ts";
import { content, mode } from "../state/app.ts";
import { duel } from "./duel/duel.ts";
import { freefly } from "./freefly.ts";
import { race } from "./race/race.ts";
import { shoot } from "./shoot/shoot.ts";
import type { ModeDef } from "./types.ts";

export const MODES: ModeDef[] = [freefly, duel, race, shoot];

/** The vehicles the current mode lets the player fly. */
export function modeVehicles(m = MODES.find((x) => x.id === mode())): Vehicle[] {
  const all = Object.values(content()?.vehicles ?? {});
  return m ? m.vehicles(all) : all;
}

/** The vehicles the menu offers for the mode. */
export function modePicks(m: ModeDef): Vehicle[] {
  const all = Object.values(content()?.vehicles ?? {});
  return (m.picks ?? m.vehicles)(all);
}

/** Where the mode can be played. */
export function modePlaces(m: ModeDef): Location[] {
  const all = Object.values(content()?.locations ?? {});
  return m.places ? m.places(all) : all;
}
