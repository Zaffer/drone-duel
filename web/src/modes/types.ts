// A game mode, the main menu's scenario (DESIGN.md §7): what the player picks before flying, and
// what the mode adds while they fly. The menu picks the scenario, then the location, then the
// vehicle; Free Fly flies on that pick, Duel and Race meet the other player in a lobby drawn under
// the vehicles. Registered in modes/registry.ts.

import type { Flight, ModeRuntime } from "../flight.ts";
import type { Location, Vehicle } from "../net/protocol.ts";
import type { ModeId } from "../state/app.ts";

export interface ModeContext {
  location: Location;
  /** The vehicle picked in the menu (a signal: the player may pick another in the lobby). */
  vehicle: () => string;
  /** Start flying: the flight is created and handed to `attach`. */
  fly(vehicle: string, pad: number, attach: (flight: Flight) => ModeRuntime): void;
  /** Back to the menus (the mode ends). */
  quit(): void;
}

export interface ModeDef {
  id: ModeId;
  name: string;
  desc: string;
  /** Which vehicles the player can fly in this mode (the pause menu's Vehicle tab). */
  vehicles(all: Vehicle[]): Vehicle[];
  /** The vehicles the menu offers, if not all of `vehicles` (Duel: the aircraft). */
  picks?(all: Vehicle[]): Vehicle[];
  /** A line over the menu's vehicles, saying what the pick is for. */
  pickHint?: string;
  /** Where this mode can be played (Race: where there is a course); default everywhere. */
  places?(all: Location[]): Location[];
  /** Duel and Race: draw the lobby into `host`, under the vehicles; returns its teardown. */
  lobby?(host: HTMLElement, ctx: ModeContext): () => void;
  /** Free Fly: the runtime for a flight. */
  run?(flight: Flight): ModeRuntime;
}
