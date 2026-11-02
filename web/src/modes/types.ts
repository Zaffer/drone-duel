// A game mode (DESIGN.md §7): how the player gets from a chosen location to flying, and what the
// mode adds while they fly. Free Fly flies straight away; Duel and Race meet the other player in
// a lobby first, where the vehicle is picked. Registered in modes/registry.ts.

import type { Flight, ModeRuntime } from "../flight.ts";
import type { Location, Vehicle } from "../net/protocol.ts";
import type { ModeId } from "../state/app.ts";

export interface ModeContext {
  location: Location;
  /** The vehicles the player may fly in this mode. */
  vehicles: Vehicle[];
  /** Start flying: the flight is created and handed to `attach`. */
  fly(vehicle: string, pad: number, attach: (flight: Flight) => ModeRuntime): void;
  /** Back to the menus (the mode ends). */
  quit(): void;
}

export interface ModeDef {
  id: ModeId;
  name: string;
  desc: string;
  /** Which vehicles the player can fly in this mode. */
  vehicles(all: Vehicle[]): Vehicle[];
  /** Where this mode can be played (Race: where there is a course); default everywhere. */
  places?(all: Location[]): Location[];
  /** Free Fly: fly at once. Duel and Race: draw the lobby into `host`; returns its teardown. */
  lobby?(host: HTMLElement, ctx: ModeContext): () => void;
  /** Free Fly: the runtime for a flight. */
  run?(flight: Flight): ModeRuntime;
}
