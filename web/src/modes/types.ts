// A game mode (DESIGN.md §7): how the player gets from a chosen scenario to flying, and what the
// mode adds while they fly. Free Fly flies straight away; Duel and Race meet the other player in
// a lobby first. Registered in modes/registry.ts.

import type { Flight, ModeRuntime } from "../flight.ts";
import type { Scenario, Vehicle } from "../net/protocol.ts";
import type { ModeId } from "../state/app.ts";

export interface ModeContext {
  scenario: Scenario;
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
  /** Which of a scenario's vehicles the player can pick. */
  vehicles(all: Vehicle[]): Vehicle[];
  /** Whether the vehicle step comes before the lobby (Free Fly, Race) or the lobby picks it
   * (Duel, by role). */
  pickVehicle: boolean;
  /** Free Fly: fly at once. Duel and Race: draw the lobby into `host`; returns its teardown. */
  lobby?(host: HTMLElement, ctx: ModeContext, vehicle: string | null): () => void;
  /** Free Fly: the runtime for a flight. */
  run?(flight: Flight): ModeRuntime;
}
