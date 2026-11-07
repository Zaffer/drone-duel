// A vehicle's Body from its spec (DESIGN.md §6): the aircraft drawn from the server's physics,
// with their moving parts, or the truck with its physics in the page.

import type { Ground } from "../engine/ground.ts";
import type { Vehicle } from "../net/protocol.ts";
import type { World } from "../world/world.ts";
import { Aircraft } from "./aircraft.ts";
import { animateQuad } from "./quad.ts";
import { Truck, type TruckStart } from "./truck/truck.ts";
import type { Body } from "./types.ts";
import { animateWing } from "./wing.ts";

/** The Body for `spec`, starting at `start` (the truck's; the aircraft go where their physics
 * puts them), on the place's `ground`. The truck resolves once its physics is ready. */
export async function makeBody(
  world: World,
  spec: Vehicle,
  start: TruckStart,
  ground: Ground,
): Promise<Body> {
  switch (spec.physics.kind) {
    case "quad":
      return new Aircraft(world, spec, animateQuad);
    case "wing":
      return new Aircraft(world, spec, animateWing);
    case "browser-truck":
      return Truck.create(world, spec, start, ground);
  }
}
