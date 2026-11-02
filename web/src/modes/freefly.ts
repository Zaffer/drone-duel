// Free Fly (DESIGN.md §7): any vehicle, switched at will (the Vehicle tab), no score. Practise
// ArduPilot missions here: plan them in QGroundControl, watch them in the Map view.

import type { ModeDef } from "./types.ts";

export const freefly: ModeDef = {
  id: "freefly",
  name: "Free Fly",
  desc: "Fly or drive anywhere. Switch vehicles at will.",
  vehicles: (all) => all,
  run: () => ({ tick() {}, marks: () => [], dispose() {} }),
};
