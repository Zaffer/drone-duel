// Free Fly (DESIGN.md §7): any vehicle the scenario offers, switched at will, no score. Practise
// ArduPilot missions here: plan them in QGroundControl, watch them in the Map view.

import type { ModeDef } from "./types.ts";

export const freefly: ModeDef = {
  id: "freefly",
  name: "Free Fly",
  desc: "Fly or drive anywhere. Switch vehicles at will.",
  pickVehicle: true,
  vehicles: (all) => all,
  run: () => ({ tick() {}, marks: () => [], dispose() {} }),
};
