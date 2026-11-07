// The pilot in the world: a person standing just behind the start pad, facing the way the
// vehicle starts, goggles on and radio in hand (tools/models/operatormodel.py). Drawn only:
// nothing in the sim knows about them (world.add: never ground to a lookup).

import * as C from "cesium";
import { modelMatrix, moveBy, quatOf, toFixed } from "../world/geo.ts";
import type { World } from "../world/world.ts";

const MODEL = "models/operator.glb";
/** Where they stand from the pad, m: behind it, and to the left, out of the chase camera's way
 * (4 m behind) before takeoff. The spotter view is their eye (views/rigs.ts). */
const BEHIND_M = 3;
const LEFT_M = 1.5;
/** Their goggles above the ground, m. */
export const GOGGLES_M = 1.7;

/** The operator's spot from a pad facing `heading_deg`: metres east and north. */
export function pilotOffset(heading_deg: number): [number, number] {
  const h = (heading_deg * Math.PI) / 180;
  const [fwdE, fwdN] = [Math.sin(h), Math.cos(h)];
  return [-fwdE * BEHIND_M - fwdN * LEFT_M, -fwdN * BEHIND_M + fwdE * LEFT_M];
}

export interface Spot {
  lat: number;
  lon: number;
  /** Ground, m above sea level (the sim's heights). */
  alt: number;
  heading_deg: number;
}

/** Where the operator of a vehicle starting on `pad` stands (pilotOffset), facing its heading.
 * Their ground is the map's there, less geo.mapOffset. */
export async function operatorSpot(
  world: World,
  pad: { lat: number; lon: number; heading_deg: number },
  mapOffset: number,
): Promise<Spot> {
  const [east, north] = pilotOffset(pad.heading_deg);
  const [lat, lon] = moveBy(pad.lat, pad.lon, north, east);
  const alt = (await world.groundHeightSettled(lat, lon)) - mapOffset;
  return { lat, lon, alt, heading_deg: pad.heading_deg };
}

export class Operator {
  model: C.Model | null = null;
  private disposed = false;
  private visible = true;

  constructor(
    private readonly world: World,
    readonly spot: Spot,
  ) {
    const matrix = modelMatrix(
      toFixed(spot.lat, spot.lon, spot.alt),
      quatOf((spot.heading_deg * Math.PI) / 180, 0, 0),
    );
    void C.Model.fromGltfAsync({ url: MODEL, modelMatrix: matrix, minimumPixelSize: 0 })
      .then((model) => {
        if (this.disposed) return;
        this.model = model;
        model.show = this.visible;
        world.add(model);
      })
      .catch((err: unknown) => console.error(`model ${MODEL}`, err));
  }

  /** Show or hide the model (hidden when the camera is their eye). */
  setVisible(on: boolean): void {
    this.visible = on;
    if (this.model) this.model.show = on;
  }

  dispose(): void {
    this.disposed = true;
    if (this.model) this.world.remove(this.model);
    this.model = null;
  }
}
