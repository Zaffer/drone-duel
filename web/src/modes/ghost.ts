// The other player's vehicle, as their page sends it (DESIGN.md §7): its model drawn a little in
// the past (DELAY_S), between the two poses either side, so 20 Hz updates over the internet move
// smoothly. Poses travel with map heights (above the ellipsoid), so neither page needs the other's
// sea-level offset.

import * as C from "cesium";
import type { Vehicle } from "../net/protocol.ts";
import { modelMatrix, type Quat } from "../world/geo.ts";
import type { World } from "../world/world.ts";

/** A pose as peers send it: lat, lon, map height (m above the ellipsoid), body→NED quaternion,
 * NED velocity, and the sender's clock (Date.now()). */
export interface PeerPose {
  lat: number;
  lon: number;
  h: number;
  q: Quat;
  v: [number, number, number];
  t: number;
}

const DELAY_S = 0.15;
const KEPT = 20;
/** No pose for this long: the ghost is hidden. */
const STALE_MS = 2000;

export class Ghost {
  model: C.Model | null = null;
  private poses: PeerPose[] = [];
  private readonly matrix = new C.Matrix4();
  private disposed = false;
  /** Load the model: only once there is a pose to put it at (its matrix is never empty). */
  private load: (() => void) | null;
  private warned = false;
  private label: C.LabelCollection;

  constructor(
    private readonly world: World,
    spec: Vehicle,
    colour: string,
    name: string,
    /** The sender's clock less ours (ms). */
    private readonly offset: () => number,
  ) {
    this.load = () =>
      void C.Model.fromGltfAsync({
        url: spec.model,
        modelMatrix: this.matrix,
        color: C.Color.fromCssColorString(colour),
        colorBlendMode: C.ColorBlendMode.MIX,
        colorBlendAmount: 0.35,
      }).then((m) => {
        if (this.disposed) return;
        this.model = m;
        world.scene.primitives.add(m);
      });
    // Hidden as a whole until the first pose: a collection counts even a hidden label's place
    // (here the earth's centre) in the camera's near and far planes, which breaks rendering.
    this.label = world.scene.primitives.add(new C.LabelCollection({ show: false }));
    this.label.add({
      text: name,
      font: "600 13px Inter, sans-serif",
      fillColor: C.Color.fromCssColorString(colour),
      outlineColor: C.Color.BLACK,
      outlineWidth: 3,
      style: C.LabelStyle.FILL_AND_OUTLINE,
      pixelOffset: new C.Cartesian2(0, -28),
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
      position: C.Cartesian3.ZERO,
      show: false,
    });
  }

  push(p: PeerPose): void {
    if (!finitePose(p)) {
      if (!this.warned) console.warn("ghost: a pose with a non-finite value, dropped", p);
      this.warned = true;
      return;
    }
    this.poses.push(p);
    if (this.poses.length > KEPT) this.poses.shift();
  }

  /** The ghost's pose now (in the past by DELAY_S), or null when stale. */
  poseNow(): PeerPose | null {
    const ps = this.poses;
    const last = ps[ps.length - 1];
    if (!last) return null;
    const at = Date.now() + this.offset() - DELAY_S * 1000;
    if (at - last.t > STALE_MS) return null;
    let i = ps.length - 1;
    while (i > 0 && ps[i - 1]!.t > at) i--;
    const b = ps[i]!;
    const a = ps[i - 1];
    if (!a || at >= b.t) return b;
    const k = Math.max(0, Math.min(1, (at - a.t) / (b.t - a.t || 1)));
    return {
      lat: a.lat + (b.lat - a.lat) * k,
      lon: a.lon + (b.lon - a.lon) * k,
      h: a.h + (b.h - a.h) * k,
      q: k < 0.5 ? a.q : b.q,
      v: b.v,
      t: at,
    };
  }

  draw(): PeerPose | null {
    const p = this.poseNow();
    const label = this.label.get(0);
    label.show = p !== null;
    this.label.show = p !== null;
    if (this.model) this.model.show = p !== null;
    if (!p) return null;
    const pos = C.Cartesian3.fromDegrees(p.lon, p.lat, p.h);
    modelMatrix(pos, p.q, this.matrix);
    label.position = pos;
    if (this.load) {
      this.load();
      this.load = null;
    }
    if (this.model) this.model.modelMatrix = this.matrix;
    return p;
  }

  dispose(): void {
    this.disposed = true;
    if (this.model) this.world.scene.primitives.remove(this.model);
    this.world.scene.primitives.remove(this.label);
  }
}

/** Every number in a peer's pose is finite (a NaN in a model's matrix stops Cesium rendering). */
export function finitePose(p: PeerPose): boolean {
  return (
    [p.lat, p.lon, p.h, p.t, ...(p.q ?? []), ...(p.v ?? [])].every(Number.isFinite) &&
    p.q?.length === 4 &&
    Math.abs(Math.hypot(...p.q) - 1) < 0.1
  );
}
