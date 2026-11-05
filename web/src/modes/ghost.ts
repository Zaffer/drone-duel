// The other player's vehicle, as their page sends it (DESIGN.md §7): on a timeline by the time
// each pose carries (vehicles/timeline.ts), drawn a jitter buffer behind (the lateness of the
// last poses, 60 to 250 ms), between the two poses either side, so 30 Hz poses over the internet,
// late and lost, move smoothly. The timeline learns the sender's clock from the poses themselves.
// Poses travel with map heights (above the ellipsoid), so neither page needs the other's
// sea-level offset.

import * as C from "cesium";
import type { Vehicle } from "../net/protocol.ts";
import { type Sample, Timeline } from "../vehicles/timeline.ts";
import { modelMatrix, type Quat } from "../world/geo.ts";
import type { World } from "../world/world.ts";

/** A pose as peers send it: lat, lon, map height (m above the ellipsoid), body→NED quaternion,
 * NED velocity, and the sender's clock when it was there (ms since the epoch). */
export interface PeerPose {
  lat: number;
  lon: number;
  h: number;
  q: Quat;
  v: [number, number, number];
  t: number;
}

/** No pose for this long: the ghost is hidden. */
const STALE_MS = 2000;

export class Ghost {
  model: C.Model | null = null;
  private readonly timeline = new Timeline<Sample>();
  private lastArrived = 0;
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
    const now = performance.now();
    this.lastArrived = now;
    this.timeline.push({ t: p.t / 1000, lat: p.lat, lon: p.lon, alt: p.h, q: p.q, v: p.v }, now);
  }

  /** The ghost's pose at `now` (performance.now()), a jitter buffer in the past; null when
   * stale. */
  poseNow(now = performance.now()): PeerPose | null {
    if (!this.timeline.newest || now - this.lastArrived > STALE_MS) return null;
    const s = this.timeline.sample(now, this.timeline.buffer(now));
    return s ? { lat: s.lat, lon: s.lon, h: s.alt, q: s.q, v: s.v, t: s.t * 1000 } : null;
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
