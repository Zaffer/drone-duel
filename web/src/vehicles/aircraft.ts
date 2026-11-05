// An aircraft flown by its firmware on the server's physics: its glTF model, placed each frame at
// the true pose (link.poses()) on the physics' own clock, from its timeline (DESIGN.md §3.1). A
// model-specific `animate` moves its parts (rotors, elevons) from the pose's actuators.

import * as C from "cesium";
import type { Vehicle } from "../net/protocol.ts";
import { link, type Pose } from "../net/server.ts";
import { modelMatrix, toFixed } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { Timeline } from "./timeline.ts";
import type { Body } from "./types.ts";

export type Animate = (model: C.Model, pose: Pose, dt: number) => void;

export class Aircraft implements Body {
  model: C.Model | null = null;
  readonly primitives: object[] = [];
  /** The poses by sim time. */
  readonly timeline = new Timeline<Pose>();
  private readonly abort = new AbortController();
  private readonly matrix = new C.Matrix4();
  private visible = true;
  private lastDraw = 0;

  constructor(
    private readonly world: World,
    readonly spec: Vehicle,
    private readonly animate?: Animate,
  ) {
    link.poses().subscribe(
      (p) => {
        // A NaN in a model's matrix stops Cesium rendering for good.
        if ([p.t, p.lat, p.lon, p.alt, ...p.q].every(Number.isFinite)) {
          this.timeline.push(p, p.at);
        }
      },
      { signal: this.abort.signal },
    );
    void C.Model.fromGltfAsync({ url: spec.model, modelMatrix: this.matrix, minimumPixelSize: 0 })
      .then((model) => {
        if (this.abort.signal.aborted) return;
        this.model = model;
        model.show = this.visible && this.timeline.newest !== null;
        world.scene.primitives.add(model);
        this.primitives.push(model);
      })
      .catch((err: unknown) => console.error(`model ${spec.model}`, err));
  }

  poseAt(now: number): Pose | null {
    return this.timeline.sample(now);
  }

  ahead(now: number): number {
    return this.timeline.ahead(now);
  }

  draw(now: number): void {
    const pose = this.poseAt(now);
    const model = this.model;
    if (!pose || !model) return;
    model.show = this.visible;
    modelMatrix(toFixed(pose.lat, pose.lon, pose.alt), pose.q, this.matrix);
    model.modelMatrix = this.matrix;
    const dt = this.lastDraw ? (now - this.lastDraw) / 1000 : 0;
    this.lastDraw = now;
    if (this.animate && model.ready) this.animate(model, pose, dt);
  }

  setVisible(on: boolean): void {
    this.visible = on;
    if (this.model) this.model.show = on && this.timeline.newest !== null;
  }

  dispose(): void {
    this.abort.abort();
    if (this.model) this.world.scene.primitives.remove(this.model);
    this.model = null;
  }
}
