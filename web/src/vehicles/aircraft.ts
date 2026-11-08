// An aircraft flown by its firmware on the server's physics: its glTF model, placed each frame at
// the true pose (link.poses()) on the physics' own clock, from its timeline (DESIGN.md §3.1). A
// model-specific `animate` moves its parts (rotors, elevons) from the pose's actuators. A payload
// (the vehicle's, content's PayloadSpec) is drawn under it, a tube along it; once the physics
// says it has gone off, neither is drawn (the blast is the mode's to draw).

import * as C from "cesium";
import type { Vehicle } from "../net/protocol.ts";
import { link, type Pose } from "../net/server.ts";
import { damage } from "../state/session.ts";
import { modelMatrix, toFixed } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { Timeline } from "./timeline.ts";
import type { Body } from "./types.ts";

export type Animate = (model: C.Model, pose: Pose, dt: number) => void;

export class Aircraft implements Body {
  model: C.Model | null = null;
  /** The poses by sim time. */
  readonly timeline = new Timeline<Pose>();
  private readonly abort = new AbortController();
  private readonly matrix = new C.Matrix4();
  private visible = true;
  private lastDraw = 0;
  private payload: { primitive: C.Primitive; offset: C.Matrix4; matrix: C.Matrix4 } | null = null;

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
        world.add(model);
      })
      .catch((err: unknown) => console.error(`model ${spec.model}`, err));
    if (spec.payload) this.payload = this.makePayload(spec.payload);
  }

  /** A payload's tube: olive drab, along the body, its axis `below_m` under the pose's point. */
  private makePayload(p: NonNullable<Vehicle["payload"]>) {
    const [length, diameter] = p.size_m;
    const geometry = new C.CylinderGeometry({
      length,
      topRadius: diameter / 2,
      bottomRadius: diameter / 2,
      slices: 20,
    });
    const primitive = new C.Primitive({
      geometryInstances: new C.GeometryInstance({
        geometry,
        attributes: {
          color: C.ColorGeometryInstanceAttribute.fromColor(C.Color.fromCssColorString("#4b5320")),
        },
      }),
      appearance: new C.PerInstanceColorAppearance({ closed: true }),
      asynchronous: false,
      show: false,
    });
    this.world.add(primitive);
    // The model's frame is forward, left, up: under it is -z; the cylinder's axis (its z) is
    // turned onto forward.
    const offset = C.Matrix4.multiply(
      C.Matrix4.fromTranslation(new C.Cartesian3(0, 0, -p.below_m)),
      C.Matrix4.fromRotation(C.Matrix3.fromRotationY(Math.PI / 2)),
      new C.Matrix4(),
    );
    return { primitive, offset, matrix: new C.Matrix4() };
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
    const shown = this.visible && !damage()?.exploded;
    model.show = shown;
    modelMatrix(toFixed(pose.lat, pose.lon, pose.alt), pose.q, this.matrix);
    model.modelMatrix = this.matrix;
    const p = this.payload;
    if (p) {
      p.primitive.show = shown;
      p.primitive.modelMatrix = C.Matrix4.multiply(this.matrix, p.offset, p.matrix);
    }
    const dt = this.lastDraw ? (now - this.lastDraw) / 1000 : 0;
    this.lastDraw = now;
    if (this.animate && model.ready) this.animate(model, pose, dt);
  }

  setVisible(on: boolean): void {
    this.visible = on;
    if (this.model) this.model.show = on && this.timeline.newest !== null;
    if (this.payload && !on) this.payload.primitive.show = false;
  }

  dispose(): void {
    this.abort.abort();
    if (this.model) this.world.remove(this.model);
    this.model = null;
    if (this.payload) this.world.remove(this.payload.primitive);
    this.payload = null;
  }
}
