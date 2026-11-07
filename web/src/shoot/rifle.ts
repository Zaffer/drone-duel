// The rifle in the shooter's hands, drawn in first person (DESIGN.md §7, Shoot Down):
// web/public/models/rifle.glb (tools/models/riflemodel.py), the rifle and the arms holding it,
// with the points the sight picture and the ballistics need measured off the mesh, in the
// scene's extras. Its own frame is glTF's: +Z along the bore to the muzzle, +Y up, +X the
// shooter's left, metres from the butt pad's centre. Here they are turned into the rifle's body
// frame, forward, right, down, as the rest of the game's bodies.
//
// The reload is drawn by moving nodes: the magazine (with the left hand that holds it) drops out
// along the magazine well, a full one comes up into it, the hand goes back to the handguard.

import * as C from "cesium";
import { modelMatrix, type Quat, type Vec3 } from "../world/geo.ts";
import type { World } from "../world/world.ts";

/** Where things are on the rifle, body frame (forward, right, down), m from the butt. */
export interface RifleGeometry {
  eye: Vec3;
  holoAxis: [Vec3, Vec3];
  holoWindow: Vec3[];
  frontPost: Vec3;
  rearAperture: Vec3;
  /** The eye behind the irons: on the line through the rear aperture and the front post. */
  ironEye: Vec3;
  muzzle: Vec3;
  boreAxis: [Vec3, Vec3];
  ejectionPort: Vec3;
  ejectionDir: Vec3;
  magWell: Vec3;
  magAxis: Vec3;
}

/** glTF (x left, y up, z forward) to body (forward, right, down). */
const toBody = (p: number[]): Vec3 => [p[2]!, -p[0]!, -p[1]!];

/** Until the model says otherwise: an 11.5" AR with an EOTech (EST, round numbers). */
const FALLBACK: RifleGeometry = {
  eye: [0.13, 0, -0.085],
  holoAxis: [
    [0.27, 0, -0.085],
    [0.33, 0, -0.085],
  ],
  holoWindow: [
    [0.27, -0.016, -0.1],
    [0.27, 0.016, -0.1],
    [0.27, 0.016, -0.07],
    [0.27, -0.016, -0.07],
  ],
  frontPost: [0.7, 0, -0.085],
  rearAperture: [0.2, 0, -0.085],
  ironEye: [0.13, 0, -0.085],
  muzzle: [0.84, 0, -0.02],
  boreAxis: [
    [0.35, 0, -0.02],
    [0.84, 0, -0.02],
  ],
  ejectionPort: [0.36, 0.015, -0.03],
  ejectionDir: [0, 1, 0],
  magWell: [0.38, 0, 0.02],
  magAxis: [0.25, 0, 0.97],
};

/** The GLB's JSON chunk. */
async function glbJson(url: string): Promise<Record<string, unknown>> {
  const buf = await (await fetch(url)).arrayBuffer();
  const view = new DataView(buf);
  const length = view.getUint32(12, true);
  return JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 20, length)));
}

function sub(a: Vec3, b: Vec3): Vec3 {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
}
function norm(a: Vec3): Vec3 {
  const l = Math.hypot(...a) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
}

/** The geometry from the model's extras, falling back point by point. */
export async function rifleGeometry(url: string): Promise<RifleGeometry> {
  let extras: Record<string, unknown> = {};
  try {
    const json = await glbJson(url);
    const scenes = json.scenes as { extras?: Record<string, unknown> }[] | undefined;
    extras = {
      ...((json.extras as Record<string, unknown>) ?? {}),
      ...(scenes?.[(json.scene as number) ?? 0]?.extras ?? {}),
    };
  } catch (err) {
    console.warn("rifle: no geometry in the model, using round numbers", err);
  }
  const pt = (k: string): Vec3 | null => {
    const v = extras[k];
    return Array.isArray(v) && v.length === 3 && v.every((x) => typeof x === "number")
      ? toBody(v)
      : null;
  };
  const pts = (k: string, n: number): Vec3[] | null => {
    const v = extras[k];
    return Array.isArray(v) && v.length === n && v.every((p) => Array.isArray(p) && p.length === 3)
      ? v.map((p) => toBody(p as number[]))
      : null;
  };
  const dir = (k: string): Vec3 | null => {
    const p = pt(k);
    return p ? norm(p) : null;
  };
  const g = FALLBACK;
  const holoAxis = (pts("holo_axis", 2) as [Vec3, Vec3] | null) ?? g.holoAxis;
  const frontPost = pt("front_post_tip") ?? g.frontPost;
  const rearAperture = pt("rear_aperture") ?? g.rearAperture;
  const bore = (pts("bore", 2) as [Vec3, Vec3] | null) ?? g.boreAxis;
  const eye = pt("eye") ?? g.eye;
  // The iron sights' eye: as far behind the rear aperture as the holo's eye is behind its window.
  const ironLine = norm(sub(frontPost, rearAperture));
  const back = Math.max(0.05, rearAperture[0] - eye[0]);
  const ironEye: Vec3 = [
    rearAperture[0] - ironLine[0] * back,
    rearAperture[1] - ironLine[1] * back,
    rearAperture[2] - ironLine[2] * back,
  ];
  return {
    eye,
    holoAxis,
    holoWindow: pts("holo_window", 4) ?? g.holoWindow,
    frontPost,
    rearAperture,
    ironEye,
    muzzle: pt("muzzle") ?? bore[1],
    boreAxis: bore,
    ejectionPort: pt("ejection_port") ?? g.ejectionPort,
    ejectionDir: dir("ejection_dir") ?? g.ejectionDir,
    magWell: pt("mag_well") ?? g.magWell,
    magAxis: dir("mag_axis") ?? g.magAxis,
  };
}

/** A body-frame point's distance from the line through `a` and `b`. */
export function offLine(p: Vec3, a: Vec3, b: Vec3): number {
  const d = norm(sub(b, a));
  const v = sub(p, a);
  const t = v[0] * d[0] + v[1] * d[1] + v[2] * d[2];
  return Math.hypot(v[0] - d[0] * t, v[1] - d[1] * t, v[2] - d[2] * t);
}

const RELOAD_STEPS = {
  /** The hand leaves the handguard, the magazine drops. */
  drop: [0.15, 0.45],
  /** A full one comes up from the pouch and seats. */
  insert: [0.9, 1.5],
  /** The hand back on the handguard. */
  back: 1.75,
} as const;

export class RifleModel {
  model: C.Model | null = null;
  private readonly matrix = new C.Matrix4();
  private nodes = new Map<string, { node: C.ModelNode; original: C.Matrix4 }>();
  private disposed = false;
  private sight: "holo" | "irons" = "holo";

  constructor(
    private readonly world: World,
    url: string,
    private readonly geometry: RifleGeometry,
  ) {
    void C.Model.fromGltfAsync({ url, modelMatrix: this.matrix, minimumPixelSize: 0 }).then(
      (m) => {
        if (this.disposed) {
          m.destroy();
          return;
        }
        this.model = world.add(m) as C.Model;
        m.readyEvent.addEventListener(() => {
          for (const name of [
            "rifle",
            "magazine",
            "holo",
            "arm_right",
            "arm_left_grip",
            "arm_left_mag",
          ]) {
            const node = m.getNode(name);
            if (node)
              this.nodes.set(name, { node, original: C.Matrix4.clone(node.originalMatrix) });
          }
          this.reloadPose(null);
          this.setSight(this.sight);
        });
      },
      (err) => console.warn("rifle: model failed", err),
    );
  }

  /** The butt at `butt` (earth-fixed), the rifle's attitude `q` (body→NED). */
  place(butt: C.Cartesian3, q: Quat): void {
    if (!this.model) return;
    modelMatrix(butt, q, this.matrix);
    this.model.modelMatrix = this.matrix;
  }

  setSight(sight: "holo" | "irons"): void {
    this.sight = sight;
    const holo = this.nodes.get("holo");
    if (holo) holo.node.show = sight === "holo";
  }

  setVisible(on: boolean): void {
    if (this.model) this.model.show = on;
  }

  /** The reload, `t` s into it (null: none). */
  reloadPose(t: number | null): void {
    const mag = this.nodes.get("magazine");
    const grip = this.nodes.get("arm_left_grip");
    const held = this.nodes.get("arm_left_mag");
    const out = (k: number) => {
      if (!mag) return;
      // Along the well, in glTF's frame: body (fwd, right, down) back to (left, up, fwd).
      const a = this.geometry.magAxis;
      const d = 0.35 * k;
      const move = C.Matrix4.fromTranslation(new C.Cartesian3(-a[1] * d, -a[2] * d, a[0] * d));
      mag.node.matrix = C.Matrix4.multiply(move, mag.original, new C.Matrix4());
      mag.node.show = k < 0.999;
    };
    if (t === null) {
      out(0);
      if (grip) grip.node.show = true;
      if (held) held.node.show = false;
      return;
    }
    const s = RELOAD_STEPS;
    const handOff = t >= s.drop[0] && t < s.back;
    if (grip) grip.node.show = !handOff;
    if (held) held.node.show = handOff;
    let k = 0;
    if (t < s.drop[0]) k = 0;
    else if (t < s.drop[1]) k = (t - s.drop[0]) / (s.drop[1] - s.drop[0]);
    else if (t < s.insert[0]) k = 1;
    else if (t < s.insert[1]) k = 1 - (t - s.insert[0]) / (s.insert[1] - s.insert[0]);
    out(k);
  }

  dispose(): void {
    this.disposed = true;
    if (this.model) this.world.remove(this.model);
    this.model = null;
  }
}

export const RELOAD = RELOAD_STEPS;
