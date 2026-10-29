// The Chupito's model: its elevons on their hinges and its prop turning, from the pose's
// actuators (act[0] left elevon, act[1] right elevon, rad, trailing edge down +, as wingsim's;
// act[2] the prop, rad/s), as the old wing page (wing.js) moved them.
//
// The model's nodes: "propeller", turning about its own +Z, and "elevonPivotL"/"elevonPivotR",
// hinged about their own +X, trailing edge up for +. Which elevon is which side goes by where
// the node sits, not its name: glTF's right is -X, and the node named elevonPivotL is on the
// aircraft's right.

import * as C from "cesium";
import type { Animate } from "./aircraft.ts";
import { PropSpin, WING_SPIN } from "./props.ts";

export const WING_PROP_NODE = "propeller";
export const WING_ELEVON_NODES = ["elevonPivotL", "elevonPivotR"] as const;

interface Part {
  node: C.ModelNode;
  original: C.Matrix4;
}

interface Rig {
  prop: (Part & { spin: PropSpin }) | null;
  /** Each elevon with the actuator that drives it: 0 the aircraft's left, 1 its right. */
  elevons: (Part & { act: 0 | 1 })[];
}

const rigs = new WeakMap<C.Model, Rig>();
const scratch3 = new C.Matrix3();
const scratch4 = new C.Matrix4();
const scratchOut = new C.Matrix4();
const scratchPos = new C.Cartesian3();

function part(model: C.Model, name: string): Part | null {
  const node = model.getNode(name);
  return node ? { node, original: C.Matrix4.clone(node.originalMatrix) } : null;
}

/** The elevon hinge angle (about the node's +X, trailing edge up +) for wingsim's deflection
 * (trailing edge down +), rad. */
export const hingeAngle = (deflection: number): number => -deflection;

function rig(model: C.Model): Rig {
  let r = rigs.get(model);
  if (!r) {
    const prop = part(model, WING_PROP_NODE);
    const elevons: Rig["elevons"] = [];
    for (const name of WING_ELEVON_NODES) {
      const p = part(model, name);
      if (!p) continue;
      const right = C.Matrix4.getTranslation(p.original, scratchPos).x < 0;
      elevons.push({ ...p, act: right ? 1 : 0 });
    }
    r = { prop: prop ? { ...prop, spin: new PropSpin(WING_SPIN) } : null, elevons };
    if (!r.prop || r.elevons.length < 2) console.warn("wing model: missing prop or elevon nodes");
    rigs.set(model, r);
  }
  return r;
}

export const animateWing: Animate = (model, pose, dt) => {
  const { prop, elevons } = rig(model);
  if (prop) {
    const angle = prop.spin.step(pose.act[2] ?? 0, dt);
    C.Matrix4.fromRotation(C.Matrix3.fromRotationZ(angle, scratch3), scratch4);
    prop.node.matrix = C.Matrix4.multiply(prop.original, scratch4, scratchOut);
  }
  for (const e of elevons) {
    const angle = hingeAngle(pose.act[e.act] ?? 0);
    C.Matrix4.fromRotation(C.Matrix3.fromRotationX(angle, scratch3), scratch4);
    e.node.matrix = C.Matrix4.multiply(e.original, scratch4, scratchOut);
  }
};
