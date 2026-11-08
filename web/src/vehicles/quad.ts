// The 5" quad's model: its four propellers turning with the rotors (pose.act[0..3], rad/s, in
// ArduPilot's motor order). The model's propeller nodes are named by corner and turn about
// their own +Y (tools/models/fpvmodel.py); each turns the way ArduPilot's quad X spins it, as
// the old COP (fleet.js) drew it. A rotor shot away (its arm, motor or prop: the physics'
// damage) has no prop drawn.

import * as C from "cesium";
import { damage } from "../state/session.ts";
import type { Animate } from "./aircraft.ts";
import { PropSpin, QUAD_SPIN } from "./props.ts";

/** The propeller nodes in ArduPilot's motor order (M1 front right, M2 rear left, M3 front left,
 * M4 rear right), with their turn seen from above, +1 anticlockwise. */
export const QUAD_PROPS: readonly (readonly [string, 1 | -1])[] = [
  ["prop_front_right", 1],
  ["prop_rear_left", 1],
  ["prop_front_left", -1],
  ["prop_rear_right", -1],
];

interface Prop {
  node: C.ModelNode;
  original: C.Matrix4;
  turn: number;
  spin: PropSpin;
}

const rigs = new WeakMap<C.Model, (Prop | null)[]>();
const scratch3 = new C.Matrix3();
const scratch4 = new C.Matrix4();
const scratchOut = new C.Matrix4();

function rig(model: C.Model): (Prop | null)[] {
  let props = rigs.get(model);
  if (!props) {
    props = QUAD_PROPS.map(([name, turn]) => {
      const node = model.getNode(name);
      if (!node) return null;
      return {
        node,
        original: C.Matrix4.clone(node.originalMatrix),
        turn,
        spin: new PropSpin(QUAD_SPIN),
      };
    });
    if (props.every((p) => p === null)) console.warn("quad model: no propeller nodes");
    rigs.set(model, props);
  }
  return props;
}

export const animateQuad: Animate = (model, pose, dt) => {
  const props = rig(model);
  const lost = damage()?.lost;
  for (let i = 0; i < props.length; i++) {
    const p = props[i];
    if (!p) continue;
    p.node.show = !lost?.[i];
    const angle = p.spin.step(pose.act[i] ?? 0, dt);
    C.Matrix4.fromRotation(C.Matrix3.fromRotationY(p.turn * angle, scratch3), scratch4);
    p.node.matrix = C.Matrix4.multiply(p.original, scratch4, scratchOut);
  }
};
