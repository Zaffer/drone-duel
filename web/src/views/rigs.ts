// The cameras of the views (DESIGN.md §11), each placing Cesium's camera from the vehicle's true
// pose every frame:
//   fpv      the vehicle's own camera: on its frame, tilted up, its field of view
//   chase    behind the nose and above, level, coming round with it on a spring (follow.ts)
//   far      the same, farther back and higher
//   spotter  the pilot's eye at the start pad, turning to keep the vehicle in sight (LOS flying)
//   orbit    round the vehicle, by mouse: drag turns, wheel zooms
//   map      Cesium's own free camera (the map view moves it once, to above the vehicle)

import * as C from "cesium";
import type { CameraSpec } from "../net/protocol.ts";
import type { Pose } from "../net/server.ts";
import { GOGGLES_M, pilotOffset } from "../people/operator.ts";
import type { ViewId } from "../state/prefs.ts";
import {
  add3,
  bodyAxesEnu,
  enuToFixed,
  eulerOf,
  len3,
  norm3,
  offsetEnu,
  scale3,
  toFixed,
  type Vec3,
} from "../world/geo.ts";
import type { World } from "../world/world.ts";
import { NoseFollow } from "./follow.ts";

export interface RigContext {
  world: World;
  camera: CameraSpec;
  /** The pilot's spot: the start pad, sim height of the ground there. */
  pad: { lat: number; lon: number; alt: number; heading_deg: number };
  /** The player's chase distance and field of view (prefs). */
  chaseDistance: number;
  fovDeg: number;
  /** The vehicle's size against a 5" quad's: the chase arm grows with it. */
  scale: number;
}

export interface Rig {
  readonly id: ViewId;
  /** Whether the mouse moves Cesium's camera in this view. */
  readonly mouse: boolean;
  /** Whether the vehicle's own model is drawn (not from inside it). */
  readonly showVehicle: boolean;
  /** Whether the camera is the pilot's own eye, so the operator is not drawn. */
  readonly fromPilot?: boolean;
  enter(ctx: RigContext, pose: Pose | null): void;
  update(ctx: RigContext, pose: Pose, dt: number): void;
  leave(ctx: RigContext): void;
}

const scratchPos = new C.Cartesian3();
const scratchDir = new C.Cartesian3();
const scratchUp = new C.Cartesian3();

function frustum(ctx: RigContext): C.PerspectiveFrustum {
  return ctx.world.viewer.camera.frustum as C.PerspectiveFrustum;
}

/** Point the camera from `eye` along `dir` with `up` (all east-north-up at `eye`'s place). */
function aim(ctx: RigContext, eye: C.Cartesian3, dir: Vec3, up: Vec3): void {
  const camera = ctx.world.viewer.camera;
  enuToFixed(eye, norm3(dir), scratchDir);
  enuToFixed(eye, norm3(up), scratchUp);
  camera.setView({ destination: eye, orientation: { direction: scratchDir, up: scratchUp } });
}

function lookAt(ctx: RigContext, eye: C.Cartesian3, target: C.Cartesian3): void {
  const camera = ctx.world.viewer.camera;
  const dir = C.Cartesian3.normalize(C.Cartesian3.subtract(target, eye, scratchDir), scratchDir);
  const surfaceUp = C.Ellipsoid.WGS84.geodeticSurfaceNormal(eye, scratchUp);
  const right = C.Cartesian3.normalize(
    C.Cartesian3.cross(dir, surfaceUp, new C.Cartesian3()),
    new C.Cartesian3(),
  );
  const up = C.Cartesian3.cross(right, dir, new C.Cartesian3());
  camera.setView({ destination: eye, orientation: { direction: dir, up } });
}

const fpv: Rig = {
  id: "fpv",
  mouse: false,
  showVehicle: false,
  enter(ctx) {
    frustum(ctx).fov = C.Math.toRadians(ctx.camera.hfov_deg);
  },
  update(ctx, pose) {
    const { fwd, right, up } = bodyAxesEnu(pose.q);
    const [f, r, d] = ctx.camera.offset_m;
    const eyeOffset = add3(add3(scale3(fwd, f), scale3(right, r)), scale3(up, -d));
    const base = toFixed(pose.lat, pose.lon, pose.alt, scratchPos);
    const eye = offsetEnu(base, eyeOffset, new C.Cartesian3());
    const t = C.Math.toRadians(ctx.camera.tilt_deg);
    const dir = add3(scale3(fwd, Math.cos(t)), scale3(up, Math.sin(t)));
    const camUp = add3(scale3(fwd, -Math.sin(t)), scale3(up, Math.cos(t)));
    aim(ctx, eye, dir, camUp);
  },
  leave() {},
};

/** A chase camera `back` m behind the nose and `high` m above, level, coming round with it. */
function chase(id: ViewId, back: (ctx: RigContext) => number, high: number): Rig {
  const follow = new NoseFollow();
  return {
    id,
    mouse: false,
    showVehicle: true,
    enter(ctx) {
      frustum(ctx).fov = C.Math.toRadians(ctx.fovDeg);
      follow.reset();
    },
    update(ctx, pose, dt) {
      const heading = follow.step(pose.q, dt);
      const [east, north] = [Math.sin(heading), Math.cos(heading)];
      const target = toFixed(pose.lat, pose.lon, pose.alt, scratchPos);
      const b = back(ctx);
      const eye = offsetEnu(target, [-east * b, -north * b, high * ctx.scale], new C.Cartesian3());
      const ahead = offsetEnu(target, [east * 2, north * 2, 0.4], new C.Cartesian3());
      lookAt(ctx, eye, ahead);
    },
    leave() {},
  };
}

const spotter: Rig = {
  id: "spotter",
  mouse: false,
  showVehicle: true,
  fromPilot: true,
  enter(ctx) {
    frustum(ctx).fov = C.Math.toRadians(ctx.fovDeg);
  },
  update(ctx, pose) {
    const p = ctx.pad;
    // Where the operator stands, behind the pad: their goggles.
    const padPos = toFixed(p.lat, p.lon, p.alt, scratchPos);
    const [east, north] = pilotOffset(p.heading_deg);
    const eye = offsetEnu(padPos, [east, north, GOGGLES_M], new C.Cartesian3());
    const target = toFixed(pose.lat, pose.lon, pose.alt, new C.Cartesian3());
    // Zoom as an eye would squint: narrower the farther it is, never narrower than 15°.
    const dist = C.Cartesian3.distance(eye, target);
    frustum(ctx).fov = C.Math.toRadians(
      Math.max(15, Math.min(ctx.fovDeg, (ctx.fovDeg * 40) / Math.max(dist, 1))),
    );
    lookAt(ctx, eye, target);
  },
  leave() {},
};

/** Round the vehicle by mouse: drag turns, wheel zooms. */
function orbit(): Rig {
  let heading = 0;
  let pitch = -0.35;
  let range = 12;
  let abort: AbortController | null = null;
  return {
    id: "orbit",
    mouse: false,
    showVehicle: true,
    enter(ctx, pose) {
      frustum(ctx).fov = C.Math.toRadians(ctx.fovDeg);
      heading = pose ? eulerOf(pose.q).heading + Math.PI : 0;
      abort = new AbortController();
      const canvas = ctx.world.viewer.canvas;
      let drag: { x: number; y: number } | null = null;
      const signal = abort.signal;
      canvas.addEventListener("pointerdown", (e) => (drag = { x: e.clientX, y: e.clientY }), {
        signal,
      });
      window.addEventListener("pointerup", () => (drag = null), { signal });
      window.addEventListener(
        "pointermove",
        (e) => {
          if (!drag) return;
          heading += (e.clientX - drag.x) * 0.006;
          pitch = Math.max(-1.45, Math.min(0.6, pitch - (e.clientY - drag.y) * 0.004));
          drag = { x: e.clientX, y: e.clientY };
        },
        { signal },
      );
      canvas.addEventListener(
        "wheel",
        (e) => {
          range = Math.max(1.5, Math.min(800, range * Math.exp(e.deltaY * 0.001)));
          e.preventDefault();
        },
        { signal, passive: false },
      );
    },
    update(ctx, pose) {
      const target = toFixed(pose.lat, pose.lon, pose.alt, scratchPos);
      const off: Vec3 = [
        Math.sin(heading) * Math.cos(pitch) * range,
        Math.cos(heading) * Math.cos(pitch) * range,
        -Math.sin(pitch) * range,
      ];
      lookAt(ctx, offsetEnu(target, off, new C.Cartesian3()), target);
    },
    leave() {
      abort?.abort();
    },
  };
}

const map: Rig = {
  id: "map",
  mouse: true,
  showVehicle: true,
  enter(ctx, pose) {
    frustum(ctx).fov = C.Math.toRadians(60);
    const p = pose ?? { lat: ctx.pad.lat, lon: ctx.pad.lon, alt: ctx.pad.alt };
    const target = toFixed(p.lat, p.lon, p.alt);
    ctx.world.viewer.camera.flyToBoundingSphere(new C.BoundingSphere(target, 1), {
      offset: new C.HeadingPitchRange(0, C.Math.toRadians(-55), 350),
      duration: 0.8,
    });
  },
  update() {},
  leave() {},
};

export function makeRigs(): Record<ViewId, Rig> {
  return {
    fpv,
    chase: chase("chase", (ctx) => ctx.chaseDistance * ctx.scale, 1.4),
    far: chase("far", (ctx) => ctx.chaseDistance * ctx.scale * 3, 4.5),
    spotter,
    orbit: orbit(),
    map,
  };
}

export const VIEW_ORDER: ViewId[] = ["fpv", "chase", "far", "spotter", "map", "orbit"];
export const VIEW_NAMES: Record<ViewId, string> = {
  fpv: "FPV",
  chase: "Chase",
  far: "Far chase",
  spotter: "Spotter",
  map: "Map",
  orbit: "Orbit",
};

export { len3 };
