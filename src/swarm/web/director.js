// Auto camera for demos: chases the leader while the swarm moves, orbits it while it holds.
// The camera eases towards each shot, so switching between them never cuts.

const C = Cesium;

const CHASE_BACK_M = 38;
const CHASE_UP_M = 16;
const CHASE_AHEAD_M = 30;
const ORBIT_DEG_PER_S = 4;
const MOVING_MPS = 1.0;
const EASE_PER_S = 1.2;

export class Director {
  /** @param {import("./fleet.js").Fleet} fleet */
  constructor(viewer, fleet) {
    this.viewer = viewer;
    this.fleet = fleet;
    this.active = false;
    this._eye = null;
    this._look = null;
    this._orbitDeg = 0;
    this._last = performance.now();
    viewer.scene.preRender.addEventListener(() => this._frame());
  }

  start() {
    this.active = true;
    this._eye = C.Cartesian3.clone(this.viewer.camera.positionWC);
    this._look = null;
    this._orbitDeg = C.Math.toDegrees(this.viewer.camera.heading) + 180;
    this._last = performance.now();
  }

  stop() {
    this.active = false;
  }

  /** The shot for this frame: where the camera wants to be and what it looks at. */
  _shot(dt) {
    const fleet = this.fleet;
    const leader = fleet.vehicles.get(fleet.leader);
    const lp = leader?.currentPosition();
    const d = leader?.data;
    if (lp && d && d.armed && Math.hypot(d.vn, d.ve) > MOVING_MPS) {
      // The shot is about the leader, so its sensor and range line are the ones drawn.
      fleet.selected = fleet.leader;
      const h = leader.currentHeading();
      const enu = C.Transforms.eastNorthUpToFixedFrame(lp);
      const at = (e, n, u) => C.Matrix4.multiplyByPoint(enu, new C.Cartesian3(e, n, u), new C.Cartesian3());
      const fe = Math.sin(h);
      const fn = Math.cos(h);
      return {
        eye: at(-fe * CHASE_BACK_M, -fn * CHASE_BACK_M, CHASE_UP_M),
        look: at(fe * CHASE_AHEAD_M, fn * CHASE_AHEAD_M, -4),
      };
    }
    const pts = fleet.allPositions();
    if (!pts.length) return null;
    const sphere = C.BoundingSphere.fromPoints(pts);
    const r = Math.max(sphere.radius * 2.2, 90);
    this._orbitDeg += ORBIT_DEG_PER_S * dt;
    const a = C.Math.toRadians(this._orbitDeg);
    const enu = C.Transforms.eastNorthUpToFixedFrame(sphere.center);
    return {
      eye: C.Matrix4.multiplyByPoint(
        enu,
        new C.Cartesian3(r * Math.sin(a), r * Math.cos(a), r * 0.6),
        new C.Cartesian3(),
      ),
      look: sphere.center,
    };
  }

  _frame() {
    const now = performance.now();
    const dt = Math.min((now - this._last) / 1000, 0.1);
    this._last = now;
    if (!this.active) return;
    const shot = this._shot(dt);
    if (!shot) return;
    const k = 1 - Math.exp(-EASE_PER_S * dt);
    this._eye = C.Cartesian3.lerp(this._eye ?? shot.eye, shot.eye, k, new C.Cartesian3());
    this._look = C.Cartesian3.lerp(this._look ?? shot.look, shot.look, k * 1.6, new C.Cartesian3());
    const direction = C.Cartesian3.normalize(
      C.Cartesian3.subtract(this._look, this._eye, new C.Cartesian3()),
      new C.Cartesian3(),
    );
    const normal = C.Ellipsoid.WGS84.geodeticSurfaceNormal(this._eye, new C.Cartesian3());
    const right = C.Cartesian3.normalize(C.Cartesian3.cross(direction, normal, new C.Cartesian3()), new C.Cartesian3());
    const up = C.Cartesian3.cross(right, direction, new C.Cartesian3());
    this.viewer.camera.setView({ destination: this._eye, orientation: { direction, up } });
  }
}
