// A drive on the map's Google Photorealistic 3D Tiles, with physics and collisions: the truck,
// its keys, gamepad and cameras, from where it starts until destroyed. The drive page
// (drive.js) runs one; the COP runs one for the duel's driver (dueldriver.js).
//
// Rapier (WebAssembly) runs the car in a local east-north-up frame around where it starts;
// the tiles drawn near the car become its colliders (tilecolliders.js).

import { Car } from "./car.js";
import { DriveRadio, isRadio } from "./driveradio.js";
import { sfx } from "./sounds.js";
import { TileColliders } from "./tilecolliders.js";

const C = Cesium;
const STEP_S = 1 / 60;
const MAX_STEPS = 4;
/** A move further than this from the frame's origin needs a new frame (onFarMove). */
export const MAX_OFFSET_M = 2500;
/** The car waits for ground under it, looking this far down from this high. */
const GROUND_PROBE = { up: 60, down: 200 };
/** The car rights itself after lying on its side or roof this long. */
const AUTO_RIGHT_MS = 2500;
/** Outside the fence (`outside`), the car goes back inside after this long. */
const OUTSIDE_FENCE_S = 5;
/** The truck's sideways or forward speed changing this much in a frame, m/s, is a bump. */
const BUMP_DV_MS = 2.5;
/** No bump this long after the truck is put down: its speed jumps to nothing. */
const BUMP_QUIET_MS = 500;
const CAMERAS = {
  chase: { back: 9, up: 3.4, look: 1.4 },
  far: { back: 22, up: 9, look: 1 },
  hood: null,
  free: null,
};
const CAMERA_NAMES = Object.keys(CAMERAS);

let rapier = null;
/** Rapier, loaded and initialised once for the page. */
function loadRapier() {
  rapier ??= import("/vendor/rapier/dist/rapier.mjs").then(async ({ default: R }) => {
    await R.init();
    return R;
  });
  return rapier;
}

export class DriveSession {
  /**
   * Drive from `lat`, `lon`, facing `headingDeg` true. Resolves once the frame is set up; the
   * truck drops onto the ground once its colliders are built.
   * @param {import("./basemap.js").BaseMap} map a map on Google 3D tiles
   * @param {{lat: number, lon: number, headingDeg?: number, ui?: object}} start `ui`: the
   *   page's `notice(text)` and its HUD elements, each optional: speed, wheels, tiles, tris,
   *   pending, buildMs, physMs, fps, camMode, help (shown and hidden with H), radio (the
   *   radio's panel, driveradio.js)
   */
  static async start(map, { lat, lon, headingDeg = 0, ui = {} }) {
    const R = await loadRapier();
    const groundH = await map.groundHeight(lat, lon);
    return new DriveSession(map, R, C.Cartesian3.fromDegrees(lon, lat, groundH), headingDeg, ui);
  }

  constructor(map, R, origin, headingDeg, ui) {
    this.map = map;
    this.viewer = map.viewer;
    this.ui = ui;
    this.origin = origin;
    this.localToFixed = C.Transforms.eastNorthUpToFixedFrame(origin);
    this.fixedToLocal = C.Matrix4.inverseTransformation(this.localToFixed, new C.Matrix4());
    this.world = new R.World({ x: 0, y: 0, z: -9.81 });
    this.world.timestep = STEP_S;
    this.ground = new TileColliders(this.viewer, map.tileset, R, this.world, origin);
    this.car = new Car(R, this.world, this.viewer.scene, this.localToFixed);
    this.car.body.setEnabled(false);
    /** Where the car goes once there is ground under it: local {x, y}, heading, and the
     * ground's height there as far as known, z, for the camera to wait above it. */
    this.placing = { x: 0, y: 0, heading: C.Math.toRadians(90 - headingDeg), z: 0 };
    /** The last pose upright on its wheels, to reset to. */
    this.lastGood = null;
    this.falls = 0;
    /** Called each frame once the truck is drawn: (session). */
    this.onStep = null;
    /** Whether local (x, y) is outside the fence; null for no fence. */
    this.outside = null;
    /** Double-click puts the car there; one further than MAX_OFFSET_M calls this with
     * (lat, lon) instead. Null: double-click does nothing. */
    this.onFarMove = null;
    this.dblClickMoves = false;
    this.keys = new Set();
    this.radio = new DriveRadio({ panel: ui.radio, notice: (text) => this._notice(text) });
    this.cameraMode = "chase";
    this.cameraHeading = this.placing.heading;
    this._lastGoodT = 0;
    this._stuckSince = null;
    this._outsideSince = null;
    this._lastInside = null;
    this._fenceSecond = null;
    this.engine = sfx.engine();
    /** The truck's level velocity last frame, for bumps, and no bumps till then. */
    this._vel = null;
    this._quietUntil = 0;
    this._last = performance.now();
    this._acc = 0;
    this._hud = { physMs: 0, frames: 0, fpsT: this._last, fps: 0, worstMs: 0, shownWorstMs: 0 };
    this._collision = map.tileset.enableCollision;
    this._wire();
    this.setCamera("chase");
    this._notice("Loading the ground…");
  }

  /** Stop driving: the truck, its colliders, keys and camera go; the map's camera is back. */
  destroy() {
    for (const off of this._unlisten) off();
    this.engine.stop();
    this.radio.destroy();
    this._handler.destroy();
    this.ground.destroy();
    this.car.destroy();
    this.world.free();
    this.viewer.scene.screenSpaceCameraController.enableInputs = true;
    this.map.tileset.enableCollision = this._collision;
    this._notice(null);
  }

  /** The car's local (x, y, z) at `lat`, `lon`, height `h` above the ellipsoid. */
  toLocal(lat, lon, h) {
    return C.Matrix4.multiplyByPoint(this.fixedToLocal, C.Cartesian3.fromDegrees(lon, lat, h), new C.Cartesian3());
  }

  get placed() {
    return this.placing === null;
  }

  /** Put the car at local (x, y), heading radians from east, CCW, once there is ground. */
  placeAt(x, y, heading, z = 0) {
    this.placing = { x, y, heading, z };
    this.radio.rearm();
    this.car.body.setEnabled(false);
    this._notice("Loading the ground…");
  }

  setCamera(mode) {
    this.cameraMode = mode;
    this.viewer.scene.screenSpaceCameraController.enableInputs = mode === "free";
    // Cesium keeps a free camera above the tiles by reading their triangles back from the GPU
    // under the camera, every frame it moves: a tenth of the frame time. The car's cameras stay
    // out of walls on the colliders instead.
    this.map.tileset.enableCollision = mode === "free";
    if (this.ui.camMode) this.ui.camMode.textContent = mode;
  }

  _notice(text) {
    this.ui.notice?.(text);
  }

  _wire() {
    const onKeyDown = (e) => {
      if (e.repeat || e.target instanceof HTMLInputElement) return;
      this.keys.add(e.code);
      if (e.code === "KeyR") this._resetUpright();
      if (e.code === "KeyC") {
        const i = CAMERA_NAMES.indexOf(this.cameraMode);
        this.setCamera(CAMERA_NAMES[(i + 1) % CAMERA_NAMES.length]);
      }
      if (e.code === "KeyG") this.ground.setDebug(!this.ground.debug);
      if (e.code === "KeyM") sfx.toggle();
      if (e.code === "KeyH" && this.ui.help) this.ui.help.hidden = !this.ui.help.hidden;
    };
    const onKeyUp = (e) => this.keys.delete(e.code);
    const onBlur = () => this.keys.clear();
    addEventListener("keydown", onKeyDown);
    addEventListener("keyup", onKeyUp);
    addEventListener("blur", onBlur);
    this._handler = new C.ScreenSpaceEventHandler(this.viewer.scene.canvas);
    this._handler.setInputAction((e) => this._doubleClick(e), C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);
    this._unlisten = [
      () => removeEventListener("keydown", onKeyDown),
      () => removeEventListener("keyup", onKeyUp),
      () => removeEventListener("blur", onBlur),
      this.viewer.scene.preUpdate.addEventListener(() => this._frame()),
    ];
  }

  _doubleClick(e) {
    if (!this.dblClickMoves) return;
    const picked = this.map.pickGround(e.position);
    if (!picked) return;
    const p = C.Matrix4.multiplyByPoint(this.fixedToLocal, C.Cartographic.toCartesian(picked), new C.Cartesian3());
    if (Math.hypot(p.x, p.y) > MAX_OFFSET_M) {
      this.onFarMove?.(C.Math.toDegrees(picked.latitude), C.Math.toDegrees(picked.longitude));
      return;
    }
    this.placeAt(p.x, p.y, this.car.heading, p.z);
  }

  /** Drop the car onto the ground at `placing` once its colliders are built. */
  _tryPlace() {
    const { x, y, heading } = this.placing;
    this.ground.focus = new C.Cartesian3(x, y, 0);
    // New colliders reach ray casts on a step; the car is out of the world till placed.
    this.world.step();
    const toi = this.ground.castRay({ x, y, z: GROUND_PROBE.up }, { x: 0, y: 0, z: -1 }, GROUND_PROBE.down);
    // Wait for the ground to settle at its finest too, or the car lands on a coarse tile.
    if (toi === null || this.ground.stats.pending > 0) return;
    this.car.place({ x, y, z: GROUND_PROBE.up - toi + 0.4 }, heading);
    this.car.body.setEnabled(true);
    this.placing = null;
    this._notice(null);
    this._quiet();
  }

  _resetUpright() {
    const t = this.car.body.translation();
    this.car.place({ x: t.x, y: t.y, z: t.z + 1.5 }, this.car.heading);
    this._quiet();
  }

  /** The truck has just been put down: its jump in speed is no bump. */
  _quiet() {
    this._vel = null;
    this._quietUntil = performance.now() + BUMP_QUIET_MS;
  }

  _controls() {
    const down = (...codes) => codes.some((c) => this.keys.has(c));
    let throttle = (down("KeyW", "ArrowUp") ? 1 : 0) - (down("KeyS", "ArrowDown") ? 1 : 0);
    let steer = (down("KeyA", "ArrowLeft") ? 1 : 0) - (down("KeyD", "ArrowRight") ? 1 : 0);
    let handbrake = down("Space");
    const radio = this.radio.read(this.placed);
    if (radio) {
      if (radio.steer !== 0) steer = radio.steer;
      if (radio.throttle !== 0) throttle = radio.throttle;
      handbrake ||= radio.handbrake;
      if (radio.reset && this.placed) this._resetUpright();
    }
    for (const pad of navigator.getGamepads?.() ?? []) {
      if (!pad || isRadio(pad)) continue;
      const stick = pad.axes[0] ?? 0;
      if (Math.abs(stick) > 0.08) steer = -stick;
      const gas = pad.buttons[7]?.value ?? 0;
      const brake = pad.buttons[6]?.value ?? 0;
      if (gas > 0.05 || brake > 0.05) throttle = gas - brake;
      handbrake ||= pad.buttons[0]?.pressed ?? false;
    }
    return { throttle, steer, handbrake };
  }

  _frame() {
    const now = performance.now();
    const dt = Math.min(0.1, (now - this._last) / 1000);
    const hud = this._hud;
    hud.worstMs = Math.max(hud.worstMs, now - this._last);
    this._last = now;
    const car = this.car;
    if (this.placing) this._tryPlace();
    if (!this.placing) {
      const start = performance.now();
      this._acc += dt;
      let steps = 0;
      const input = this._controls();
      while (this._acc >= STEP_S && steps < MAX_STEPS) {
        car.drive(STEP_S, input);
        this.world.step();
        this._acc -= STEP_S;
        steps++;
      }
      if (steps === MAX_STEPS) this._acc = 0;
      hud.physMs = hud.physMs * 0.9 + (performance.now() - start) * 0.1;
      const t = car.body.translation();
      this.ground.focus = new C.Cartesian3(t.x, t.y, t.z);
      this._guardFall(t);
      this._guardFence(t);
      this._sound(input);
    } else {
      this.engine.update(0, 0, false);
    }
    car.draw();
    if (car.model) car.model.show = !this.placing;
    this.onStep?.(this);
    this._updateCamera(dt);
    hud.frames++;
    if (now - hud.fpsT > 500) {
      hud.fps = (hud.frames * 1000) / (now - hud.fpsT);
      hud.shownWorstMs = hud.worstMs;
      hud.worstMs = 0;
      hud.frames = 0;
      hud.fpsT = now;
      this._showHud();
    }
  }

  /** Keep the last upright pose; put the car back there if it falls out of the world. Right it
   * when it has lain on its side or roof a while. */
  _guardFall(t) {
    const now = performance.now();
    const car = this.car;
    const upZ = car.uprightness;
    if (upZ < 0.4 && Math.abs(car.speed) < 1) {
      this._stuckSince ??= now;
      if (now - this._stuckSince > AUTO_RIGHT_MS) this._resetUpright();
    } else {
      this._stuckSince = null;
    }
    if (car.wheelsOnGround >= 3 && upZ > 0.8 && now - this._lastGoodT > 500) {
      this.lastGood = { x: t.x, y: t.y, heading: car.heading, z: t.z };
      this._lastGoodT = now;
    }
    const good = this.lastGood;
    if (good && t.z < good.z - 40) {
      this.falls++;
      this.placeAt(good.x, good.y, good.heading, good.z);
    }
  }

  /** The engine at the truck's speed and throttle; a bump when it stops short on something. */
  _sound(input) {
    this.engine.update(this.car.speed, input.throttle);
    const v = this.car.body.linvel();
    const prev = this._vel;
    this._vel = { x: v.x, y: v.y };
    if (!prev || performance.now() < this._quietUntil) return;
    const dv = Math.hypot(v.x - prev.x, v.y - prev.y);
    if (dv > BUMP_DV_MS) sfx.play("bump", { intensity: (dv - BUMP_DV_MS) / 8 });
  }

  /** The car outside the fence goes back in after OUTSIDE_FENCE_S, turned round. */
  _guardFence(t) {
    if (!this.outside) return;
    const now = performance.now();
    if (!this.outside(t.x, t.y)) {
      if (this._outsideSince !== null) this._notice(null);
      this._outsideSince = null;
      this._fenceSecond = null;
      if (this.car.wheelsOnGround >= 3) this._lastInside = { x: t.x, y: t.y, z: t.z, heading: this.car.heading };
      return;
    }
    this._outsideSince ??= now;
    const left = OUTSIDE_FENCE_S - (now - this._outsideSince) / 1000;
    if (left > 0) {
      this._notice(`Outside the fence: back inside in ${Math.ceil(left)} s`);
      if (Math.ceil(left) !== this._fenceSecond) sfx.play("fence");
      this._fenceSecond = Math.ceil(left);
      return;
    }
    this._outsideSince = null;
    this._fenceSecond = null;
    const back = this._lastInside ?? { x: 0, y: 0, z: 0, heading: this.car.heading };
    this.placeAt(back.x, back.y, back.heading + Math.PI, back.z);
  }

  /** Point the camera from local `eye` at local `target`, its top towards local `skyward`. */
  _look(eye, target, skyward = C.Cartesian3.UNIT_Z) {
    const V = C.Cartesian3;
    const direction = V.normalize(V.subtract(target, eye, new V()), new V());
    const right = V.normalize(V.cross(direction, skyward, new V()), new V());
    const up = V.cross(right, direction, new V());
    const rot = C.Matrix4.getMatrix3(this.localToFixed, new C.Matrix3());
    this.viewer.camera.setView({
      destination: C.Matrix4.multiplyByPoint(this.localToFixed, eye, new V()),
      orientation: {
        direction: C.Matrix3.multiplyByVector(rot, direction, direction),
        up: C.Matrix3.multiplyByVector(rot, up, up),
      },
    });
  }

  _updateCamera(dt) {
    if (this.cameraMode === "free") return;
    const { car, placing } = this;
    const t = placing ? { x: placing.x, y: placing.y, z: placing.z + 2 } : car.body.translation();
    const heading = placing ? placing.heading : car.heading;
    // Swing round behind the car, quicker the faster it goes; not when it is backing up.
    if (!placing && car.speed > -1) {
      let d = heading - this.cameraHeading;
      d = Math.atan2(Math.sin(d), Math.cos(d));
      this.cameraHeading += d * Math.min(1, dt * (1.5 + Math.abs(car.speed) * 0.12));
    }
    if (this.cameraMode === "hood") {
      const q = car.body.rotation();
      const m = C.Matrix3.fromQuaternion(new C.Quaternion(q.x, q.y, q.z, q.w));
      const eye = C.Matrix3.multiplyByVector(m, new C.Cartesian3(0.9, 0, 2.05), new C.Cartesian3());
      const ahead = C.Matrix3.multiplyByVector(m, new C.Cartesian3(20, 0, 1.6), new C.Cartesian3());
      // The truck's own up: the view rolls with it, upside down on its roof.
      const roof = C.Matrix3.multiplyByVector(m, C.Cartesian3.UNIT_Z, new C.Cartesian3());
      this._look(C.Cartesian3.add(eye, t, eye), C.Cartesian3.add(ahead, t, ahead), roof);
      return;
    }
    const rig = CAMERAS[this.cameraMode];
    const target = new C.Cartesian3(t.x, t.y, t.z + rig.look);
    const back = new C.Cartesian3(
      -Math.cos(this.cameraHeading) * rig.back,
      -Math.sin(this.cameraHeading) * rig.back,
      rig.up,
    );
    // Pull in in front of a wall between the car and the camera.
    const reach = C.Cartesian3.magnitude(back);
    const dir = C.Cartesian3.normalize(back, new C.Cartesian3());
    const toi = this.ground.castRay(target, dir, reach);
    const dist = toi === null ? reach : Math.max(1.5, toi - 0.4);
    const eye = C.Cartesian3.add(target, C.Cartesian3.multiplyByScalar(dir, dist, dir), new C.Cartesian3());
    this._look(eye, target);
  }

  _showHud() {
    const { hud: el = {} } = this.ui;
    const set = (key, text) => {
      if (el[key]) el[key].textContent = text;
    };
    const s = this.ground.stats;
    const h = this._hud;
    set("speed", Math.abs(this.car.speed * 3.6).toFixed(0));
    set("wheels", this.placing ? "–" : `${this.car.wheelsOnGround} / 4`);
    set("tiles", s.tiles);
    set("tris", s.triangles.toLocaleString());
    set("pending", s.pending);
    set("buildMs", `${s.buildMs.toFixed(1)} ms`);
    set("physMs", `${h.physMs.toFixed(2)} ms`);
    set("fps", `${h.fps.toFixed(0)} (worst ${h.shownWorstMs.toFixed(0)} ms)`);
  }
}
