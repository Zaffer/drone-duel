// Fly the TBS Chupito on the map: Nanawing's flight model (wingflight.js) in a local frame round
// the start, drawn as narenana's Chupito (external/web/chupito/, fetched by scripts/setup.sh).
// A first, basic demo: the ground under it is all it can hit. Start somewhere else with
// ?at=LAT,LON[,HEADING_DEG]; it starts at mission.toml's [sim] home, where drone 1 starts.
//
// The sticks: the keys, a gamepad (right stick, right trigger the throttle) or the drone radio
// (swarm.view's /radio, as mode 2). Raising the radio's throttle from the bottom throws it.
//
// Under `swarm wingsim` it flies on INAV SITL instead, in Python (swarm.wingsim): this page only
// draws it, from swarm.view's /wing, and posts back the map's ground under it, which the flight
// model then flies over. The radio is INAV's, read there. D shows the flight model at work, on the
// wing and beside it (wingdynamics.js).

import { BaseMap } from "./basemap.js";
import { feed as radio, isRadio } from "./driveradio.js";
import { ARROW_ID, DynamicsView } from "./wingdynamics.js";
import {
  CHUPITO,
  STEP_S,
  expo,
  headingOf,
  launch,
  reachedGround,
  ready,
  rotate,
  step,
  touchdown,
} from "./wingflight.js";

const C = Cesium;
const $ = (id) => document.getElementById(id);

const MODEL_URI = "/vendor/chupito/chupito_v6.glb";
const MODEL_CREDIT =
  "Wing model: TBS Chupito, recreated by narenana for Nanawing (sim.narenana.com); airframe by Team BlackSheep";
/** At most this much flight caught up in one frame, as Nanawing. */
const MAX_STEPS = Math.ceil(0.25 / STEP_S);
const GROUND_SAMPLE_MS = 100;
const ELEVON_MAX_RAD = C.Math.toRadians(25);
/** The prop's drawn spin at full throttle: slow enough not to strobe backwards. */
const PROP_RAD_S = 30;
const KEY_THROTTLE_PER_S = 0.8;
const DEADBAND = 0.04;
/** The radio's throttle throws the wing on the way up past here, once it has been down. */
const RADIO_THROW = { armBelow: 0.1, throwAbove: 0.5 };
const CAMERAS = {
  /** Nanawing's chase camera. */
  chase: { back: 4, up: 1.4, ahead: 3, fov: 1.1 },
  /** At the lens on the nose, tilted up. */
  fpv: { nose: 0.22, uptilt: C.Math.toRadians(20), fov: 1.62 },
  free: null,
};
const CAMERA_NAMES = Object.keys(CAMERAS);
const PHASES = { READY: "in the hand", FLYING: "flying", LANDED: "landed", CRASHED: "crashed" };
/** swarm.wingsim's phases. */
const SIM_PHASES = { hand: "in the hand", throw: "thrown", flying: "flying", landed: "landed", crashed: "crashed" };
/** At most this far ahead of the last state from /wing, along its velocity, s. */
const MAX_EXTRAPOLATE_S = 0.1;
/** Under INAV, the ground is sampled where the wing will be this far ahead, s: about a sample's
 * own delay, so it is there when the wing arrives. */
const GROUND_LEAD_S = 0.25;
/** The radio panel's live values this often, ms. */
const RADIO_PANEL_MS = 50;
const HIGH_US = 1700;
/** INAV's eight channels and what each means to it, from its settings (swarm.wingsim.inav's
 * CHUPITO_SETTINGS aux lines) and swarm.wingrun's throw switch. */
const INAV_CHANNELS = [
  { name: "roll" },
  { name: "pitch" },
  { name: "throttle", upward: true },
  { name: "yaw" },
  { name: "arm", mean: (us) => (us >= HIGH_US ? "ARM" : "off"), on: (us) => us >= HIGH_US },
  { name: "mode", mean: (us) => (us <= 1300 ? "ANGLE" : us >= HIGH_US ? "MANUAL" : "ACRO") },
  { name: "RTH", mean: (us) => (us >= HIGH_US ? "RTH" : "off"), on: (us) => us >= HIGH_US },
  { name: "throw", mean: (us) => (us > HIGH_US ? "high" : "low") },
];

/** Nanawing's frame (x east, y up, z north) to east-north-up. */
const enu = (v) => new C.Cartesian3(v[0], v[2], v[1]);
const deadband = (v) => (Math.abs(v) < DEADBAND ? 0 : v);

/** The body's right, up and nose in east-north-up, from roll, pitch and heading in degrees. */
function axesOf(rollDeg, pitchDeg, headingDeg) {
  const [r, p, y] = [rollDeg, pitchDeg, headingDeg].map(C.Math.toRadians);
  const [sr, cr, sp, cp, sy, cy] = [Math.sin(r), Math.cos(r), Math.sin(p), Math.cos(p), Math.sin(y), Math.cos(y)];
  // The body's axes in north-east-down, then east-north-up.
  const nose = [cp * cy, cp * sy, -sp];
  const right = [sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, sr * cp];
  const down = [cr * sp * cy + sr * sy, cr * sp * sy - sr * cy, cr * cp];
  const toEnu = (v, k = 1) => new C.Cartesian3(k * v[1], k * v[0], -k * v[2]);
  return { right: toEnu(right), up: toEnu(down, -1), nose: toEnu(nose) };
}

/** The wing on INAV SITL (`swarm wingsim`): its state from swarm.view's /wing as each comes, and
 * the page's commands back. It flies in heights above sea level from `origin` (lat, lon, alt);
 * the map's ground at the origin is at `mapGround` in the map's heights. */
class WingLink {
  constructor(origin, mapGround) {
    this.origin = origin;
    this.mapGround = mapGround;
    this.frame = null;
    this.at = 0;
    new EventSource("/wing").onmessage = (e) => {
      this.frame = Object.values(JSON.parse(e.data))[0];
      this.at = performance.now();
    };
  }

  toMap(alt) {
    return alt - this.origin[2] + this.mapGround;
  }

  fromMap(height) {
    return height - this.mapGround + this.origin[2];
  }

  _post(path, body = {}) {
    fetch(path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }).catch(
      () => {},
    );
  }

  throw() {
    this._post("/wing/throw");
  }

  reset() {
    this._post("/wing/reset");
  }

  /** The map's ground at `lat`, `lon` is at map height `height`. */
  ground(lat, lon, height) {
    this._post("/wing/ground", { lat, lon, alt: this.fromMap(height) });
  }
}

/** INAV's RC as it gets it: the two sticks (mode 2: throttle and yaw on the left, pitch and roll
 * on the right) and every channel, in µs and what INAV makes of it. */
class RadioPanel {
  constructor(root) {
    const el = (tag, cls, text) => {
      const e = document.createElement(tag);
      if (cls) e.className = cls;
      if (text !== undefined) e.textContent = text;
      return e;
    };
    this.root = root;
    const sticks = el("div", "sticks");
    this.dots = ["throttle · yaw", "pitch · roll"].map((label) => {
      const box = el("div", "stick");
      const dot = el("i");
      box.append(dot, el("b", "", label));
      sticks.append(box);
      return dot;
    });
    this.rows = INAV_CHANNELS.map((ch, i) => {
      const row = el("div", "ch-row");
      const bar = el("span", `ch-bar${ch.upward || ch.mean ? "" : " centred"}`);
      const fill = el("i");
      bar.append(fill);
      const us = el("span", "ch-us");
      const mean = el("span", "ch-mean");
      row.append(el("span", "ch-name", `CH${i + 1} ${ch.name}`), bar, us, mean);
      root.append(row);
      return { row, fill, us, mean };
    });
    root.prepend(el("div", "radio-title", "RADIO → INAV"), sticks);
    this._at = 0;
  }

  show(rc, now) {
    this.root.hidden = !rc?.length;
    if (!rc?.length || now - this._at < RADIO_PANEL_MS) return;
    this._at = now;
    const unit = (us) => Math.max(-1, Math.min(1, (us - 1500) / 500));
    // Left: yaw across, throttle up; right: roll across, pitch up (forward is high).
    for (const [dot, x, y] of [
      [this.dots[0], rc[3], rc[2]],
      [this.dots[1], rc[0], rc[1]],
    ]) {
      dot.style.left = `${50 + unit(x) * 50}%`;
      dot.style.top = `${50 - unit(y) * 50}%`;
    }
    INAV_CHANNELS.forEach((ch, i) => {
      const r = this.rows[i];
      const v = unit(rc[i]);
      if (ch.upward || ch.mean) {
        r.fill.style.left = "0%";
        r.fill.style.width = `${((v + 1) / 2) * 100}%`;
      } else {
        r.fill.style.left = `${50 + Math.min(0, v) * 50}%`;
        r.fill.style.width = `${Math.abs(v) * 50}%`;
      }
      const us = `${Math.round(rc[i])}`;
      if (r.us.textContent !== us) r.us.textContent = us;
      const mean = ch.mean ? ch.mean(rc[i]) : "";
      if (r.mean.textContent !== mean) r.mean.textContent = mean;
      r.row.classList.toggle("on", ch.on?.(rc[i]) ?? false);
    });
  }
}

/** A key's stick: towards `target` (-1, 0, 1) over the ramp, back to centre over the release. */
function ramp(value, target, dt, { rampSeconds, releaseSeconds }) {
  const rate = dt / (target === 0 ? releaseSeconds : rampSeconds);
  return value + Math.max(-rate, Math.min(rate, target - value));
}

class WingSession {
  constructor(map, origin, heading, ui, link = null) {
    this.map = map;
    /** The wing on INAV SITL, or null to fly it here. */
    this.link = link;
    this.viewer = map.viewer;
    this.ui = ui;
    this.localToFixed = C.Transforms.eastNorthUpToFixedFrame(origin);
    this.fixedToLocal = C.Matrix4.inverseTransformation(this.localToFixed, new C.Matrix4());
    this.startHeading = heading;
    /** The ground's height in the local frame under the wing, as last sampled. */
    this.groundY = 0;
    this._groundAt = 0;
    this.state = ready([0, CHUPITO.launchHeight, 0], heading);
    this.keys = new Set();
    this.keyThrottle = 0;
    this._keyPitch = 0;
    this._keyRoll = 0;
    this._padLaunch = false;
    this._radioArmed = false;
    this.input = { pitch: 0, roll: 0, throttle: 0, source: "keys" };
    this._acc = 0;
    this._last = performance.now();
    this._hud = { frames: 0, fpsT: this._last, fps: 0 };
    this.cameraMode = "chase";
    this.cameraHeading = heading;
    this._propAngle = 0;
    this.radioPanel = link && ui.radio ? new RadioPanel(ui.radio) : null;
    /** The flight model at work: only swarm.wingsim's tells. */
    this.dynamics =
      link && ui.dynamics
        ? new DynamicsView(this.viewer, this.localToFixed, ui.dynamics, ui.dynamicsHistory, ui.dynamicsLegend)
        : null;

    this.modelMatrix = C.Matrix4.clone(C.Matrix4.IDENTITY);
    this.model = null;
    this._nodes = null;
    C.Model.fromGltfAsync({ url: MODEL_URI, modelMatrix: this.modelMatrix }).then(
      (model) => {
        this.model = this.viewer.scene.primitives.add(model);
        model.readyEvent.addEventListener(() => this._findNodes(model));
      },
      (err) => this._notice(`No Chupito model (run scripts/setup.sh): ${err.message ?? err}`),
    );
    this.viewer.creditDisplay.addStaticCredit(new C.Credit(MODEL_CREDIT));
    if (link) this._reportGround();
    else radio.open();
    this._wire();
    this.setCamera("chase");
  }

  /** The prop and elevon nodes, to move with the throttle and sticks. Each elevon by its side: a
   * glTF model's right is -x. */
  _findNodes(model) {
    const node = (name) => {
      const n = model.getNode(name);
      return n ? { node: n, original: C.Matrix4.clone(n.originalMatrix) } : null;
    };
    const elevons = [];
    for (const name of ["elevonPivotL", "elevonPivotR"]) {
      const n = node(name);
      if (n) elevons.push({ ...n, right: C.Matrix4.getTranslation(n.original, new C.Cartesian3()).x < 0 });
    }
    this._nodes = { prop: node("propeller"), elevons };
  }

  _notice(text) {
    this.ui.notice(text);
  }

  setCamera(mode) {
    this.cameraMode = mode;
    this.viewer.scene.screenSpaceCameraController.enableInputs = mode === "free";
    const rig = CAMERAS[mode];
    if (rig) this.viewer.camera.frustum.fov = rig.fov;
    this.ui.camMode.textContent = mode;
  }

  _wire() {
    addEventListener("keydown", (e) => {
      if (e.repeat) return;
      this.keys.add(e.code);
      if (e.code === "Space") this._throw();
      if (e.code === "KeyR") this._reset();
      if (e.code === "KeyC") {
        const i = CAMERA_NAMES.indexOf(this.cameraMode);
        this.setCamera(CAMERA_NAMES[(i + 1) % CAMERA_NAMES.length]);
      }
      if (e.code === "KeyH") this.ui.help.hidden = !this.ui.help.hidden;
      // The dynamics panel takes the help's corner.
      if (e.code === "KeyD" && this.dynamics) this.ui.help.hidden = this.dynamics.togglePanel();
      if (e.code === "BracketLeft" || e.code === "BracketRight") this.dynamics?.resize(e.code === "BracketRight");
      // The dynamics on the wing: each layer on its number, Shift+5 the trail's colours.
      const layer = /^Digit([1-5])$/.exec(e.code);
      if (layer && this.dynamics) {
        if (e.shiftKey && layer[1] === "5") this.dynamics.cycleTrail();
        else this.dynamics.toggleLayer(Number(layer[1]));
      }
      if (e.code.startsWith("Arrow") || e.code === "Space") e.preventDefault();
    });
    addEventListener("keyup", (e) => this.keys.delete(e.code));
    addEventListener("blur", () => this.keys.clear());
    this.viewer.scene.preUpdate.addEventListener(() => this._frame());
  }

  /** Throw it: from the hand, or picked up from where it came down, facing the same way. */
  _throw() {
    if (this.link) {
      this.link.throw();
      return;
    }
    let s = this.state;
    if (s.phase === "FLYING") return;
    if (s.phase !== "READY") {
      s = ready([s.position[0], this.groundY + CHUPITO.launchHeight, s.position[2]], headingOf(s.orientation));
    }
    this.state = launch(s);
    this.keyThrottle = CHUPITO.launchThrottle;
    this._notice(null);
  }

  /** Back in the hand at the start. */
  _reset() {
    if (this.link) {
      this.link.reset();
      return;
    }
    this.state = ready([0, CHUPITO.launchHeight, 0], this.startHeading);
    this.cameraHeading = this.startHeading;
    this._groundAt = 0;
    this._notice(null);
  }

  /** The sticks: the keys while any is held, else the radio, else a gamepad. Throws it on the
   * radio's throttle coming up, or a gamepad's A. */
  _readInput(dt) {
    const down = (...codes) => codes.some((c) => this.keys.has(c));
    const kb = CHUPITO.keyboard;
    const keyPitch = (down("ArrowDown") ? 1 : 0) - (down("ArrowUp") ? 1 : 0);
    const keyRoll = (down("ArrowRight") ? 1 : 0) - (down("ArrowLeft") ? 1 : 0);
    this._keyPitch = ramp(this._keyPitch, keyPitch, dt, kb.pitch);
    this._keyRoll = ramp(this._keyRoll, keyRoll, dt, kb.roll);
    const keyThrottle = (down("KeyW", "ShiftLeft") ? 1 : 0) - (down("KeyS", "ControlLeft") ? 1 : 0);
    this.keyThrottle = Math.max(0, Math.min(1, this.keyThrottle + keyThrottle * KEY_THROTTLE_PER_S * dt));
    const input = {
      pitch: expo(this._keyPitch, kb.pitch.expo),
      roll: expo(this._keyRoll, kb.roll.expo),
      throttle: this.keyThrottle,
      source: "keys",
    };
    const keysHeld = keyPitch !== 0 || keyRoll !== 0;
    const flying = this.state.phase === "FLYING";
    const e = CHUPITO.stickExpo;

    const ch = radio.channels;
    if (ch) {
      // swarm.radio's order: roll, pitch (forward high), throttle, yaw.
      const throttle = (ch[2] + 1) / 2;
      if (!keysHeld) {
        input.roll = expo(deadband(ch[0]), e);
        input.pitch = expo(-deadband(ch[1]), e);
      }
      input.throttle = throttle;
      input.source = "radio";
      if (!flying && throttle < RADIO_THROW.armBelow) this._radioArmed = true;
      if (!flying && this._radioArmed && throttle > RADIO_THROW.throwAbove) {
        this._radioArmed = false;
        this._throw();
      }
      return input;
    }
    for (const pad of navigator.getGamepads?.() ?? []) {
      if (!pad || isRadio(pad)) continue;
      if (!keysHeld) {
        input.roll = expo(deadband(pad.axes[2] ?? 0), e);
        input.pitch = expo(deadband(pad.axes[3] ?? 0), e);
      }
      input.throttle = pad.buttons[7]?.value ?? 0;
      input.source = "gamepad";
      const a = pad.buttons[0]?.pressed ?? false;
      if (a && !this._padLaunch) this._throw();
      this._padLaunch = a;
      break;
    }
    return input;
  }

  /** What the ground is not: the wing and its dynamics arrows. */
  _notGround() {
    return this.model ? [this.model, ARROW_ID] : [ARROW_ID];
  }

  /** The ground's height under the wing, from what is drawn there: the 3D tiles' roofs too. */
  _sampleGround() {
    const fixed = C.Matrix4.multiplyByPoint(this.localToFixed, enu(this.state.position), new C.Cartesian3());
    const carto = C.Cartographic.fromCartesian(fixed);
    const scene = this.viewer.scene;
    const h = this.map.groundIsTiles
      ? scene.sampleHeight(carto, this._notGround())
      : scene.globe.getHeight(carto);
    if (h !== undefined) this._setGround(carto, h);
  }

  _setGround(carto, h) {
    const ground = C.Cartesian3.fromRadians(carto.longitude, carto.latitude, h);
    this.groundY = C.Matrix4.multiplyByPoint(this.fixedToLocal, ground, ground).z;
  }

  /** Under INAV: the map's ground under the wing, posted to the flight model, each from the most
   * detailed tiles there. Coarser ones, drawn while those load, can be kilometres out. */
  async _reportGround() {
    const scene = this.viewer.scene;
    const terrain = this.viewer.terrainProvider;
    for (;;) {
      const p = this.state.position;
      const v = this.state.velocity;
      const ahead = [0, 1, 2].map((i) => p[i] + v[i] * GROUND_LEAD_S);
      const fixed = C.Matrix4.multiplyByPoint(this.localToFixed, enu(ahead), new C.Cartesian3());
      const carto = C.Cartographic.fromCartesian(fixed);
      carto.height = undefined;
      try {
        if (this.map.groundIsTiles) {
          await scene.sampleHeightMostDetailed([carto], this._notGround());
        } else if (terrain instanceof C.EllipsoidTerrainProvider) {
          carto.height = 0;
        } else {
          await C.sampleTerrainMostDetailed(terrain, [carto]);
        }
      } catch (err) {
        console.warn("ground height lookup failed", err);
      }
      if (carto.height !== undefined) {
        this._setGround(carto, carto.height);
        this.link.ground(C.Math.toDegrees(carto.latitude), C.Math.toDegrees(carto.longitude), carto.height);
      }
      await new Promise((r) => setTimeout(r, GROUND_SAMPLE_MS));
    }
  }

  _frame() {
    const now = performance.now();
    const dt = Math.min(0.25, (now - this._last) / 1000);
    this._last = now;
    if (!this.link && now - this._groundAt > GROUND_SAMPLE_MS) {
      this._groundAt = now;
      this._sampleGround();
    }
    if (this.link) {
      this.state = this._linkState(now) ?? this.state;
      this.input = { pitch: 0, roll: 0, throttle: this.state.throttle, source: "INAV" };
      this.radioPanel?.show(this.link.frame?.rc, now);
    } else {
      this._fly(dt);
    }
    this._draw(this.input, dt);
    if (this.link) this.dynamics?.update(this.link.frame, enu(this.state.position), this._axes(), now);
    this._updateCamera(dt);
    const hud = this._hud;
    hud.frames++;
    if (now - hud.fpsT > 250) {
      hud.fps = (hud.frames * 1000) / (now - hud.fpsT);
      hud.frames = 0;
      hud.fpsT = now;
      this._showHud();
    }
  }

  /** The last state from /wing in this page's terms, carried on along its velocity since. */
  _linkState(now) {
    const link = this.link;
    const f = link.frame;
    if (!f) return null;
    const w = f.wing;
    const fixed = C.Cartesian3.fromDegrees(w.lon, w.lat, link.toMap(w.alt));
    const p = C.Matrix4.multiplyByPoint(this.fixedToLocal, fixed, fixed);
    const [vn, ve, vd] = w.velocity;
    const t = Math.min(MAX_EXTRAPOLATE_S, (now - link.at) / 1000);
    return {
      // Nanawing's frame, as the rest of this page: x east, y up, z north.
      position: [p.x + ve * t, p.z - vd * t, p.y + vn * t],
      velocity: [ve, -vd, vn],
      axes: axesOf(w.roll, w.pitch, w.heading),
      heading: C.Math.toRadians(w.heading),
      airspeed: w.airspeed,
      throttle: w.motor,
      phase: w.phase,
      agl: w.agl,
      elevons: w.elevons,
      armed: f.armed,
      blockers: f.blockers,
    };
  }

  /** Fly it here, on Nanawing's flight model. */
  _fly(dt) {
    const input = this._readInput(dt);
    this.input = input;
    const s = this.state;
    if (s.phase === "FLYING") {
      this._acc += dt;
      let steps = 0;
      while (this._acc >= STEP_S && steps < MAX_STEPS && this.state.phase === "FLYING") {
        const next = step(this.state, input, STEP_S);
        this.state = reachedGround(next, this.groundY, STEP_S) ? touchdown(next, this.groundY) : next;
        this._acc -= STEP_S;
        steps++;
      }
      if (this.state.phase === "CRASHED") this._notice("Crashed. Space (or the radio's throttle, down then up) to go again");
      if (this.state.phase === "LANDED") this._notice("Landed. Space to throw it again");
    } else {
      this._acc = 0;
      if (s.phase === "READY") s.position[1] = this.groundY + CHUPITO.launchHeight;
    }
  }

  _heading() {
    return this.state.heading ?? headingOf(this.state.orientation);
  }

  /** The body's right, up and nose in east-north-up. */
  _axes() {
    if (this.state.axes) return this.state.axes;
    const q = this.state.orientation;
    return { right: enu(rotate(q, [1, 0, 0])), up: enu(rotate(q, [0, 1, 0])), nose: enu(rotate(q, [0, 0, 1])) };
  }

  _draw(input, dt) {
    const { right, up, nose } = this._axes();
    const p = enu(this.state.position);
    // Cesium turns the model's glTF front (+Z) and up (+Y) onto +x and +z, its left onto +y.
    // prettier-ignore
    const local = new C.Matrix4(
      nose.x, -right.x, up.x, p.x,
      nose.y, -right.y, up.y, p.y,
      nose.z, -right.z, up.z, p.z,
      0, 0, 0, 1,
    );
    C.Matrix4.multiply(this.localToFixed, local, this.modelMatrix);
    if (!this.model) return;
    this.model.modelMatrix = this.modelMatrix;
    const nodes = this._nodes;
    if (!nodes) return;
    const propSpeed = this.link || this.state.phase === "FLYING" ? this.state.throttle : 0;
    if (nodes.prop) {
      this._propAngle = (this._propAngle + propSpeed * PROP_RAD_S * dt) % (2 * Math.PI);
      const spin = C.Matrix4.fromRotation(C.Matrix3.fromRotationZ(this._propAngle));
      nodes.prop.node.matrix = C.Matrix4.multiply(nodes.prop.original, spin, new C.Matrix4());
    }
    // Elevons, trailing edge up for +: both with the pitch, each against the other with the roll.
    // swarm.wingsim's, in degrees, left then right, trailing edge down for +.
    const sim = this.state.elevons;
    for (const e of nodes.elevons) {
      const mix = Math.max(-1, Math.min(1, input.pitch + (e.right ? input.roll : -input.roll)));
      const angle = sim ? -C.Math.toRadians(sim[e.right ? 1 : 0]) : mix * ELEVON_MAX_RAD;
      const hinge = C.Matrix4.fromRotation(C.Matrix3.fromRotationX(angle));
      e.node.matrix = C.Matrix4.multiply(e.original, hinge, new C.Matrix4());
    }
  }

  /** Point the camera from local `eye` along local `direction`, its top towards local `top`. */
  _look(eye, direction, top) {
    const V = C.Cartesian3;
    const right = V.normalize(V.cross(direction, top, new V()), new V());
    const up = V.cross(right, direction, new V());
    const rot = C.Matrix4.getMatrix3(this.localToFixed, new C.Matrix3());
    this.viewer.camera.setView({
      destination: C.Matrix4.multiplyByPoint(this.localToFixed, eye, new V()),
      orientation: {
        direction: C.Matrix3.multiplyByVector(rot, direction, new V()),
        up: C.Matrix3.multiplyByVector(rot, up, up),
      },
    });
  }

  _updateCamera(dt) {
    if (this.cameraMode === "free") return;
    const V = C.Cartesian3;
    const p = enu(this.state.position);
    const { up, nose } = this._axes();
    if (this.cameraMode === "fpv") {
      const rig = CAMERAS.fpv;
      const eye = V.add(p, V.multiplyByScalar(nose, rig.nose, new V()), new V());
      const c = Math.cos(rig.uptilt);
      const k = Math.sin(rig.uptilt);
      const look = V.normalize(V.add(V.multiplyByScalar(nose, c, new V()), V.multiplyByScalar(up, k, new V()), new V()), new V());
      const top = V.subtract(V.multiplyByScalar(up, c, new V()), V.multiplyByScalar(nose, k, new V()), new V());
      this._look(eye, look, top);
      return;
    }
    // Behind the way it is going, swinging round quicker the faster it goes.
    const rig = CAMERAS.chase;
    const v = this.state.velocity;
    const heading = Math.hypot(v[0], v[2]) > 2 ? Math.atan2(v[0], v[2]) : this._heading();
    const d = Math.atan2(Math.sin(heading - this.cameraHeading), Math.cos(heading - this.cameraHeading));
    this.cameraHeading += d * Math.min(1, dt * (2 + this.state.airspeed * 0.15));
    const back = new V(Math.sin(this.cameraHeading), Math.cos(this.cameraHeading), 0);
    const eye = new V(p.x - back.x * rig.back, p.y - back.y * rig.back, p.z + rig.up);
    const target = new V(p.x + back.x * rig.ahead, p.y + back.y * rig.ahead, p.z);
    this._look(eye, V.normalize(V.subtract(target, eye, new V()), new V()), V.UNIT_Z);
  }

  /** What to do next, under INAV. */
  _linkNotice() {
    const s = this.state;
    if (!this.link.frame) return "Waiting for the wing…";
    if (s.phase === "crashed") return "Crashed. R to put it back in the hand";
    if (s.phase === "landed") return "Landed. R to put it back in the hand";
    if (s.phase !== "hand") return null;
    const others = s.blockers.filter((b) => b !== "arm switch");
    if (!s.armed && others.length) return `INAV cannot arm yet: ${others.join(", ")}`;
    if (!s.armed) return "Arm it (CH5 up), raise the throttle, then Space or CH8 to throw";
    return "Raise the throttle, then Space or CH8 to throw";
  }

  _showHud() {
    const { hud } = this.ui;
    const s = this.state;
    if (this.link) {
      hud.speed.textContent = (s.airspeed * 3.6).toFixed(0);
      hud.height.textContent = s.agl === undefined ? "–" : `${s.agl.toFixed(1)} m`;
      hud.throttle.textContent = `${Math.round(s.throttle * 100)} %`;
      hud.phase.textContent = SIM_PHASES[s.phase] ?? s.phase;
      hud.phase.className = "";
      hud.input.textContent = s.armed ? "INAV, armed" : "INAV, disarmed";
      hud.fps.textContent = this._hud.fps.toFixed(0);
      this._notice(this._linkNotice());
      return;
    }
    hud.speed.textContent = (s.airspeed * 3.6).toFixed(0);
    hud.height.textContent = `${(s.position[1] - this.groundY).toFixed(1)} m`;
    hud.throttle.textContent = `${Math.round((s.phase === "FLYING" ? s.throttle : this.input.throttle) * 100)} %`;
    hud.phase.textContent = s.phase === "FLYING" && s.stalled ? "STALL" : PHASES[s.phase];
    hud.phase.className = s.phase === "FLYING" && s.stalled ? "stall" : "";
    hud.input.textContent = this.input.source;
    hud.fps.textContent = this._hud.fps.toFixed(0);
  }
}

const config = await fetch("/config.json").then((r) => r.json());
const map = new BaseMap($("map"), config);
const notice = (text) => {
  $("notice").hidden = !text;
  $("notice").textContent = text ?? "";
};
const params = new URLSearchParams(location.search);
const simOrigin = config.wing?.origin;
const [lat, lon, headingDeg] = simOrigin
  ? [simOrigin[0], simOrigin[1], 0]
  : params.has("at")
    ? params.get("at").split(",").map(Number)
    : [config.markers?.[0]?.lat ?? 45.319287, config.markers?.[0]?.lon ?? -75.758596, 0];
$("help-keys").hidden = Boolean(simOrigin);
$("help-inav").hidden = !simOrigin;

await map.ready;
notice("Finding the ground…");
const groundH = await map.groundHeight(lat, lon);
const link = simOrigin ? new WingLink(simOrigin, groundH) : null;
const ui = {
  notice,
  radio: $("radio"),
  dynamics: $("dynamics"),
  dynamicsHistory: $("dynamics-history"),
  dynamicsLegend: $("dynamics-legend"),
  help: $("help"),
  camMode: $("cam-mode"),
  hud: {
    speed: $("speed"),
    height: $("height"),
    throttle: $("throttle"),
    phase: $("phase"),
    input: $("input"),
    fps: $("fps"),
  },
};
const session = new WingSession(
  map,
  C.Cartesian3.fromDegrees(lon, lat, groundH),
  C.Math.toRadians(headingDeg || 0),
  ui,
  link,
);
if (!link) notice("Space to throw it");

window.wing = { session, viewer: map.viewer };
