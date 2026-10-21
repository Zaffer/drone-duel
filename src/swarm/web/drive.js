// Drive a car on the map's Google Photorealistic 3D Tiles, with physics and collisions.
//
// Rapier (WebAssembly) runs the car in a local east-north-up frame around where it starts;
// the tiles drawn near the car become its colliders (tilecolliders.js). Start somewhere else
// with ?at=LAT,LON[,HEADING_DEG], or double-click the map to move the car there.

import RAPIER from "/vendor/rapier/dist/rapier.mjs";
import { BaseMap } from "./basemap.js";
import { Car } from "./car.js";
import { TileColliders } from "./tilecolliders.js";

const C = Cesium;
const $ = (id) => document.getElementById(id);
const STEP_S = 1 / 60;
const MAX_STEPS = 4;
/** A move further than this from the frame's origin reloads the page there instead. */
const MAX_OFFSET_M = 2500;
/** The car waits for ground under it, looking this far down from this high. */
const GROUND_PROBE = { up: 60, down: 200 };
/** The car rights itself after lying on its side or roof this long. */
const AUTO_RIGHT_MS = 2500;
const CAMERAS = {
  chase: { back: 9, up: 3.4, look: 1.4 },
  far: { back: 22, up: 9, look: 1 },
  hood: null,
  free: null,
};
const CAMERA_NAMES = Object.keys(CAMERAS);

const config = await fetch("/config.json").then((r) => r.json());
const map = new BaseMap($("map"), config);
const viewer = map.viewer;
const scene = viewer.scene;
const notice = (text) => {
  $("notice").hidden = !text;
  $("notice").textContent = text ?? "";
};

const params = new URLSearchParams(location.search);
const [startLat, startLon, startHeadingDeg] = params.has("at")
  ? params.get("at").split(",").map(Number)
  : [config.markers?.[0]?.lat ?? 45.319287, config.markers?.[0]?.lon ?? -75.758596, 0];

await Promise.all([RAPIER.init(), map.ready]);
if (!map.groundIsTiles) {
  notice(`Driving needs Google 3D Tiles: set GOOGLE_MAPS_API_KEY. ${map.fallbackReason ?? ""}`);
  throw new Error("no 3D tiles");
}

notice("Finding the ground…");
const groundH = await map.groundHeight(startLat, startLon);
const origin = C.Cartesian3.fromDegrees(startLon, startLat, groundH);
const localToFixed = C.Transforms.eastNorthUpToFixedFrame(origin);
const fixedToLocal = C.Matrix4.inverseTransformation(localToFixed, new C.Matrix4());
const toFixed = (p) =>
  C.Matrix4.multiplyByPoint(localToFixed, new C.Cartesian3(p.x, p.y, p.z), new C.Cartesian3());

const world = new RAPIER.World({ x: 0, y: 0, z: -9.81 });
world.timestep = STEP_S;
const ground = new TileColliders(viewer, map.tileset, RAPIER, world, origin);
const car = new Car(RAPIER, world, scene, localToFixed);

// --- Placing the car -------------------------------------------------------------------

/** Where the car goes once there is ground under it: local {x, y}, heading. */
let placing = { x: 0, y: 0, heading: C.Math.toRadians(90 - (startHeadingDeg || 0)) };
car.body.setEnabled(false);
/** The last pose with the car upright on its wheels, to reset to. */
let lastGood = null;

function placeAt(x, y, heading) {
  placing = { x, y, heading };
  car.body.setEnabled(false);
  notice("Loading the ground…");
}

/** Drop the car onto the ground at `placing` once its colliders are built. */
function tryPlace() {
  const { x, y, heading } = placing;
  ground.focus = new C.Cartesian3(x, y, 0);
  // New colliders reach ray casts on a step; the car is out of the world till placed.
  world.step();
  const toi = ground.castRay({ x, y, z: GROUND_PROBE.up }, { x: 0, y: 0, z: -1 }, GROUND_PROBE.down);
  // Wait for the ground to settle at its finest too, or the car lands on a coarse tile.
  if (toi === null || ground.stats.pending > 0) return;
  car.place({ x, y, z: GROUND_PROBE.up - toi + 0.4 }, heading);
  car.body.setEnabled(true);
  placing = null;
  notice(null);
}

function resetUpright() {
  const t = car.body.translation();
  car.place({ x: t.x, y: t.y, z: t.z + 1.5 }, car.heading);
}

// --- Input -----------------------------------------------------------------------------

const keys = new Set();
addEventListener("keydown", (e) => {
  if (e.repeat) return;
  keys.add(e.code);
  if (e.code === "KeyR") resetUpright();
  if (e.code === "KeyC") {
    setCamera(CAMERA_NAMES[(CAMERA_NAMES.indexOf(cameraMode) + 1) % CAMERA_NAMES.length]);
  }
  if (e.code === "KeyG") ground.setDebug(!ground.debug);
  if (e.code === "KeyH") $("help").hidden = !$("help").hidden;
});
addEventListener("keyup", (e) => keys.delete(e.code));
addEventListener("blur", () => keys.clear());

function controls() {
  const down = (...codes) => codes.some((c) => keys.has(c));
  let throttle = (down("KeyW", "ArrowUp") ? 1 : 0) - (down("KeyS", "ArrowDown") ? 1 : 0);
  let steer = (down("KeyA", "ArrowLeft") ? 1 : 0) - (down("KeyD", "ArrowRight") ? 1 : 0);
  let handbrake = down("Space");
  for (const pad of navigator.getGamepads?.() ?? []) {
    if (!pad) continue;
    const stick = pad.axes[0] ?? 0;
    if (Math.abs(stick) > 0.08) steer = -stick;
    const gas = pad.buttons[7]?.value ?? 0;
    const brake = pad.buttons[6]?.value ?? 0;
    if (gas > 0.05 || brake > 0.05) throttle = gas - brake;
    handbrake ||= pad.buttons[0]?.pressed ?? false;
  }
  return { throttle, steer, handbrake };
}

const handler = new C.ScreenSpaceEventHandler(scene.canvas);
handler.setInputAction((e) => {
  const picked = map.pickGround(e.position);
  if (!picked) return;
  const p = C.Matrix4.multiplyByPoint(fixedToLocal, C.Cartographic.toCartesian(picked), new C.Cartesian3());
  if (Math.hypot(p.x, p.y) > MAX_OFFSET_M) {
    const [lat, lon] = [picked.latitude, picked.longitude].map((r) => C.Math.toDegrees(r).toFixed(6));
    location.search = `?at=${lat},${lon}`;
    return;
  }
  placeAt(p.x, p.y, car.heading);
}, C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);

// --- Camera ----------------------------------------------------------------------------

let cameraMode = "chase";
let cameraHeading = placing.heading;

function setCamera(mode) {
  cameraMode = mode;
  scene.screenSpaceCameraController.enableInputs = mode === "free";
  // Cesium keeps a free camera above the tiles by reading their triangles back from the GPU
  // under the camera, every frame it moves: a tenth of the frame time. The car's cameras stay
  // out of walls on the colliders instead.
  map.tileset.enableCollision = mode === "free";
  $("cam-mode").textContent = mode;
}
setCamera("chase");

/** Point the camera from local `eye` at local `target`. */
function look(eye, target) {
  const V = C.Cartesian3;
  const direction = V.normalize(V.subtract(target, eye, new V()), new V());
  const right = V.normalize(V.cross(direction, V.UNIT_Z, new V()), new V());
  const up = V.cross(right, direction, new V());
  const rot = C.Matrix4.getMatrix3(localToFixed, new C.Matrix3());
  viewer.camera.setView({
    destination: toFixed(eye),
    orientation: {
      direction: C.Matrix3.multiplyByVector(rot, direction, direction),
      up: C.Matrix3.multiplyByVector(rot, up, up),
    },
  });
}

function updateCamera(dt) {
  if (cameraMode === "free") return;
  const t = placing ? { x: placing.x, y: placing.y, z: 2 } : car.body.translation();
  const heading = placing ? placing.heading : car.heading;
  // Swing round behind the car, quicker the faster it goes; not when it is backing up.
  if (!placing && car.speed > -1) {
    let d = heading - cameraHeading;
    d = Math.atan2(Math.sin(d), Math.cos(d));
    cameraHeading += d * Math.min(1, dt * (1.5 + Math.abs(car.speed) * 0.12));
  }
  if (cameraMode === "hood") {
    const q = car.body.rotation();
    const quat = new C.Quaternion(q.x, q.y, q.z, q.w);
    const m = C.Matrix3.fromQuaternion(quat);
    const eye = C.Matrix3.multiplyByVector(m, new C.Cartesian3(0.9, 0, 2.05), new C.Cartesian3());
    const ahead = C.Matrix3.multiplyByVector(m, new C.Cartesian3(20, 0, 1.6), new C.Cartesian3());
    look(C.Cartesian3.add(eye, t, eye), C.Cartesian3.add(ahead, t, ahead));
    return;
  }
  const rig = CAMERAS[cameraMode];
  const target = new C.Cartesian3(t.x, t.y, t.z + rig.look);
  const back = new C.Cartesian3(
    -Math.cos(cameraHeading) * rig.back,
    -Math.sin(cameraHeading) * rig.back,
    rig.up,
  );
  // Pull in in front of a wall between the car and the camera.
  const reach = C.Cartesian3.magnitude(back);
  const dir = C.Cartesian3.normalize(back, new C.Cartesian3());
  const toi = ground.castRay(target, dir, reach);
  const dist = toi === null ? reach : Math.max(1.5, toi - 0.4);
  const eye = C.Cartesian3.add(target, C.Cartesian3.multiplyByScalar(dir, dist, dir), new C.Cartesian3());
  look(eye, target);
}

// --- Loop ------------------------------------------------------------------------------

let last = performance.now();
let acc = 0;
let physMs = 0;
let frames = 0;
let fpsT = last;
let fps = 0;
let worstMs = 0;
let shownWorstMs = 0;
let falls = 0;

scene.preUpdate.addEventListener(() => {
  const now = performance.now();
  const dt = Math.min(0.1, (now - last) / 1000);
  worstMs = Math.max(worstMs, now - last);
  last = now;
  if (placing) tryPlace();
  if (!placing) {
    const start = performance.now();
    acc += dt;
    let steps = 0;
    const input = controls();
    while (acc >= STEP_S && steps < MAX_STEPS) {
      car.drive(STEP_S, input);
      world.step();
      acc -= STEP_S;
      steps++;
    }
    if (steps === MAX_STEPS) acc = 0;
    physMs = physMs * 0.9 + (performance.now() - start) * 0.1;
    const t = car.body.translation();
    ground.focus = new C.Cartesian3(t.x, t.y, t.z);
    guardFall(t);
  }
  car.draw();
  if (car.model) car.model.show = !placing;
  updateCamera(dt);
  frames++;
  if (now - fpsT > 500) {
    fps = (frames * 1000) / (now - fpsT);
    shownWorstMs = worstMs;
    worstMs = 0;
    frames = 0;
    fpsT = now;
    showHud();
  }
});

let lastGoodT = 0;
let stuckSince = null;
/** Keep the last upright pose; put the car back there if it falls out of the world. Right it
 * when it has lain on its side or roof a while. */
function guardFall(t) {
  const now = performance.now();
  const upZ = car.uprightness;
  if (upZ < 0.4 && Math.abs(car.speed) < 1) {
    stuckSince ??= now;
    if (now - stuckSince > AUTO_RIGHT_MS) resetUpright();
  } else {
    stuckSince = null;
  }
  if (car.wheelsOnGround >= 3 && upZ > 0.8 && now - lastGoodT > 500) {
    lastGood = { x: t.x, y: t.y, heading: car.heading, z: t.z };
    lastGoodT = now;
  }
  if (lastGood && t.z < lastGood.z - 40) {
    falls++;
    placeAt(lastGood.x, lastGood.y, lastGood.heading);
  }
}

function showHud() {
  const s = ground.stats;
  $("speed").textContent = Math.abs(car.speed * 3.6).toFixed(0);
  $("wheels").textContent = placing ? "–" : `${car.wheelsOnGround} / 4`;
  $("tiles").textContent = s.tiles;
  $("tris").textContent = s.triangles.toLocaleString();
  $("pending").textContent = s.pending;
  $("build-ms").textContent = `${s.buildMs.toFixed(1)} ms`;
  $("phys-ms").textContent = `${physMs.toFixed(2)} ms`;
  $("fps").textContent = `${fps.toFixed(0)} (worst ${shownWorstMs.toFixed(0)} ms)`;
}

notice("Loading the ground…");
window.drive = { car, ground, world, viewer, placeAt, keys, fixedToLocal, falls: () => falls };
