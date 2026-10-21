// Drone Duel COP 3D viewer: telemetry stream, camera modes, fleet panel, FPV HUD, events.
//
// Two views look through the selected drone's camera. FPV is the COP's own: the map seen from
// the drone, on its telemetry. The camera feed is what the drone's camera itself sends: in
// the field its video; in the sim, every drone where its physics puts it (swarm.physics), with
// none of the map's symbols. Over the camera feed, ArduPilot's own OSD as the goggles draw it
// from MSP DisplayPort (swarm.osd); without it (a replay), a HUD from the telemetry instead.
//
// The Duel panel plays a match against another player's COP (duelmatch.js): in their turns
// they fly the drones at this player's truck, and the map here turns into the truck's drive.
// ?room=CODE&as=host|guest in the address opens the room on load.

import { BaseMap } from "./basemap.js";
import { Director } from "./director.js";
import { DuelMatch } from "./duelmatch.js";
import { callsign, controlMode, DEFAULT_COLOR, DRONE_COLORS, Fleet } from "./fleet.js";
import { Instruments } from "./instruments.js";
import { OsdView } from "./osd.js";
import { PlanLayer } from "./plans.js";

const C = Cesium;
const $ = (id) => document.getElementById(id);
const STALE_MS = 2000;
const OPTIONS_KEY = "drone-duel-view-options";
const DEFAULT_OPTIONS = {
  sensors: "selected", // off | selected | all
  trails: true,
  slots: true,
  labels: true,
  rangeBearing: true,
  mission: true,
  fence: true,
  controlArea: true,
  tiltDeg: 0,
  hfovDeg: 84,
  imagery: "satellite",
  /** sysid -> a DRONE_COLORS name, picked on the fleet card; DEFAULT_COLOR otherwise. */
  droneColors: {},
};
const SENSOR_MODES = ["selected", "all", "off"];
/** Each ArduCopter mode's light on a fleet card, and what it means. */
const MODE_LIGHTS = {
  manual: "a pilot flies it with no position hold",
  hold: "a pilot flies it; it holds position on GPS",
  auto: "the autopilot flies it",
  recover: "the autopilot is returning, landing, braking or avoiding",
};
const MODE_KIND = {
  STABILIZE: "manual",
  ACRO: "manual",
  ALT_HOLD: "manual",
  SPORT: "manual",
  DRIFT: "manual",
  LOITER: "hold",
  POSHOLD: "hold",
  AUTO: "auto",
  GUIDED: "auto",
  FOLLOW: "auto",
  RTL: "recover",
  LAND: "recover",
  BRAKE: "recover",
  AVOID_ADSB: "recover",
};
const SOURCE_NAMES = { google: "Google 3D Tiles", ion: "Cesium ion", esri: "Esri imagery" };

const options = loadOptions();
Object.defineProperty(options, "vfovDeg", {
  get() {
    // The camera's image has the map's shape: the FPV view fills the map with it.
    const h = C.Math.toRadians(this.hfovDeg);
    return C.Math.toDegrees(2 * Math.atan(Math.tan(h / 2) / mapAspect()));
  },
});

/** Width over height of the map, as the FPV view shows it. */
function mapAspect() {
  const canvas = viewer.scene.canvas;
  return canvas.clientWidth / canvas.clientHeight || 16 / 9;
}

const config = await fetch("/config.json").then((r) => r.json());
const map = new BaseMap($("map"), config);
const viewer = map.viewer;
const fleet = new Fleet(map, options, config);
const plans = new PlanLayer(fleet, options);
const camera = viewer.camera;
const defaultFov = camera.frustum.fov;
const director = new Director(viewer, fleet);
const duel = new DuelMatch({ fleet, map, config, toast, setDriving });

const state = {
  mode: "free", // free | follow | fpv | camera | auto
  lastFrameAt: 0,
  framed: false,
  prev: null,
};

$("source").textContent = SOURCE_NAMES[map.source];
map.ready.then(() => {
  map.setImagery(options.imagery);
  $("source").textContent = SOURCE_NAMES[map.source];
  if (map.fallbackReason) toast(map.fallbackReason, "bad");
  refreshButtons();
});

// -- telemetry ----------------------------------------------------------------

const events = new EventSource("/events");
const sameOrigin = (a, b) => a[0] === b[0] && a[1] === b[1];
events.addEventListener("history", (e) => {
  // Only the latest field's frames: older ones would place the drones on another ground.
  const frames = JSON.parse(e.data);
  const origin = frames.at(-1)?.origin;
  fleet.history(origin ? frames.filter((f) => sameOrigin(f.origin, origin)) : frames);
});
const plansSeen = new Set();
events.addEventListener("plan", (e) => {
  const plan = JSON.parse(e.data);
  if (!plans.set(plan)) return;
  // Announce uploads that happen while the page is open, not the plans it opens with.
  if (plansSeen.has(plan.sysid)) {
    const points = plan.items.filter((i) => i.nav).length;
    toast(`${callsign(plan.sysid)} plan updated: ${points} mission point${points === 1 ? "" : "s"}`);
  }
  plansSeen.add(plan.sysid);
  plans.update();
});
events.onmessage = (e) => {
  const frame = JSON.parse(e.data);
  // The stream reconnects on its own when the stack restarts. A new origin (another field, or
  // a watcher anchored elsewhere) makes every ground height, trail and camera spot stale.
  const before = state.prev?.origin;
  if (before && !sameOrigin(before, frame.origin)) {
    location.reload();
    return;
  }
  state.lastFrameAt = performance.now();
  announce(state.prev, frame);
  fleet.update(frame);
  duel.frame();
  for (const v of frame.vehicles) instruments.get(v.sysid)?.update(v);
  plans.update();
  state.prev = frame;
  if (fleet.selected === null && frame.vehicles.length) {
    fleet.selected = frame.leader ?? frame.vehicles[0].sysid;
  }
  if (!state.framed && fleet.allPositions().length) {
    state.framed = true;
    if (state.mode === "free") overview(0);
  }
};

// -- camera modes -------------------------------------------------------------

function framingSphere() {
  const pts = fleet.allPositions();
  if (options.mission) pts.push(...plans.allPositions());
  for (const e of viewer.entities.values) {
    if (e.id.startsWith?.("marker-")) pts.push(e.position.getValue(C.JulianDate.now()));
  }
  if (!pts.length) return null;
  const sphere = C.BoundingSphere.fromPoints(pts);
  sphere.radius = Math.max(sphere.radius, 60);
  return sphere;
}

/** Looking through the selected drone's camera: the FPV view or the camera feed. */
const throughCamera = () => state.mode === "fpv" || state.mode === "camera";

function freeCamera() {
  if (throughCamera()) exitCamera();
  director.stop();
  viewer.scene.screenSpaceCameraController.enableInputs = true;
  viewer.trackedEntity = undefined;
  state.mode = "free";
  refreshButtons();
}

/** Hand the map over to the duel's truck (`on`), or take it back: the COP's panels, symbols,
 * plans and keys step aside for the drive's while it lasts. */
function setDriving(on) {
  if (on) freeCamera();
  document.body.classList.toggle("driving", on);
  fleet.driving = on;
  viewer.entities.show = !on;
  plans.update();
  if (!on) overview(0.8);
}

const driving = () => document.body.classList.contains("driving");

function overview(duration = 1.2) {
  freeCamera();
  const sphere = framingSphere();
  if (!sphere) return;
  camera.flyToBoundingSphere(sphere, {
    offset: new C.HeadingPitchRange(camera.heading, C.Math.toRadians(-38), Math.max(sphere.radius * 3.0, 320)),
    duration,
  });
}

function topDown() {
  freeCamera();
  const sphere = framingSphere();
  if (!sphere) return;
  camera.flyToBoundingSphere(sphere, {
    offset: new C.HeadingPitchRange(0, C.Math.toRadians(-89.9), Math.max(sphere.radius * 3.4, 380)),
    duration: 1.2,
  });
}

function autoCamera() {
  freeCamera();
  viewer.scene.screenSpaceCameraController.enableInputs = false;
  director.start();
  state.mode = "auto";
  refreshButtons();
}

function follow() {
  const v = fleet.vehicles.get(fleet.selected);
  if (!v) return;
  if (throughCamera()) exitCamera();
  director.stop();
  viewer.scene.screenSpaceCameraController.enableInputs = true;
  v.entity.viewFrom = new C.Cartesian3(-45, -45, 30);
  viewer.trackedEntity = v.entity;
  state.mode = "follow";
  refreshButtons();
}

/** The camera feed's picture shape. Digital FPV cameras (Walksnail, HDZero, DJI) send 16:9;
 * an analog one 4:3. */
const CAMERA_ASPECT = 16 / 9;
/** Black between the camera's frame and the panels round it, px. */
const FRAME_MARGIN = 12;

/** The camera feed's frame: the largest CAMERA_ASPECT box that fits between the top bar, the
 * fleet panel and the toolbar, centred there. */
function cameraFrame() {
  const W = window.innerWidth;
  const H = window.innerHeight;
  const fleetBox = $("fleet").getBoundingClientRect();
  const toolbar = $("toolbar").getBoundingClientRect();
  const fleetRight = fleetBox.left > W / 2; // beside the map, or (narrow screens) under it
  const left = FRAME_MARGIN;
  const right = (fleetRight ? fleetBox.left : W) - FRAME_MARGIN;
  const top = $("topbar").getBoundingClientRect().bottom + FRAME_MARGIN;
  const bottom = Math.min(toolbar.top, fleetRight ? H : fleetBox.top) - FRAME_MARGIN;
  const width = Math.max(Math.min(right - left, (bottom - top) * CAMERA_ASPECT), 0);
  const height = width / CAMERA_ASPECT;
  return { left: left + (right - left - width) / 2, top: top + (bottom - top - height) / 2, width, height };
}

/** Crop the map to the camera's frame, black round it, and lay the OSD over that frame; or,
 * with `on` false, give the map back the whole window. The map's shape sets the camera's
 * vertical field of view (options.vfovDeg). */
function cropToCamera(on) {
  document.body.classList.toggle("camera-feed", on);
  for (const el of [$("map"), $("osd")]) {
    if (!on) {
      el.removeAttribute("style");
      continue;
    }
    const f = cameraFrame();
    Object.assign(el.style, {
      left: `${f.left}px`,
      top: `${f.top}px`,
      width: `${f.width}px`,
      height: `${f.height}px`,
    });
  }
}
window.addEventListener("resize", () => {
  if (state.mode === "camera") cropToCamera(true);
});

/** Look through the selected drone's camera: `mode` "fpv" or "camera" (the camera feed). */
function enterCamera(mode) {
  const v = fleet.vehicles.get(fleet.selected);
  if (!v?.currentPosition()) return;
  if (throughCamera()) exitCamera();
  viewer.trackedEntity = undefined;
  director.stop();
  state.mode = mode;
  fleet.fpv = v.sysid;
  const feed = mode === "camera";
  fleet.setFeed(feed);
  watchFeeds(feed);
  plans.update();
  viewer.scene.screenSpaceCameraController.enableInputs = false;
  document.body.classList.add("fpv-mode");
  $("fpv").hidden = false;
  $("fpv").classList.toggle("feed", feed);
  $("fpv-exit").textContent = feed ? "Exit camera feed (Esc)" : "Exit FPV (Esc)";
  cropToCamera(feed);
  refreshButtons();
}

function exitCamera() {
  fleet.fpv = null;
  fleet.setFeed(false);
  watchFeeds(false);
  osdView.clear();
  cropToCamera(false);
  plans.update();
  camera.frustum.fov = defaultFov;
  viewer.scene.screenSpaceCameraController.enableInputs = true;
  document.body.classList.remove("fpv-mode");
  $("fpv").hidden = true;
  state.mode = "free";
}

// Every drone's physics and OSD while the camera feed is open, in the sim: each physics state
// as SITL computes it, no buffer, and each OSD screen as ArduPilot draws it. Not there (a
// replay, the field), the streams fail once: the feed shows NO FEED over the telemetry.
const OSD_STALE_MS = 1000;
const osdView = new OsdView($("osd"));
/** sysid -> {screen, at}: each drone's last OSD screen and when it came. */
const osdScreens = new Map();
let feedStreams = [];
function watchFeeds(on) {
  for (const s of feedStreams) s.close();
  feedStreams = [];
  osdScreens.clear();
  if (!on) return;
  const physics = new EventSource("/physics");
  physics.onmessage = (e) => fleet.physics(JSON.parse(e.data));
  const osd = new EventSource("/osd");
  osd.onmessage = (e) => {
    const at = performance.now();
    for (const [sysid, screen] of Object.entries(JSON.parse(e.data))) osdScreens.set(Number(sysid), { screen, at });
  };
  feedStreams = [physics, osd];
}

/** The camera feed's drone's OSD screen, while it is coming; null otherwise. */
function liveOsd() {
  const shot = osdScreens.get(fleet.fpv);
  return shot && performance.now() - shot.at < OSD_STALE_MS ? shot.screen : null;
}

viewer.scene.preRender.addEventListener(() => {
  if (!throughCamera()) return;
  const pose = fleet.vehicles.get(fleet.fpv)?.cameraPose(state.mode === "camera");
  if (!pose) return;
  // Cesium's fov is the wider side's: horizontal on a wide map, vertical on a tall one. Either
  // way the view spans hfovDeg across and vfovDeg up, as the camera's view lines on the map.
  camera.frustum.fov = C.Math.toRadians(mapAspect() > 1 ? options.hfovDeg : options.vfovDeg);
  // The drone's own camera: it pitches, rolls and yaws with the frame.
  const { f, u } = pose.axes;
  const enu = C.Transforms.eastNorthUpToFixedFrame(pose.pos);
  const world = (a) => C.Matrix4.multiplyByPointAsVector(enu, new C.Cartesian3(...a), new C.Cartesian3());
  camera.setView({ destination: pose.pos, orientation: { direction: world(f), up: world(u) } });
});

viewer.scene.postRender.addEventListener(() => {
  $("compass").querySelector("svg").style.transform = `rotate(${-C.Math.toDegrees(camera.heading)}deg)`;
  if (throughCamera()) drawFpvHud();
});

$("compass").addEventListener("click", () => {
  if (state.mode !== "free") return;
  const sphere = framingSphere();
  const center = sphere?.center;
  if (!center) return;
  camera.flyToBoundingSphere(new C.BoundingSphere(center, 1), {
    offset: new C.HeadingPitchRange(0, camera.pitch, C.Cartesian3.distance(camera.positionWC, center)),
    duration: 0.8,
  });
});

// -- FPV HUD ------------------------------------------------------------------

function drawFpvHud() {
  // Over the camera feed, ArduPilot's own OSD when it is coming, in place of the HUD.
  const screen = state.mode === "camera" ? liveOsd() : null;
  $("fpv").classList.toggle("osd", screen !== null);
  if (screen) {
    osdView.draw(screen);
    return;
  }
  osdView.clear();
  const v = fleet.vehicles.get(fleet.fpv);
  if (!v?.data) return;
  const d = v.data;
  // Standing in for the OSD over the camera feed: the autopilot's latest numbers, unsmoothed.
  const osd = state.mode === "camera";
  const hdg = (C.Math.toDegrees(osd ? d.heading : v.currentHeading()) + 360) % 360;
  $("fpv-callsign").textContent = `${callsign(d.sysid)} · ${d.armed ? d.role.toUpperCase() : "ON GROUND"}`;
  $("fpv-time").textContent = dtg(new Date());
  $("fpv-alt").textContent = (osd ? d.alt : v.currentAlt()).toFixed(1);
  $("fpv-spd").textContent = Math.hypot(d.vn, d.ve).toFixed(1);
  $("fpv-hdg").textContent = `HDG ${String(Math.round(hdg) % 360).padStart(3, "0")}°`;
  $("fpv-mode").textContent = [d.mode, controlMode(d)].filter(Boolean).join(" · ");
  const cam = `CAM ${options.tiltDeg}° · ${options.hfovDeg}° FOV`;
  if (state.mode !== "camera") {
    $("fpv-cam").textContent = cam;
    return;
  }
  const physical = v.cameraPose(true)?.physical;
  $("fpv-cam").textContent = `${cam} · ${physical ? "SIM CAMERA" : "NO FEED"}`;
  $("fpv-cam").classList.toggle("no-feed", !physical);
}

// -- drone colours --------------------------------------------------------------

const colorPicker = document.createElement("div");
colorPicker.className = "color-picker";
colorPicker.hidden = true;
colorPicker.setAttribute("role", "dialog");
colorPicker.innerHTML =
  `<div class="color-picker-title"></div><div class="color-picker-grid">` +
  Object.entries(DRONE_COLORS)
    .map(
      ([name, css]) =>
        `<button data-color="${name}" title="${name}" aria-label="${name}" style="background:${css}"></button>`,
    )
    .join("") +
  `</div>`;
document.body.append(colorPicker);
let colorPickerSysid = null;

/** Show the colour choices for drone `sysid` beside its card's colour bar. */
function openColorPicker(sysid, swatch) {
  if (colorPickerSysid === sysid && !colorPicker.hidden) return closeColorPicker();
  colorPickerSysid = sysid;
  colorPicker.querySelector(".color-picker-title").textContent = `${callsign(sysid)} colour`;
  const picked = fleet.droneColor(sysid);
  for (const b of colorPicker.querySelectorAll("[data-color]")) {
    b.classList.toggle("on", b.dataset.color === picked);
  }
  colorPicker.hidden = false;
  const bar = swatch.getBoundingClientRect();
  const box = colorPicker.getBoundingClientRect();
  colorPicker.style.left = `${Math.max(8, bar.left - box.width - 10)}px`;
  colorPicker.style.top = `${Math.min(Math.max(8, bar.top), window.innerHeight - box.height - 8)}px`;
  colorPicker.querySelector(".on")?.focus();
}

function closeColorPicker() {
  colorPicker.hidden = true;
  colorPickerSysid = null;
}

colorPicker.addEventListener("click", (e) => {
  const b = e.target.closest("[data-color]");
  if (!b || colorPickerSysid === null) return;
  if (b.dataset.color === DEFAULT_COLOR) delete options.droneColors[colorPickerSysid];
  else options.droneColors[colorPickerSysid] = b.dataset.color;
  saveOptions();
  closeColorPicker();
  renderCards(true);
});
document.addEventListener("pointerdown", (e) => {
  if (!colorPicker.hidden && !colorPicker.contains(e.target) && !e.target.closest(".swatch")) {
    closeColorPicker();
  }
});

// -- picking and cursor ---------------------------------------------------------

const handler = new C.ScreenSpaceEventHandler(viewer.scene.canvas);
handler.setInputAction((click) => {
  const picked = viewer.scene.pick(click.position);
  const id = picked?.id?.id;
  if (typeof id === "string" && id.startsWith("vehicle-")) select(Number(id.slice(8)));
}, C.ScreenSpaceEventType.LEFT_CLICK);

// Double click: follow a drone, or fly halfway in toward the ground point.
handler.setInputAction((click) => {
  if (state.mode === "auto" || throughCamera() || driving()) return;
  const id = viewer.scene.pick(click.position)?.id?.id;
  if (typeof id === "string" && id.startsWith("vehicle-")) {
    select(Number(id.slice(8)));
    follow();
    return;
  }
  const carto = map.pickGround(click.position);
  if (!carto) return;
  const point = C.Cartographic.toCartesian(carto);
  const range = Math.max(C.Cartesian3.distance(camera.positionWC, point) / 2, 30);
  freeCamera();
  camera.flyToBoundingSphere(new C.BoundingSphere(point, 0), {
    offset: new C.HeadingPitchRange(camera.heading, camera.pitch, range),
    duration: 0.8,
  });
}, C.ScreenSpaceEventType.LEFT_DOUBLE_CLICK);

let cursorPending = null;
handler.setInputAction((move) => {
  cursorPending = move.endPosition;
}, C.ScreenSpaceEventType.MOUSE_MOVE);

setInterval(() => {
  if (!cursorPending) return;
  const carto = map.pickGround(cursorPending);
  cursorPending = null;
  if (!carto) return;
  const lat = C.Math.toDegrees(carto.latitude);
  const lon = C.Math.toDegrees(carto.longitude);
  $("cursor-mgrs").textContent = formatMgrs(lon, lat);
  $("cursor-ll").textContent = `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
  $("cursor-elev").textContent = `${Math.round(carto.height)} m`;
}, 100);

function formatMgrs(lon, lat) {
  try {
    const m = mgrs.forward([lon, lat], 5);
    // 18TVR1234567890 -> 18T VR 12345 67890
    const g = m.match(/^(\d{1,2}[C-X])([A-Z]{2})(\d{5})(\d{5})$/);
    return g ? `${g[1]} ${g[2]} ${g[3]} ${g[4]}` : m;
  } catch {
    return "—";
  }
}

function select(sysid) {
  if (!fleet.vehicles.has(sysid)) return;
  fleet.selected = sysid;
  fleet.selectAll = false;
  refreshButtons();
  if (state.mode === "follow") follow();
  if (throughCamera()) fleet.fpv = sysid;
  renderCards(true);
}

// -- panels ---------------------------------------------------------------------

/** Each card's dials, by sysid. Updated on every frame; the rest of the card 4 times a second. */
const instruments = new Map();

let lastCards = 0;
function renderCards(force = false) {
  const now = performance.now();
  if (!force && now - lastCards < 250) return;
  lastCards = now;
  const frame = state.prev;
  if (!frame) return;
  const cards = $("cards");
  $("fleet-count").textContent = `· ${frame.vehicles.filter((v) => v.armed).length}/${frame.vehicles.length} ARMED`;
  for (const v of frame.vehicles) {
    let card = cards.querySelector(`[data-sysid="${v.sysid}"]`);
    if (!card) {
      card = document.createElement("button");
      card.className = "card";
      card.dataset.sysid = v.sysid;
      card.innerHTML = `<span class="swatch"></span>
        <span class="name"><span class="callsign"></span><span class="ctl"></span>
        <span class="mode"><i class="light"></i><span class="mode-name"></span></span></span><span class="role"></span>
        <span class="battery"><span class="level"><span></span></span><span class="volts"></span></span>`;
      const dials = new Instruments();
      dials.svg.classList.add("card-instruments");
      card.insertBefore(dials.svg, card.querySelector(".battery"));
      dials.update(v);
      instruments.set(v.sysid, dials);
      const swatch = card.querySelector(".swatch");
      swatch.addEventListener("click", (e) => {
        e.stopPropagation();
        openColorPicker(v.sysid, swatch);
      });
      swatch.addEventListener("dblclick", (e) => e.stopPropagation());
      card.addEventListener("click", () => select(v.sysid));
      card.addEventListener("dblclick", () => {
        select(v.sysid);
        follow();
      });
      cards.append(card);
      [...cards.children]
        .sort((a, b) => a.dataset.sysid - b.dataset.sysid)
        .forEach((c) => cards.append(c));
    }
    card.classList.toggle("selected", fleet.isSelected(v.sysid));
    const swatch = card.querySelector(".swatch");
    swatch.style.background = fleet.droneCss(v.sysid);
    swatch.title = `${fleet.droneColor(v.sysid)} · click to change colour`;
    card.querySelector(".callsign").textContent = callsign(v.sysid);
    const ctl = card.querySelector(".ctl");
    const how = controlMode(v);
    ctl.textContent = how ?? "";
    ctl.hidden = how === null;
    ctl.className = `ctl ${how?.toLowerCase() ?? ""}`;
    ctl.title =
      how === "RP" ? "remotely piloted: a pilot is flying it" : "autonomous: the autopilot is flying it";
    const role = card.querySelector(".role");
    const roleText = v.armed ? v.role : "ground";
    role.textContent = roleText.toUpperCase();
    role.className = `role ${roleText}`;
    const kind = MODE_KIND[v.mode] ?? "unknown";
    const mode = card.querySelector(".mode");
    mode.className = `mode ${kind}${v.armed ? "" : " disarmed"}`;
    mode.title = `${v.mode}: ${MODE_LIGHTS[kind] ?? "mode unknown"}${v.armed ? "" : " (disarmed)"}`;
    card.querySelector(".mode-name").textContent = v.mode;
    // The battery: remaining as a bar, as ArduPilot estimates it, and the voltage it measures.
    const bat = card.querySelector(".battery .level span");
    const pct = v.battery ?? 0;
    bat.style.width = `${pct}%`;
    bat.style.background = pct > 40 ? "var(--ok)" : pct > 20 ? "var(--warn)" : "var(--bad)";
    const volts = v.battery_v ?? null;
    card.querySelector(".volts").textContent = volts === null ? "— V" : `${volts.toFixed(1)} V`;
    card.querySelector(".battery").title =
      `battery ${volts === null ? "voltage unknown" : `${volts.toFixed(2)} V`}, ` +
      `${v.battery === null ? "remaining unknown" : `${v.battery}% left`}`;
  }
}

function renderStatus() {
  const age = performance.now() - state.lastFrameAt;
  const link = $("link");
  const stale = !state.lastFrameAt || age > STALE_MS;
  link.className = `chip ${stale ? "stale" : config.replay ? "replay" : "live"}`;
  $("link-text").textContent = stale
    ? state.lastFrameAt
      ? "NO TELEMETRY"
      : "WAITING"
    : config.replay
      ? `REPLAY · ${config.replay}`
      : "LIVE";
  $("phase").textContent = state.prev?.phase ?? "—";
  $("clock").textContent = dtg(new Date());
  renderCards();
}
setInterval(renderStatus, 250);

/** Date-time group, e.g. 291432Z SEP 26. */
function dtg(d) {
  const p = (n) => String(n).padStart(2, "0");
  const mon = d.toLocaleString("en", { month: "short", timeZone: "UTC" }).toUpperCase();
  return `${p(d.getUTCDate())}${p(d.getUTCHours())}${p(d.getUTCMinutes())}${p(d.getUTCSeconds())}Z ${mon} ${String(d.getUTCFullYear()).slice(2)}`;
}

// -- events ---------------------------------------------------------------------

function toast(text, kind = "") {
  const el = document.createElement("div");
  el.className = `toast ${kind}`;
  el.textContent = text;
  const box = $("toasts");
  box.prepend(el);
  while (box.children.length > 5) box.lastChild.remove();
  setTimeout(() => el.classList.add("fade"), 5500);
  setTimeout(() => el.remove(), 6200);
}

function announce(prev, frame) {
  if (!prev) return;
  if (frame.phase !== prev.phase) toast(`Phase: ${frame.phase}`);
  if (frame.leader !== prev.leader) {
    if (frame.leader === null) toast(`${callsign(prev.leader)} no longer leading`, "leader");
    else toast(`${callsign(frame.leader)} is now leader`, "leader");
  }
  if (Boolean(frame.target) !== Boolean(prev.target)) {
    toast(frame.target ? "Point B tracker reporting" : "Point B tracker lost", frame.target ? "" : "bad");
  }
  const before = new Map(prev.vehicles.map((v) => [v.sysid, v]));
  for (const v of frame.vehicles) {
    const p = before.get(v.sysid);
    if (!p) continue;
    const name = callsign(v.sysid);
    if (v.armed !== p.armed) toast(`${name} ${v.armed ? "armed" : "disarmed"}`, v.armed ? "" : "bad");
    else if (v.role !== p.role && v.role === "landing") toast(`${name} landing at B`, "bad");
    else if (v.role !== p.role && v.role === "parked" && p.role !== "idle") toast(`${name} holding at A`);
    else if (v.mode !== p.mode && v.armed) toast(`${name} → ${v.mode}`);
  }
}

// -- toolbar and keys -----------------------------------------------------------

const actions = {
  overview: () => overview(),
  top: topDown,
  follow: () => (state.mode === "follow" ? freeCamera() : follow()),
  fpv: () => (state.mode === "fpv" ? freeCamera() : enterCamera("fpv")),
  camera: () => (state.mode === "camera" ? freeCamera() : enterCamera("camera")),
  auto: () => (state.mode === "auto" ? freeCamera() : autoCamera()),
  exit: freeCamera,
  duel: () => duel.togglePanel(),
  selectAll: () => {
    fleet.selectAll = !fleet.selectAll;
    refreshButtons();
    renderCards(true);
  },
  imagery: () => {
    options.imagery = options.imagery === "satellite" ? "streets" : "satellite";
    map.setImagery(options.imagery);
    saveOptions();
    refreshButtons();
  },
};

document.addEventListener("click", (e) => {
  const btn = e.target.closest("button");
  if (!btn) return;
  if (btn.dataset.action) actions[btn.dataset.action]?.();
  if (btn.dataset.toggle) toggle(btn.dataset.toggle);
  if (btn.dataset.cycle === "sensors") cycleSensors();
});

document.addEventListener("keydown", (e) => {
  if (e.target instanceof HTMLInputElement || driving()) return;
  const key = e.key.toLowerCase();
  if (key === "escape" && !colorPicker.hidden) return closeColorPicker();
  if (key === "0") actions.selectAll();
  else if (/^[1-9]$/.test(key)) select(Number(key));
  else if (key === "o") overview();
  else if (key === "t") topDown();
  else if (key === "f") actions.follow();
  else if (key === "v") actions.fpv();
  else if (key === "c") actions.camera();
  else if (key === "a") actions.auto();
  else if (key === "s") cycleSensors();
  else if (key === "l") toggle("labels");
  else if (key === "r") toggle("rangeBearing");
  else if (key === "m") toggle("mission");
  else if (key === "g") toggle("fence");
  else if (key === "k") toggle("controlArea");
  else if (key === "d") actions.duel();
  else if (key === "escape") freeCamera();
});

for (const [id, key] of [
  ["tilt", "tiltDeg"],
  ["fov", "hfovDeg"],
]) {
  const input = $(id);
  input.value = options[key];
  $(`${id}-out`).textContent = `${options[key]}°`;
  input.addEventListener("input", () => {
    options[key] = Number(input.value);
    $(`${id}-out`).textContent = `${options[key]}°`;
    saveOptions();
  });
}

const SENSOR_LABELS = { selected: "Sensor", all: "Sensors ×all", off: "Sensors" };

function toggle(name) {
  options[name] = !options[name];
  saveOptions();
  refreshButtons();
}

function cycleSensors() {
  const i = SENSOR_MODES.indexOf(options.sensors);
  options.sensors = SENSOR_MODES[(i + 1) % SENSOR_MODES.length];
  saveOptions();
  refreshButtons();
}

function refreshButtons() {
  for (const btn of document.querySelectorAll("[data-toggle]")) {
    btn.classList.toggle("on", Boolean(options[btn.dataset.toggle]));
  }
  $("select-all").classList.toggle("on", fleet.selectAll);
  const sensorBtn = document.querySelector('[data-cycle="sensors"]');
  sensorBtn.classList.toggle("on", options.sensors !== "off");
  sensorBtn.textContent = SENSOR_LABELS[options.sensors];
  document.querySelector('[data-action="follow"]').classList.toggle("on", state.mode === "follow");
  document.querySelector('[data-action="fpv"]').classList.toggle("on", state.mode === "fpv");
  document.querySelector('[data-action="camera"]').classList.toggle("on", state.mode === "camera");
  document.querySelector('[data-action="auto"]').classList.toggle("on", state.mode === "auto");
  $("imagery-btn").textContent = options.imagery === "satellite" ? "Satellite" : "Streets";
  $("imagery-btn").disabled = map.groundIsTiles;
  $("sensor-controls").style.display =
    options.sensors !== "off" || throughCamera() ? "" : "none";
}
refreshButtons();

function loadOptions() {
  try {
    const stored = JSON.parse(localStorage.getItem(OPTIONS_KEY) ?? "{}");
    // Every option used to be saved, defaults included: drop the old default tilt so the
    // level FPV camera applies.
    if (stored.tiltDeg === -50) delete stored.tiltDeg;
    const saved = { ...DEFAULT_OPTIONS, ...stored };
    if (!SENSOR_MODES.includes(saved.sensors)) saved.sensors = DEFAULT_OPTIONS.sensors;
    saved.droneColors = Object.fromEntries(
      Object.entries(stored.droneColors ?? {}).filter(([, name]) => name in DRONE_COLORS),
    );
    return saved;
  } catch {
    return { ...DEFAULT_OPTIONS, droneColors: {} };
  }
}

function saveOptions() {
  try {
    // Only what differs from the defaults, so a new default reaches everyone.
    const changed = Object.fromEntries(
      Object.keys(DEFAULT_OPTIONS)
        .filter((key) => JSON.stringify(options[key]) !== JSON.stringify(DEFAULT_OPTIONS[key]))
        .map((key) => [key, options[key]]),
    );
    localStorage.setItem(OPTIONS_KEY, JSON.stringify(changed));
  } catch {
    // Storage blocked: options last for this page only.
  }
}

// For headless checks and the browser console.
window.swarmView = {
  viewer,
  fleet,
  plans,
  options,
  state,
  overview,
  topDown,
  follow,
  enterCamera,
  autoCamera,
  freeCamera,
  select,
  duel,
};
