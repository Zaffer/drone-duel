// Drive a car on the map's Google Photorealistic 3D Tiles, with physics and collisions
// (drivesession.js). Start somewhere else with ?at=LAT,LON[,HEADING_DEG], or double-click the
// map to move the car there. The duel's driver drives in the COP instead (dueldriver.js).

import { BaseMap } from "./basemap.js";
import { DriveSession } from "./drivesession.js";

const $ = (id) => document.getElementById(id);

const config = await fetch("/config.json").then((r) => r.json());
const map = new BaseMap($("map"), config);
const notice = (text) => {
  $("notice").hidden = !text;
  $("notice").textContent = text ?? "";
};

const params = new URLSearchParams(location.search);
const [lat, lon, headingDeg] = params.has("at")
  ? params.get("at").split(",").map(Number)
  : [config.markers?.[0]?.lat ?? 45.319287, config.markers?.[0]?.lon ?? -75.758596, 0];

await map.ready;
if (!map.groundIsTiles) {
  notice(`Driving needs Google 3D Tiles: set GOOGLE_MAPS_API_KEY. ${map.fallbackReason ?? ""}`);
  throw new Error("no 3D tiles");
}

notice("Finding the ground…");
const hud = {
  speed: $("speed"),
  wheels: $("wheels"),
  tiles: $("tiles"),
  tris: $("tris"),
  pending: $("pending"),
  buildMs: $("build-ms"),
  physMs: $("phys-ms"),
  fps: $("fps"),
};
const session = await DriveSession.start(map, {
  lat,
  lon,
  headingDeg: headingDeg || 0,
  ui: { notice, hud, camMode: $("cam-mode"), help: $("help") },
});
session.dblClickMoves = true;
session.onFarMove = (toLat, toLon) => {
  location.search = `?at=${toLat.toFixed(6)},${toLon.toFixed(6)}`;
};

window.drive = { session, viewer: map.viewer };
