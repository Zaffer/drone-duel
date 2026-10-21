// The MIL-STD-2525 code for the vehicles (drawn by milsymbol), and canvas markers for points.

/**
 * MIL-STD-2525E code for a friendly military unmanned aircraft (CoT type a-f-A-M-F-Q, as in
 * ATAK), marked AUT (2525E common sector 2 modifier 110, autonomous control) when
 * `autonomous`. Remotely piloted (111, RP) is left off the map: RP and AUT are hard to tell
 * apart at symbol size, so the symbol says only whether the autopilot is flying.
 */
export function sidcUav(autonomous) {
  const modifier = autonomous ? "10" : null;
  return [
    "13", // version: 2525E
    "0", // context: reality
    "3", // standard identity: friend
    "01", // symbol set: air
    "0", // status: present
    "0", // headquarters, task force, dummy: none
    "00", // amplifier: none
    "110300", // entity: military, unmanned aircraft
    "00", // sector 1 modifier: none
    modifier ?? "00", // sector 2 modifier
    "0", // sector 1 modifier set: the symbol set's own
    modifier ? "1" : "0", // sector 2 modifier set: common
    "0", // frame shape: the standard identity's
    "0000000",
  ].join("");
}

/** A round point marker with a letter, e.g. "A" or "B". */
export function pointMarker(letter, color) {
  const ratio = window.devicePixelRatio || 1;
  const size = 34;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size * ratio;
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
  const c = size / 2;
  ctx.beginPath();
  ctx.arc(c, c, c - 3, 0, Math.PI * 2);
  ctx.fillStyle = "rgba(10,14,18,0.82)";
  ctx.fill();
  ctx.lineWidth = 3;
  ctx.strokeStyle = color;
  ctx.stroke();
  ctx.fillStyle = color;
  ctx.font = "bold 17px Arial";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  ctx.fillText(letter, c, c + 1);
  return canvas;
}

/** QGroundControl's current-item green, and the grey it uses for inactive items. */
const ACTIVE_CSS = "#14a63c";
const DONE_CSS = "#585858";

/**
 * A mission point marker: an ATAK-style route waypoint disc in the drone's colour with QGC's
 * item text inside (number, T, L, H, R). `state` is "next", "active" (green, larger, as QGC
 * draws the current item) or "done" (grey). Returns a data URL and its CSS pixel size.
 *
 * @param {"waypoint" | "action" | "loiter" | "home" | "rally" | "roi"} kind
 */
export function planMarker(text, color, kind = "waypoint", state = "next") {
  const ratio = window.devicePixelRatio || 1;
  const active = state === "active";
  const size = active ? 26 : kind === "home" ? 22 : 20;
  const canvas = document.createElement("canvas");
  canvas.width = canvas.height = size * ratio;
  const ctx = canvas.getContext("2d");
  ctx.scale(ratio, ratio);
  const c = size / 2;
  const fill = active ? ACTIVE_CSS : state === "done" ? DONE_CSS : color;
  ctx.beginPath();
  if (kind === "roi") {
    ctx.moveTo(c, 2);
    ctx.lineTo(size - 2, c);
    ctx.lineTo(c, size - 2);
    ctx.lineTo(2, c);
    ctx.closePath();
  } else {
    ctx.arc(c, c, c - 2, 0, Math.PI * 2);
  }
  // Home is hollow so it reads as a place, not a point to fly to.
  ctx.fillStyle = kind === "home" ? "rgba(10,14,18,0.85)" : fill;
  ctx.fill();
  ctx.lineWidth = kind === "home" ? 3 : 1.5;
  ctx.strokeStyle = kind === "home" ? fill : "rgba(0,0,0,0.85)";
  ctx.stroke();
  if (active) {
    ctx.beginPath();
    ctx.arc(c, c, c - 0.75, 0, Math.PI * 2);
    ctx.lineWidth = 1.5;
    ctx.strokeStyle = "rgba(255,255,255,0.9)";
    ctx.stroke();
  }
  if (kind === "loiter") {
    ctx.beginPath();
    ctx.arc(c, c, c - 5, 0, Math.PI * 2);
    ctx.lineWidth = 1;
    ctx.strokeStyle = "rgba(255,255,255,0.7)";
    ctx.stroke();
  }
  if (text) {
    ctx.fillStyle = kind === "home" ? fill : "#ffffff";
    ctx.font = `bold ${text.length > 1 ? 10 : 12}px Arial`;
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, c, c + 0.5);
  }
  return { url: canvas.toDataURL(), size };
}
