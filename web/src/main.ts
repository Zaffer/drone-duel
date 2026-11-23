// Drone Duels: the page. Loads the content, config and prefs, opens the link to the server,
// builds the world, and hands over to the UI (ui/app.ts) and the flight loop (flight.ts).

import "./cesium-base.ts";
import "./theme/base.css";
import "./ui/ui.css";
import { effect } from "alien-signals";
import * as Cesium from "cesium";
import { perf } from "./hud/perf.ts";
import { radio } from "./input/radio.ts";
import { api } from "./net/api.ts";
import { link } from "./net/server.ts";
import { config, content, location, screen } from "./state/app.ts";
import { loadPrefs, prefs } from "./state/prefs.ts";
import { wireSession } from "./state/session.ts";
import { mountApp } from "./ui/app.ts";
import { World } from "./world/world.ts";

async function boot(): Promise<void> {
  const [c, cfg] = await Promise.all([api.content(), api.config()]);
  content(c);
  config(cfg);
  await loadPrefs();
  wireSession();
  link.open();
  const world = new World(document.getElementById("world")!, cfg.map_keys);
  // The sun: the time of day set in Graphics, at the place chosen.
  effect(() => world.setTimeOfDay(prefs().graphics.timeOfDay, location()?.origin[1] ?? 0));
  effect(() => world.setDetail(prefs().graphics.quality));
  // For the console and the browser tests: the world, the link, the controller, the frame stats,
  // and Cesium itself (to time its insides).
  (window as unknown as { duels: unknown }).duels = { world, link, radio, perf, Cesium };
  mountApp(document.getElementById("app")!, world);
  screen("menu");
}

/** What the page cannot run without, if this browser lacks any of it: native Observables
 * (DESIGN.md §9; Chrome and the browsers built on it have them, Safari and Firefox not yet, and
 * on an iPhone or iPad every browser is Safari underneath). */
function missing(): string | null {
  const observables = typeof Observable === "function" && "when" in EventTarget.prototype;
  return observables ? null : "Observables";
}

const lacks = missing();
if (lacks) {
  // Said plainly, rather than a stack trace from the first place that needed it.
  const note = document.createElement("div");
  note.className = "stopped";
  const title = document.createElement("h1");
  title.textContent = "Drone Duels needs Chrome";
  const why = document.createElement("p");
  why.textContent = `This browser has no ${lacks}, which the game is built on. Chrome, Edge and the other Chromium browsers have them, on a computer or an Android phone; Safari and Firefox do not yet, and on an iPhone or iPad every browser is Safari underneath.`;
  why.style.cssText = "max-width: 34em; padding: 0 20px";
  note.append(title, why);
  document.getElementById("app")!.append(note);
} else {
  boot().catch((err: unknown) => {
    console.error(err);
    document.body.insertAdjacentHTML(
      "beforeend",
      `<pre style="position:fixed;inset:auto 16px 16px;color:#ff4f6e;white-space:pre-wrap">Drone Duels failed to start: ${String(err)}</pre>`,
    );
  });
}
