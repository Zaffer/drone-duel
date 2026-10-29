// Drone Duels: the page. Loads the content, config and prefs, opens the link to the server,
// builds the world, and hands over to the UI (ui/app.ts) and the flight loop (flight.ts).

import "./cesium-base.ts";
import "./theme/base.css";
import "./ui/ui.css";
import { radio } from "./input/radio.ts";
import { api } from "./net/api.ts";
import { link } from "./net/server.ts";
import { config, content, screen } from "./state/app.ts";
import { loadPrefs } from "./state/prefs.ts";
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
  // For the console and the browser tests: the world, the link and the controller.
  (window as unknown as { duels: unknown }).duels = { world, link, radio };
  mountApp(document.getElementById("app")!, world);
  screen("menu");
}

boot().catch((err: unknown) => {
  console.error(err);
  document.body.insertAdjacentHTML(
    "beforeend",
    `<pre style="position:fixed;inset:auto 16px 16px;color:#ff4f6e">Drone Duels failed to start: ${String(err)}</pre>`,
  );
});
