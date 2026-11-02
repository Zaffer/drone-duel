// The game's screens (DESIGN.md §11): the main menu (mode), the place picker, a mode's lobby
// (Duel, Race: where the vehicle is picked), and flying (ui/hud.ts, with the pause menu,
// ui/pause.ts, whose Vehicle tab switches vehicle). While in the menus the world turns slowly
// round the chosen place.

import { effect } from "alien-signals";
import * as C from "cesium";
import { sounds } from "../audio/sounds.ts";
import { Flight, type ModeRuntime } from "../flight.ts";
import { radio } from "../input/radio.ts";
import { MODES, modePlaces, modeVehicles } from "../modes/registry.ts";
import type { ModeContext, ModeDef } from "../modes/types.ts";
import type { Location, Vehicle } from "../net/protocol.ts";
import { link } from "../net/server.ts";
import {
  content,
  location,
  locationId,
  mode,
  paused,
  screen,
  toast,
  toasts,
  vehicleId,
  wind,
} from "../state/app.ts";
import { controllerName, inputSource } from "../state/input.ts";
import { frames } from "../streams/time.ts";
import type { World } from "../world/world.ts";
import { h, render, untracked } from "./h.ts";
import { hudSlots, mountHud } from "./hud.ts";
import { icons } from "./icons.ts";
import { openPause } from "./pause.ts";

const svg = (markup: string): HTMLElement => {
  const span = h("span", { class: "icon" });
  span.innerHTML = markup;
  return span;
};

export function mountApp(root: HTMLElement, world: World): void {
  let teardown: (() => void) | null = null;
  let flight: Flight | null = null;
  let orbit: AbortController | null = null;
  radio.start();
  // The channels for the UI (the Controls tab's bars, the HUD), in the menus as in flight.
  frames().subscribe(() => radio.publish());
  window.addEventListener("pointerdown", () => sounds.unlock(), { once: true });
  window.addEventListener("keydown", () => sounds.unlock(), { once: true });

  // The menus' backdrop: the world turning round the chosen place.
  const startOrbit = () => {
    orbit?.abort();
    orbit = new AbortController();
    let heading = 0.6;
    let last = 0;
    let target: C.Cartesian3 | null = null;
    let at = "";
    frames().subscribe(
      (now) => {
        const loc = location() ?? Object.values(content()?.locations ?? {})[0];
        if (!loc) return;
        if (loc.id !== at) {
          at = loc.id;
          target = null;
          void world.groundHeightSettled(loc.origin[0], loc.origin[1], 6).then((g) => {
            if (at === loc.id) target = C.Cartesian3.fromDegrees(loc.origin[1], loc.origin[0], g);
          });
        }
        const dt = last ? (now - last) / 1000 : 0;
        last = now;
        heading += dt * 0.035;
        if (target) {
          world.viewer.camera.lookAt(
            target,
            new C.HeadingPitchRange(heading, C.Math.toRadians(-24), 520),
          );
        }
      },
      { signal: orbit.signal },
    );
  };
  const stopOrbit = () => {
    orbit?.abort();
    orbit = null;
    world.viewer.camera.lookAtTransform(C.Matrix4.IDENTITY);
  };

  const menuChips = () => {
    const server = h("span", { class: "chip" }, h("span", { class: "dot" }), "Server");
    const ctl = h("span", { class: "chip" }, h("span", { class: "dot" }), "");
    const map = h("span", { class: "chip" }, h("span", { class: "dot" }), "");
    const dispose = effect(() => {
      server.className = `chip ${link.connected() ? "good" : "bad"}`;
      const name = controllerName();
      ctl.className = `chip ${name ? "good" : "warn"}`;
      ctl.lastChild!.textContent = name
        ? `${name}${inputSource() === "server" ? " (via Linux)" : ""}`
        : "No controller: move a stick";
    });
    void world.ready.then(() => {
      map.className = `chip ${world.fallbackReason ? "warn" : "good"}`;
      const globe = { ion: "Cesium ion", esri: "Esri imagery" }[world.globeSource];
      map.lastChild!.textContent =
        world.fallbackReason ?? (world.hasTiles ? `${globe}; Google 3D Tiles in flight` : globe);
    });
    return { node: h("div", { class: "status-row" }, server, ctl, map), dispose };
  };

  const showMenu = () => {
    teardown?.();
    // The menus' backdrop is the free globe; Google's tiles are billed, so only flights use them.
    void world.useTiles(false);
    if (!orbit) startOrbit();
    const main = h("div", { class: "menu-main" });
    const modeButtons = MODES.map((m) =>
      h(
        "button",
        { class: "mode-btn", onclick: () => pickMode(m, main, modeButtons) },
        svg(icons[m.id]),
        h("span", null, h("div", { class: "title" }, m.name), h("div", { class: "desc" }, m.desc)),
        h("span", { class: "kbd" }, String(MODES.indexOf(m) + 1)),
      ),
    );
    const chips = menuChips();
    const brand = h("div", { class: "brand" });
    brand.innerHTML = `${icons.logo}<div><h1>DRONE <span>DUELS</span></h1><p>Real firmware. Real places. Fly, duel, race.</p></div>`;
    const options = h(
      "button",
      { class: "mode-btn", onclick: () => openPause(root, null, "controls") },
      svg(icons.options),
      h(
        "span",
        null,
        h("div", { class: "title" }, "Options"),
        h("div", { class: "desc" }, "Controls, keys and how to fly, video, wind, graphics, audio."),
      ),
      h("span", { class: "kbd" }, "O"),
    );
    const side = h(
      "div",
      { class: "menu-side" },
      brand,
      h("div", { class: "modes" }, modeButtons, options),
      chips.node,
    );
    const toastBox = h("div", { class: "toasts" });
    const disposeToasts = effect(() =>
      toastBox.replaceChildren(
        ...toasts().map((t) => h("div", { class: `toast ${t.tone}` }, t.text)),
      ),
    );
    const node = h("div", { class: "menu" }, side, main, toastBox);
    render(root, node);
    const keys = (e: KeyboardEvent) => {
      const i = Number(e.key) - 1;
      const m = MODES[i];
      if (document.activeElement?.tagName === "INPUT" || root.querySelector(".pause")) return;
      if (m) pickMode(m, main, modeButtons);
      if (e.code === "KeyO") openPause(root, null, "controls");
      if (e.key === "Escape") {
        mode(null);
        render(main);
        modeButtons.forEach((b) => b.classList.remove("active"));
      }
    };
    window.addEventListener("keydown", keys);
    teardown = () => {
      chips.dispose();
      disposeToasts();
      window.removeEventListener("keydown", keys);
      lobbyTeardown?.();
      lobbyTeardown = null;
    };
    const current = MODES.find((m) => m.id === mode());
    if (current) pickMode(current, main, modeButtons);
  };

  let lobbyTeardown: (() => void) | null = null;

  const pickMode = (m: ModeDef, main: HTMLElement, buttons: HTMLElement[]) => {
    sounds.click();
    mode(m.id);
    buttons.forEach((b, i) => b.classList.toggle("active", MODES[i] === m));
    lobbyTeardown?.();
    lobbyTeardown = null;
    const all = modePlaces(m);
    const cards = all.map((loc) => locationCard(loc, () => pickLocation(m, loc, main)));
    render(
      main,
      h(
        "div",
        { class: "picker" },
        h("h2", null, `${m.name}: choose a place`),
        h("div", { class: "cards" }, cards),
      ),
    );
    const current = all.find((loc) => loc.id === locationId()) ?? all[0];
    if (current) {
      locationId(current.id);
      cards.forEach((c, i) => c.classList.toggle("selected", all[i] === current));
    }
  };

  const locationCard = (loc: Location, onPick: () => void) => {
    const gates = Object.values(content()?.courses ?? {})
      .filter((c) => c.location === loc.id)
      .map((c) => h("span", { class: "tag" }, `${c.gates.length} gates`));
    const card = h(
      "button",
      {
        class: "card",
        onclick: onPick,
        onmouseenter: () => locationId(loc.id),
      },
      h("div", { class: "name" }, loc.name),
      h("div", { class: "blurb" }, loc.blurb),
      h("div", { class: "tags" }, ...(mode() === "race" ? gates : [])),
    );
    card.prepend(h("span", { class: "glyph" }, svg(icons.pin)));
    return card;
  };

  const pickLocation = (m: ModeDef, loc: Location, main: HTMLElement) => {
    sounds.click();
    locationId(loc.id);
    wind(loc.wind);
    const vehicles = modeVehicles(m);
    const ctx: ModeContext = {
      location: loc,
      vehicles,
      fly: (vehicle, pad, attach) => fly(loc, vehicle, pad, attach),
      quit: () => {
        flight?.dispose();
        flight = null;
        screen("menu");
      },
    };
    if (m.lobby) {
      lobbyTeardown?.();
      const host = h("div", { class: "picker" });
      render(main, host);
      lobbyTeardown = m.lobby(host, ctx);
      return;
    }
    // Free Fly: the vehicle last flown (at first the quad), switched in flight (the Vehicle tab).
    const chosen =
      vehicles.find((v) => v.id === vehicleId()) ??
      vehicles.find((v) => v.kind === "multirotor") ??
      vehicles[0];
    if (chosen) fly(loc, chosen.id, 0, (f) => m.run!(f));
  };

  const fly = (loc: Location, vehicle: string, pad: number, attach: (f: Flight) => ModeRuntime) => {
    locationId(loc.id);
    flight?.dispose();
    teardown?.();
    teardown = null;
    stopOrbit();
    void world.useTiles(true);
    screen("flying");
    paused(false);
    try {
      const slots = hudSlots();
      const f = new Flight(world, slots);
      flight = f;
      teardown = mountHud(root, f, slots, () => {
        f.dispose();
        flight = null;
        screen("menu");
      });
      // Started first, so the mode sees this session starting, not the last one's "ready".
      f.start(loc.id, vehicle, pad);
      f.mode = attach(f);
    } catch (err) {
      console.error("could not start the flight", err);
      flight?.dispose();
      flight = null;
      screen("menu");
      toast(`Could not start: ${err instanceof Error ? err.message : String(err)}`, "bad", 8000);
    }
  };

  // Only the screen redraws the menu: what the menu reads and writes as it draws (the mode, the
  // place, prefs) is not this effect's, or it would be left pending and redraw on any change.
  effect(() => {
    if (screen() === "menu") untracked(showMenu);
  });
}

export function vehicleCard(v: Vehicle, onPick: () => void): HTMLButtonElement {
  const fw = { "ardupilot-copter": "ArduCopter 4.7", inav: "INAV 9.1", none: "no FC" }[
    v.firmware.kind
  ];
  const phys = { quad: "SkyFlow racer5in", wing: "strips + VLM", "browser-truck": "Rapier" }[
    v.physics.kind
  ];
  const card = h(
    "button",
    { class: "card", onclick: onPick },
    h("div", { class: "name" }, v.name),
    h("div", { class: "blurb" }, v.blurb),
    h("div", { class: "tags" }, h("span", { class: "tag" }, fw), h("span", { class: "tag" }, phys)),
  );
  const glyph = h("span", { class: "glyph" });
  glyph.innerHTML = icons[v.kind];
  card.prepend(glyph);
  return card;
}
