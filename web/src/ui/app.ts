// The game's screens (DESIGN.md §11): the main menu, three columns picked in turn (scenario,
// location, vehicle; Duel and Race add their lobby under the vehicles), and flying (ui/hud.ts,
// with the pause menu, ui/pause.ts, whose Vehicle tab switches vehicle). Options and Exit sit in
// the menu's top corner. While in the menus the world turns slowly round the chosen place.

import { effect, signal } from "alien-signals";
import * as C from "cesium";
import { sounds } from "../audio/sounds.ts";
import { Flight, type ModeRuntime } from "../flight.ts";
import { Controls } from "../input/actions.ts";
import { radio } from "../input/radio.ts";
import { MODES, modePicks, modePlaces } from "../modes/registry.ts";
import type { ModeContext, ModeDef } from "../modes/types.ts";
import { api } from "../net/api.ts";
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
import { after, frames } from "../streams/time.ts";
import type { World } from "../world/world.ts";
import { h, render, untracked } from "./h.ts";
import { hudSlots, mountHud } from "./hud.ts";
import { icons } from "./icons.ts";
import { menuBindings } from "./keys.ts";
import { openPause } from "./pause.ts";

const svg = (markup: string): HTMLElement => {
  const span = h("span", { class: "icon" });
  span.innerHTML = markup;
  return span;
};

/** One of the menu's columns: a numbered heading over what is picked there. */
const column = (n: number, title: string) => {
  const body = h("div", { class: "col-body" });
  const node = h(
    "section",
    { class: "menu-col" },
    h("h2", null, h("span", { class: "step" }, String(n)), title),
    body,
  );
  return { node, body };
};

export function mountApp(root: HTMLElement, world: World): void {
  let teardown: (() => void) | null = null;
  let flight: Flight | null = null;
  let orbit: AbortController | null = null;
  /** The location clicked in the menu (hovering one only shows it), kept for the next visit. */
  let chosenPlace: string | null = null;
  radio.start();
  // The channels for the UI (the Controls tab's bars, the HUD), in the menus as in flight.
  frames("radio to the UI", "input").subscribe(() => radio.publish());
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
    frames("menu orbit").subscribe(
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

  // Exit stops the server (and its firmware) and closes the window; it asks once first.
  const exitButton = () => {
    const label = h("span", null, "Exit");
    let armed: AbortController | null = null;
    const button = h(
      "button",
      {
        class: "corner-btn",
        title: "Stop the game and close the window",
        onclick: () => {
          if (!armed) {
            sounds.click();
            armed = new AbortController();
            button.classList.add("armed");
            label.textContent = "Click again to exit";
            after(3000).subscribe(
              () => {
                armed = null;
                button.classList.remove("armed");
                label.textContent = "Exit";
              },
              { signal: armed.signal },
            );
            return;
          }
          armed.abort();
          void quitGame();
        },
      },
      svg(icons.exit),
      label,
    );
    return { node: button, dispose: () => armed?.abort() };
  };

  const quitGame = async () => {
    teardown?.();
    teardown = null;
    stopOrbit();
    render(
      root,
      h(
        "div",
        { class: "stopped" },
        h("h1", null, "Drone Duels has stopped"),
        h(
          "p",
          null,
          "You can close this window. Run ",
          h("code", null, "uv run duels"),
          " to fly again.",
        ),
      ),
    );
    try {
      await api.quit();
    } catch (err) {
      console.error("could not stop the server", err);
    }
    window.close();
  };

  const showMenu = () => {
    teardown?.();
    // The menus' backdrop is the free globe; Google's tiles are billed, so only flights use them.
    void world.useTiles(false);
    if (!orbit) startOrbit();
    const scenario = column(1, "Scenario");
    const place = column(2, "Location");
    const ride = column(3, "Vehicle");
    let lobbyTeardown: (() => void) | null = null;
    const closeLobby = () => {
      lobbyTeardown?.();
      lobbyTeardown = null;
    };

    const scenarioButtons = MODES.map((m, i) =>
      h(
        "button",
        { class: "mode-btn", onclick: () => pickMode(m) },
        svg(icons[m.id]),
        h("span", null, h("div", { class: "title" }, m.name), h("div", { class: "desc" }, m.desc)),
        h("span", { class: "kbd" }, String(i + 1)),
      ),
    );
    render(scenario.body, scenarioButtons);

    const pickMode = (m: ModeDef) => {
      sounds.click();
      drawPlaces(m);
    };

    // Column 2: where this scenario can be played. Picking one opens column 3.
    const drawPlaces = (m: ModeDef) => {
      mode(m.id);
      scenarioButtons.forEach((b, i) => b.classList.toggle("active", MODES[i] === m));
      closeLobby();
      const all = modePlaces(m);
      if (!all.some((loc) => loc.id === chosenPlace)) chosenPlace = null;
      const cards = all.map((loc) =>
        locationCard(loc, () => {
          sounds.click();
          chosenPlace = loc.id;
          wind(loc.wind);
          drawPlaces(m);
        }),
      );
      cards.forEach((c, i) => c.classList.toggle("selected", all[i]!.id === chosenPlace));
      render(
        place.body,
        h(
          "div",
          { class: "col-list", onmouseleave: () => chosenPlace && locationId(chosenPlace) },
          cards,
        ),
      );
      place.node.hidden = false;
      const loc = all.find((l) => l.id === chosenPlace);
      if (loc) {
        locationId(loc.id);
        drawVehicles(m, loc);
      } else {
        ride.node.hidden = true;
        render(ride.body);
      }
    };

    // Column 3: what to fly. Free Fly flies on the pick; Duel and Race draw their lobby under it.
    const drawVehicles = (m: ModeDef, loc: Location) => {
      closeLobby();
      const picks = modePicks(m);
      const picked = signal(picks.find((v) => v.id === vehicleId())?.id ?? picks[0]?.id ?? "");
      const cards = picks.map((v) =>
        vehicleCard(v, () => {
          sounds.click();
          if (m.lobby) picked(v.id);
          else fly(loc, v.id, 0, (f) => m.run!(f));
        }),
      );
      const disposeCards = effect(() =>
        cards.forEach((c, i) => c.classList.toggle("selected", picks[i]!.id === picked())),
      );
      const lobby = h("div", { class: "col-lobby" });
      render(
        ride.body,
        h("div", { class: "col-list" }, cards),
        ...(m.pickHint ? [h("p", { class: "hint" }, m.pickHint)] : []),
        lobby,
      );
      ride.node.hidden = false;
      const ctx: ModeContext = {
        location: loc,
        vehicle: picked,
        fly: (vehicle, pad, attach) => fly(loc, vehicle, pad, attach),
        quit: () => {
          flight?.dispose();
          flight = null;
          screen("menu");
        },
      };
      const disposeLobby = m.lobby?.(lobby, ctx);
      lobbyTeardown = () => {
        disposeCards();
        disposeLobby?.();
      };
    };

    const back = () => {
      const m = MODES.find((x) => x.id === mode());
      if (m && chosenPlace) {
        chosenPlace = null;
        drawPlaces(m);
      } else if (m) {
        closeLobby();
        mode(null);
        scenarioButtons.forEach((b) => b.classList.remove("active"));
        place.node.hidden = true;
        ride.node.hidden = true;
        render(place.body);
        render(ride.body);
      }
    };

    const chips = menuChips();
    const exit = exitButton();
    const brand = h("div", { class: "brand" });
    brand.innerHTML = `${icons.logo}<div><h1>DRONE <span>DUELS</span></h1><p>Real firmware. Real places. Fly, duel, race.</p></div>`;
    const options = h(
      "button",
      {
        class: "corner-btn",
        title: "Controls, keys and how to fly, video, wind, graphics, audio (O)",
        onclick: () => openPause(root, null, "controls"),
      },
      svg(icons.options),
      h("span", null, "Options"),
    );
    const toastBox = h("div", { class: "toasts" });
    const disposeToasts = effect(() =>
      toastBox.replaceChildren(
        ...toasts().map((t) => h("div", { class: `toast ${t.tone}` }, t.text)),
      ),
    );
    place.node.hidden = true;
    ride.node.hidden = true;
    const node = h(
      "div",
      { class: "menu" },
      h(
        "header",
        { class: "menu-top" },
        brand,
        h("div", { class: "menu-corner" }, options, exit.node),
      ),
      h("div", { class: "menu-cols" }, scenario.node, place.node, ride.node),
      chips.node,
      toastBox,
    );
    render(root, node);
    // The menu's keys (ui/keys.ts); not while the options are open over it.
    const free = () => !root.querySelector(".pause");
    const keys = new Controls(menuBindings(MODES.map((m) => m.name)))
      .on("mode", (code) => {
        const m = MODES[Number(code.slice(5)) - 1];
        if (m && free()) pickMode(m);
      })
      .on("options", () => free() && openPause(root, null, "controls"))
      .on("back", () => free() && back());
    teardown = () => {
      chips.dispose();
      exit.dispose();
      disposeToasts();
      keys.dispose();
      closeLobby();
    };
    const current = MODES.find((m) => m.id === mode());
    if (current) drawPlaces(current);
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
      const def = MODES.find((m) => m.id === mode());
      const f = new Flight(world, slots, { ownsView: def?.ownsView ?? false });
      flight = f;
      // For the browser tests (scripts/perf.py): the flight now.
      (window as unknown as { duels: { flight?: Flight } }).duels.flight = f;
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
  const fw = {
    "ardupilot-copter": "ArduCopter 4.7",
    betaflight: "Betaflight 2026.6",
    inav: "INAV 9.1",
    none: "no FC",
  }[v.firmware.kind];
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
