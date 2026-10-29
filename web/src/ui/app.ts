// The game's screens (DESIGN.md §11): the main menu (mode), the scenario and vehicle pickers, a
// mode's lobby, and flying (ui/hud.ts, with the pause menu, ui/pause.ts). While in the menus the
// world turns slowly round the chosen scenario's place.

import { effect } from "alien-signals";
import * as C from "cesium";
import { sounds } from "../audio/sounds.ts";
import { Flight, type ModeRuntime } from "../flight.ts";
import { radio } from "../input/radio.ts";
import { MODES } from "../modes/registry.ts";
import type { ModeContext, ModeDef } from "../modes/types.ts";
import type { Scenario, Vehicle } from "../net/protocol.ts";
import { link } from "../net/server.ts";
import {
  content,
  location,
  mode,
  paused,
  scenario,
  scenarioId,
  screen,
  toast,
  toasts,
  vehicleId,
  wind,
} from "../state/app.ts";
import { controllerName, inputSource } from "../state/input.ts";
import { frames } from "../streams/time.ts";
import type { World } from "../world/world.ts";
import { h, render } from "./h.ts";
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
  window.addEventListener("pointerdown", () => sounds.unlock(), { once: true });
  window.addEventListener("keydown", () => sounds.unlock(), { once: true });

  // The menus' backdrop: the world turning round the scenario's place.
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
      map.lastChild!.textContent =
        world.fallbackReason ??
        { google: "Google 3D Tiles", ion: "Cesium ion", esri: "Esri imagery" }[world.source];
    });
    return { node: h("div", { class: "status-row" }, server, ctl, map), dispose };
  };

  const showMenu = () => {
    teardown?.();
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
        h("div", { class: "desc" }, "Controls, video, wind, graphics, audio."),
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
    const all = Object.values(content()?.scenarios ?? {}).filter((s) => s.mode === m.id);
    const cards = all.map((s) => scenarioCard(s, () => pickScenario(m, s, main)));
    render(
      main,
      h(
        "div",
        { class: "picker" },
        h("h2", null, `${m.name}: choose a scenario`),
        h("div", { class: "cards" }, cards),
      ),
    );
    const current = all.find((s) => s.id === scenarioId()) ?? all[0];
    if (current) {
      scenarioId(current.id);
      cards.forEach((c, i) => c.classList.toggle("selected", all[i] === current));
    }
  };

  const scenarioCard = (s: Scenario, onPick: () => void) => {
    const loc = content()?.locations[s.location];
    const card = h(
      "button",
      {
        class: "card",
        onclick: onPick,
        onmouseenter: () => scenarioId(s.id),
      },
      h("div", { class: "name" }, s.name),
      h("div", { class: "blurb" }, s.blurb),
      h(
        "div",
        { class: "tags" },
        h("span", { class: "tag" }, loc?.name.split(":")[0] ?? s.location),
        ...(s.course
          ? [
              h(
                "span",
                { class: "tag" },
                `${content()?.courses[s.course]?.gates.length ?? 0} gates`,
              ),
            ]
          : []),
      ),
    );
    card.prepend(h("span", { class: "glyph" }, svg(icons.pin)));
    return card;
  };

  const pickScenario = (m: ModeDef, s: Scenario, main: HTMLElement) => {
    sounds.click();
    scenarioId(s.id);
    wind(content()?.locations[s.location]?.wind ?? null);
    const all = s.vehicles.map((id) => content()?.vehicles[id]).filter((v): v is Vehicle => !!v);
    const vehicles = m.vehicles(all);
    const ctx: ModeContext = {
      scenario: s,
      vehicles,
      fly: (vehicle, pad, attach) => fly(s, vehicle, pad, attach),
      quit: () => {
        flight?.dispose();
        flight = null;
        screen("menu");
      },
    };
    const back = h(
      "button",
      {
        onclick: () => pickMode(m, main, [...document.querySelectorAll<HTMLElement>(".mode-btn")]),
      },
      "Back",
    );
    if (!m.pickVehicle) {
      lobbyTeardown?.();
      const host = h("div", { class: "picker" });
      render(main, host);
      lobbyTeardown = m.lobby?.(host, ctx, null) ?? null;
      return;
    }
    let chosen = vehicles.find((v) => v.id === vehicleId())?.id ?? s.default_vehicle;
    if (!vehicles.some((v) => v.id === chosen)) chosen = vehicles[0]?.id ?? chosen;
    const cards = vehicles.map((v) =>
      vehicleCard(v, () => {
        chosen = v.id;
        cards.forEach((c, i) => c.classList.toggle("selected", vehicles[i]!.id === chosen));
      }),
    );
    cards.forEach((c, i) => c.classList.toggle("selected", vehicles[i]!.id === chosen));
    const go = h(
      "button",
      {
        class: "primary",
        onclick: () => {
          sounds.click();
          if (m.lobby) {
            lobbyTeardown?.();
            const host = h("div", { class: "picker" });
            render(main, host);
            lobbyTeardown = m.lobby(host, ctx, chosen);
          } else {
            fly(s, chosen, 0, (f) => m.run!(f));
          }
        },
      },
      m.lobby ? "Next" : "Fly",
    );
    render(
      main,
      h(
        "div",
        { class: "picker" },
        h("h2", null, `${s.name}: choose your vehicle`),
        h("div", { class: "cards" }, cards),
        h("div", { class: "actions" }, back, h("span", { class: "spacer" }), go),
      ),
    );
  };

  const fly = (s: Scenario, vehicle: string, pad: number, attach: (f: Flight) => ModeRuntime) => {
    scenarioId(s.id);
    flight?.dispose();
    teardown?.();
    teardown = null;
    stopOrbit();
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
      f.mode = attach(f);
      f.start(s.id, vehicle, pad);
    } catch (err) {
      console.error("could not start the flight", err);
      flight?.dispose();
      flight = null;
      screen("menu");
      toast(`Could not start: ${err instanceof Error ? err.message : String(err)}`, "bad", 8000);
    }
  };

  effect(() => {
    const s = screen();
    if (s === "menu") showMenu();
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

export { scenario };
