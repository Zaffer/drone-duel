// The flight's HUD (DESIGN.md §11): a thin top bar (fading in FPV), the dials bottom left and the
// minimap bottom right (the dials hide in FPV, where the OSD is the instrument panel), toasts,
// what the session is doing while it starts, the mode's banner and panel, and the keys:
//   Esc pause and options   V / 1-6 views   R reset   B vehicle   T throw   D dynamics
//   N minimap   H keys help   G a gust now   P performance
// On a touch screen the sticks are on the glass (ui/sticks.ts), the top bar's buttons stand in
// for the keys (Menu, View, Reset), and the corners' panels move up out of the thumbs' way
// (ui.css).

import { computed, effect, signal } from "alien-signals";
import { type Flight, sendWind } from "../flight.ts";
import { mountDials } from "../hud/dials.ts";
import { mountPerf } from "../hud/perf.ts";
import { Controls } from "../input/actions.ts";
import { touch } from "../input/touch.ts";
import { link } from "../net/server.ts";
import {
  dynamicsOn,
  location,
  mode,
  paused,
  perfOn,
  toast,
  toasts,
  vehicle,
  view,
  wind,
} from "../state/app.ts";
import { clearHud, hudBanner, hudPanel, hudTimer } from "../state/hud.ts";
import { inputSource, touchScreen } from "../state/input.ts";
import { prefs, updatePrefs } from "../state/prefs.ts";
import { phase, status, telemetry } from "../state/session.ts";
import { every } from "../streams/time.ts";
import { VIEW_NAMES, VIEW_ORDER } from "../views/rigs.ts";
import { h } from "./h.ts";
import { flightKeys, shooterKeys } from "./inputs.ts";
import { flightBindings, footBindings } from "./keys.ts";
import { openPause } from "./pause.ts";
import { mountSticks } from "./sticks.ts";

const MODE_NAMES = {
  freefly: "Free Fly",
  duel: "Duel",
  race: "Race",
  shoot: "Shoot Down",
} as const;
const COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];
/** A crash resets itself after this long (R or a tap sooner). */
const AUTO_RESET_S = 3;

export interface HudSlots {
  feed: HTMLElement;
  minimap: HTMLElement;
  dynamics: HTMLElement;
}

export function hudSlots(): HudSlots {
  return {
    feed: h("div", { class: "feed-layer" }),
    minimap: h("div", { class: "corner right" }),
    dynamics: h("div", { class: "dyn-layer" }),
  };
}

export function mountHud(
  root: HTMLElement,
  flight: Flight,
  slots: HudSlots,
  onQuit: () => void,
): () => void {
  const seg = (name: string, label: string, onclick?: () => void) => {
    const value = h("b");
    const el = onclick
      ? h("button", { class: `seg seg-${name}`, onclick }, label, value)
      : h("span", { class: `seg seg-${name}` }, label, value);
    return { el, value };
  };
  const pause = (tab?: string) => openPause(root, flight, tab, onQuit);

  const modeSeg = seg("mode", "");
  const vehicleSeg = seg("vehicle", "", () => pause("vehicle"));
  const viewSeg = seg("view", "View ", () => cycleView(1));
  const videoSeg = seg("video", "Video ", () =>
    updatePrefs((p) => ({
      ...p,
      video: { ...p.video, system: p.video.system === "analog" ? "digital" : "analog" },
    })),
  );
  const windSeg = seg("wind", "Wind ", () => pause("wind"));
  const timer = h("span", { class: "seg timer" });
  const reset = () => vehicle()?.kind !== "ground" && flight.reset();
  /** Reset by a tap: the sticks on the glass are let go first (disarmed, the throttle down). */
  const tapReset = () => {
    if (inputSource() === "touch") touch.rest();
    flight.reset();
  };
  // The R key's button, where there are no keys (ui.css shows it on a touch screen).
  const resetBtn = h(
    "button",
    { class: "seg seg-reset", onclick: tapReset },
    h("b", null, "Reset"),
  );
  const menuBtn = h(
    "button",
    { class: "seg seg-menu", onclick: () => pause() },
    h("span", { class: "key" }, "Esc "),
    h("b", null, "Menu"),
  );
  const topbar = h(
    "div",
    { class: "topbar" },
    modeSeg.el,
    h("span", { class: "sep" }),
    vehicleSeg.el,
    viewSeg.el,
    videoSeg.el,
    windSeg.el,
    resetBtn,
    timer,
    h("span", { class: "sep" }),
    menuBtn,
  );
  const dials = h("div", { class: "corner left" });
  const disposeDials = mountDials(dials);
  const toastBox = h("div", { class: "toasts" });
  // Crashed, it counts down to a reset of its own (paused, it waits; a Duel's destroyed aircraft
  // is the mode's to bring back). Seconds left, or null.
  const crashedHere = computed(
    () => phase() === "crashed" && !onFoot && status().state !== "destroyed" && !paused(),
  );
  const resetIn = signal<number | null>(null);
  let countdown: AbortController | null = null;
  // Crashed or destroyed, it says how to reset, and a tap on it does.
  const starting = h("div", {
    class: "starting",
    onclick: () => {
      const s = status().state;
      if (s === "destroyed" || (phase() === "crashed" && !onFoot)) tapReset();
    },
  });
  const banner = h("div", { class: "banner" });
  const panelHost = h("div");
  const perfBox = h("div");
  const perfPanel = mountPerf(perfBox);
  // On the ground with a rifle (Shoot Down), the keys are the shooter's (shoot/shooter.ts).
  const onFoot = flight.ownsView;
  const keysHelp = h(
    "div",
    { class: "keys-help" },
    ...(onFoot ? shooterKeys() : flightKeys())
      .filter((k) => k.strip)
      .map((k) => h("span", null, h("span", { class: "kbd" }, k.keys[0]!), " ", k.strip!)),
  );
  if (onFoot) for (const seg of [vehicleSeg.el, viewSeg.el, videoSeg.el]) seg.hidden = true;
  // Under the panels: a panel over a stick's side of the screen keeps its own taps.
  const sticksHost = h("div", { class: "sticks-layer" });
  const disposeSticks = onFoot ? () => {} : mountSticks(sticksHost);
  const hud = h(
    "div",
    { class: "hud" },
    slots.feed,
    slots.dynamics,
    sticksHost,
    topbar,
    dials,
    slots.minimap,
    toastBox,
    starting,
    banner,
    panelHost,
    perfBox,
    keysHelp,
  );
  root.replaceChildren(hud);

  const cycleView = (step: number) => {
    const i = VIEW_ORDER.indexOf(view());
    view(VIEW_ORDER[(i + step + VIEW_ORDER.length) % VIEW_ORDER.length]!);
  };

  // The top bar and the keys fade in FPV after a while still; the mouse brings them back.
  let lastActivity = performance.now();
  const wake = () => (lastActivity = performance.now());
  window.addEventListener("mousemove", wake);
  const fadeTimer = setInterval(() => {
    // On a touch screen the top bar is the only way to the menu: it stays.
    const idle = performance.now() - lastActivity > 3000 && !touchScreen();
    topbar.classList.toggle("faded", idle && view() === "fpv" && !paused());
    keysHelp.classList.toggle("faded", performance.now() - lastActivity > 6000);
  }, 250);

  // The keys (ui/keys.ts): on foot, the few that still mean something; Esc resumes from paused.
  const toggleHelp = () => keysHelp.classList.toggle("faded");
  const controls = onFoot
    ? new Controls(footBindings())
        .on("menu", () => pause())
        .on("help", toggleHelp)
        .on("gust", () => sendWind(true))
        .on("perf", () => perfOn(!perfOn()))
    : new Controls(flightBindings(VIEW_ORDER.map((v) => VIEW_NAMES[v])))
        .on("menu", () => pause())
        .on("nextView", () => cycleView(1))
        .on("lastView", () => cycleView(-1))
        .on("view", (code) => view(VIEW_ORDER[Number(code.slice(5)) - 1]!))
        .on("reset", reset)
        .on("vehicle", () => pause("vehicle"))
        .on("throw", () => flight.throw())
        .on("dynamics", () => vehicle()?.kind !== "ground" && dynamicsOn(!dynamicsOn()))
        .on("minimap", () =>
          updatePrefs((p) => ({ ...p, hud: { ...p.hud, minimap: !p.hud.minimap } })),
        )
        .on("help", toggleHelp)
        .on("gust", () => sendWind(true))
        .on("perf", () => perfOn(!perfOn()));
  window.addEventListener("keydown", wake);

  const disposers = [
    effect(() => {
      countdown?.abort();
      countdown = null;
      if (!crashedHere()) {
        resetIn(null);
        return;
      }
      const stop = new AbortController();
      countdown = stop;
      const due = performance.now() + AUTO_RESET_S * 1000;
      resetIn(AUTO_RESET_S);
      every(100).subscribe(
        (now) => {
          const left = Math.ceil((due - now) / 1000);
          if (left > 0) resetIn(left);
          else {
            stop.abort();
            tapReset();
          }
        },
        { signal: stop.signal },
      );
    }),
    effect(() => {
      const m = mode();
      modeSeg.el.replaceChildren(
        h("b", null, m ? MODE_NAMES[m] : ""),
        h("span", null, ` · ${location()?.name.split(": ").at(-1) ?? ""}`),
      );
    }),
    effect(() => {
      vehicleSeg.value.textContent = vehicle()?.name ?? "";
    }),
    effect(() => {
      viewSeg.value.textContent = VIEW_NAMES[view()];
      const v = view();
      dials.hidden =
        onFoot || v === "fpv" || !prefs().hud.dials || vehicle()?.firmware.kind === "none";
      slots.minimap.hidden = onFoot || v === "map" || !prefs().hud.minimap;
    }),
    effect(() => {
      videoSeg.value.textContent = prefs().video.system === "analog" ? "Analog" : "Digital";
    }),
    effect(() => {
      const w = wind();
      windSeg.value.textContent = w
        ? `${w.speed_ms.toFixed(0)} m/s ${COMPASS[Math.round(w.from_deg / 45) % 8]}${w.gust_factor > 1 ? " gusty" : ""}`
        : "calm";
    }),
    effect(() => {
      const t = hudTimer();
      timer.hidden = !t;
      timer.textContent = t ?? "";
    }),
    effect(() => {
      const b = hudBanner();
      banner.replaceChildren(
        ...(b
          ? [
              h("div", { class: "big", style: b.tone ? `color: var(--${b.tone})` : "" }, b.big),
              ...(b.small ? [h("div", { class: "small" }, b.small)] : []),
            ]
          : []),
      );
    }),
    effect(() => {
      const p = hudPanel();
      panelHost.replaceChildren(...(p ? [p] : []));
    }),
    effect(() => {
      toastBox.replaceChildren(
        ...toasts().map((t) => h("div", { class: `toast ${t.tone}` }, t.text)),
      );
    }),
    effect(() => {
      const s = status();
      // Shooting a drone down, its crash is the point: the mode says so itself.
      const crashed = phase() === "crashed" && !onFoot;
      const show =
        s.state === "starting" || s.state === "failed" || s.state === "destroyed" || crashed;
      starting.hidden = !show;
      starting.classList.toggle("failed", s.state === "failed");
      if (s.state === "starting")
        starting.replaceChildren(
          h("span", { class: "spinner" }),
          `${vehicle()?.name ?? ""}: ${s.step}…`,
        );
      else if (s.state === "failed") starting.replaceChildren(`Could not start: ${s.step}`);
      else if (s.state === "destroyed" || crashed) {
        const left = resetIn();
        const now = touchScreen() ? ["Tap"] : [h("span", { class: "kbd" }, "R")];
        starting.replaceChildren(
          crashed ? "Crashed. " : "Destroyed. ",
          ...(left !== null ? [`Resetting in ${left}… `, ...now, " now"] : [...now, " to reset"]),
        );
      }
      starting.classList.toggle("tap", s.state === "destroyed" || crashed);
    }),
    effect(() => {
      // Say once how to arm, when the vehicle is first ready.
      if (status().state !== "ready" || onFoot) return;
      const v = vehicle();
      const t = telemetry();
      if (!v || !t || !v.aux || t.armed) return;
      hint(v.aux.arm, v.aux.modes, v.launch === "hand");
    }),
  ];
  let hinted = "";
  const hint = (arm: number, modes: string[], hand: boolean) => {
    const key = `${vehicle()?.id}`;
    if (hinted === key) return;
    hinted = key;
    const kb = inputSource() === "keyboard";
    const glass = inputSource() === "touch";
    const armHow = glass
      ? "Tap Arm with the left stick down, then raise it. Left stick: throttle and yaw; right: roll and pitch"
      : kb
        ? "Enter arms, W/S throttle, arrows and Q/E steer"
        : `CH${arm} high arms`;
    const modeHow = modes.length
      ? glass
        ? `The mode's button steps ${modes.join("/")}`
        : kb
          ? `M steps ${modes.join("/")}`
          : `mode switch: ${modes.join(" / ")}`
      : "";
    const throwHow = hand ? "; raise the throttle, then T (or the launch switch) throws" : "";
    toast(`${armHow}. ${modeHow}${throwHow}`, "info", glass ? 12000 : 9000);
  };

  return () => {
    disposeSticks();
    disposeDials();
    perfPanel.dispose();
    for (const d of disposers) d();
    countdown?.abort();
    clearInterval(fadeTimer);
    controls.dispose();
    window.removeEventListener("keydown", wake);
    window.removeEventListener("mousemove", wake);
    clearHud();
    link.send({ ch: "dynamics", on: false });
  };
}
