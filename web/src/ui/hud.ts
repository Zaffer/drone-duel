// The flight's HUD (DESIGN.md §11): a thin top bar (fading in FPV), the dials bottom left and the
// minimap bottom right (the dials hide in FPV, where the OSD is the instrument panel), toasts,
// what the session is doing while it starts, the mode's banner and panel, and the keys:
//   Esc pause and options   V / 1-6 views   R reset   B vehicle   T throw   D dynamics
//   N minimap   H keys help   G a gust now   P performance

import { effect } from "alien-signals";
import { type Flight, sendWind } from "../flight.ts";
import { mountDials } from "../hud/dials.ts";
import { mountPerf } from "../hud/perf.ts";
import { link } from "../net/server.ts";
import {
  dynamicsOn,
  location,
  mode,
  paused,
  toast,
  toasts,
  vehicle,
  view,
  wind,
} from "../state/app.ts";
import { clearHud, hudBanner, hudPanel, hudTimer } from "../state/hud.ts";
import { inputSource } from "../state/input.ts";
import { prefs, updatePrefs } from "../state/prefs.ts";
import { phase, status, telemetry } from "../state/session.ts";
import { VIEW_NAMES, VIEW_ORDER } from "../views/rigs.ts";
import { h } from "./h.ts";
import { flightKeys } from "./inputs.ts";
import { openPause } from "./pause.ts";

const MODE_NAMES = { freefly: "Free Fly", duel: "Duel", race: "Race" } as const;
const COMPASS = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"];

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
  const seg = (label: string, onclick?: () => void) => {
    const value = h("b");
    const el = onclick
      ? h("button", { class: "seg", onclick }, label, value)
      : h("span", { class: "seg" }, label, value);
    return { el, value };
  };
  const pause = (tab?: string) => openPause(root, flight, tab, onQuit);

  const modeSeg = seg("");
  const vehicleSeg = seg("", () => pause("vehicle"));
  const viewSeg = seg("View ", () => cycleView(1));
  const videoSeg = seg("Video ", () =>
    updatePrefs((p) => ({
      ...p,
      video: { ...p.video, system: p.video.system === "analog" ? "digital" : "analog" },
    })),
  );
  const windSeg = seg("Wind ", () => pause("wind"));
  const timer = h("span", { class: "seg timer" });
  const menuBtn = h(
    "button",
    { class: "seg", onclick: () => pause() },
    "Esc ",
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
    timer,
    h("span", { class: "sep" }),
    menuBtn,
  );
  const dials = h("div", { class: "corner left" });
  const disposeDials = mountDials(dials);
  const toastBox = h("div", { class: "toasts" });
  const starting = h("div", { class: "starting" });
  const banner = h("div", { class: "banner" });
  const panelHost = h("div");
  const perfBox = h("div");
  const perfPanel = mountPerf(perfBox);
  const keysHelp = h(
    "div",
    { class: "keys-help" },
    ...flightKeys()
      .filter((k) => k.strip)
      .map((k) => h("span", null, h("span", { class: "kbd" }, k.keys[0]!), " ", k.strip!)),
  );
  const hud = h(
    "div",
    { class: "hud" },
    slots.feed,
    slots.dynamics,
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
    const idle = performance.now() - lastActivity > 3000;
    topbar.classList.toggle("faded", idle && view() === "fpv" && !paused());
    keysHelp.classList.toggle("faded", performance.now() - lastActivity > 6000);
  }, 250);

  const keys = (e: KeyboardEvent) => {
    if (e.repeat || paused() || (e.target as HTMLElement).tagName === "INPUT") {
      if (e.key === "Escape" && paused()) paused(false);
      return;
    }
    wake();
    const n = Number(e.key);
    if (e.key === "Escape") pause();
    else if (e.code === "KeyV") cycleView(e.shiftKey ? -1 : 1);
    else if (n >= 1 && n <= VIEW_ORDER.length && !e.shiftKey) view(VIEW_ORDER[n - 1]!);
    else if (e.code === "KeyR" && vehicle()?.kind !== "ground") flight.reset();
    else if (e.code === "KeyB") pause("vehicle");
    else if (e.code === "KeyT") flight.throw();
    else if (e.code === "KeyD" && vehicle()?.kind !== "ground") dynamicsOn(!dynamicsOn());
    else if (e.code === "KeyN")
      updatePrefs((p) => ({ ...p, hud: { ...p.hud, minimap: !p.hud.minimap } }));
    else if (e.code === "KeyH") keysHelp.classList.toggle("faded");
    else if (e.code === "KeyG") sendWind(true);
    else if (e.code === "KeyP") perfPanel.toggle();
  };
  window.addEventListener("keydown", keys);

  const disposers = [
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
      dials.hidden = v === "fpv" || !prefs().hud.dials || vehicle()?.firmware.kind === "none";
      slots.minimap.hidden = v === "map" || !prefs().hud.minimap;
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
      const crashed = phase() === "crashed";
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
      else if (s.state === "destroyed" || crashed)
        starting.replaceChildren(
          crashed ? "Crashed. " : "Destroyed. ",
          h("span", { class: "kbd" }, "R"),
          " to reset",
        );
    }),
    effect(() => {
      // Say once how to arm, when the vehicle is first ready.
      if (status().state !== "ready") return;
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
    const armHow = kb ? "Enter arms, W/S throttle, arrows and Q/E steer" : `CH${arm} high arms`;
    const modeHow = modes.length
      ? kb
        ? `M steps ${modes.join("/")}`
        : `mode switch: ${modes.join(" / ")}`
      : "";
    const throwHow = hand ? "; raise the throttle, then T (or the launch switch) throws" : "";
    toast(`${armHow}. ${modeHow}${throwHow}`, "info", 9000);
  };

  return () => {
    disposeDials();
    perfPanel.dispose();
    for (const d of disposers) d();
    clearInterval(fadeTimer);
    window.removeEventListener("keydown", keys);
    window.removeEventListener("mousemove", wake);
    clearHud();
    link.send({ ch: "dynamics", on: false });
  };
}
