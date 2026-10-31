// The pause and options menu (Esc; DESIGN.md §11), in tabs: Vehicle, View, Video, Wind,
// Controls, Inputs, Graphics, Audio and Session. From the main menu (no flight) it holds only the
// options. The sim keeps running while it is open: the firmware and physics are real time.

import { effect } from "alien-signals";
import { type Flight, sendWind } from "../flight.ts";
import { radio } from "../input/radio.ts";
import { config, content, paused, scenario, vehicleId, view, wind } from "../state/app.ts";
import { channels, controllerName, inputSource } from "../state/input.ts";
import { type Prefs, prefs, updatePrefs, type ViewId } from "../state/prefs.ts";
import { log, status } from "../state/session.ts";
import { VIEW_NAMES, VIEW_ORDER } from "../views/rigs.ts";
import { vehicleCard } from "./app.ts";
import { calibrationWizard, forgetCalibration } from "./calibrate.ts";
import { h } from "./h.ts";
import { inputsTab } from "./inputs.ts";

type Tab = { id: string; name: string; body: () => HTMLElement; flight?: boolean };

const opt = (label: string, ...ctl: (Node | string)[]) =>
  h("div", { class: "opt" }, h("label", null, label), h("div", { class: "ctl" }, ...ctl));

function segmented<T extends string>(
  values: [T, string][],
  get: () => T,
  set: (v: T) => void,
): HTMLElement {
  const el = h("div", { class: "seg-ctl" });
  const draw = () =>
    el.replaceChildren(
      ...values.map(([v, label]) =>
        h("button", { class: get() === v ? "on" : "", onclick: () => (set(v), draw()) }, label),
      ),
    );
  draw();
  return el;
}

function slider(
  min: number,
  max: number,
  step: number,
  get: () => number,
  set: (v: number) => void,
  fmt: (v: number) => string,
): Node[] {
  const val = h("span", { class: "val num" }, fmt(get()));
  const input = h("input", {
    type: "range",
    min,
    max,
    step,
    value: String(get()),
  }) as HTMLInputElement;
  input.addEventListener("input", () => {
    set(Number(input.value));
    val.textContent = fmt(Number(input.value));
  });
  return [input, val];
}

function check(get: () => boolean, set: (v: boolean) => void): HTMLInputElement {
  const c = h("input", { type: "checkbox" }) as HTMLInputElement;
  c.checked = get();
  c.addEventListener("change", () => set(c.checked));
  return c;
}

const setP = <K extends keyof Prefs>(k: K, v: Partial<Prefs[K]>) =>
  updatePrefs((p) => ({ ...p, [k]: { ...p[k], ...v } }));

export function openPause(
  root: HTMLElement,
  flight: Flight | null,
  tab = "",
  onQuit?: () => void,
): void {
  if (root.querySelector(".pause")) return;
  paused(true);
  const disposers: (() => void)[] = [];

  const tabs: Tab[] = [
    {
      id: "vehicle",
      name: "Vehicle",
      flight: true,
      body: () => {
        const s = scenario();
        const all = (s?.vehicles ?? []).map((id) => content()!.vehicles[id]!).filter(Boolean);
        return h(
          "div",
          null,
          h("h4", null, "Vehicle"),
          h(
            "p",
            { class: "muted" },
            "Switching starts the new vehicle back at the scenario's start.",
          ),
          h(
            "div",
            { class: "vehicle-list" },
            all.map((v) => {
              const c = vehicleCard(v, () => {
                flight?.switchVehicle(v.id);
                close();
              });
              c.classList.toggle("selected", v.id === vehicleId());
              return c;
            }),
          ),
          h(
            "div",
            { class: "actions" },
            h("button", { onclick: () => (flight?.reset(), close()) }, "Reset to the start"),
          ),
        );
      },
    },
    {
      id: "view",
      name: "View",
      body: () =>
        h(
          "div",
          null,
          h("h4", null, "View"),
          opt(
            "View now",
            segmented(
              VIEW_ORDER.map((v) => [v, VIEW_NAMES[v]] as [ViewId, string]),
              () => view(),
              (v) => view(v),
            ),
          ),
          opt(
            "First view",
            segmented(
              VIEW_ORDER.map((v) => [v, VIEW_NAMES[v]] as [ViewId, string]),
              () => prefs().view.initial,
              (v) => setP("view", { initial: v }),
            ),
          ),
          opt(
            "Chase distance",
            ...slider(
              2,
              12,
              0.5,
              () => prefs().view.chaseDistance,
              (v) => setP("view", { chaseDistance: v }),
              (v) => `${v} m`,
            ),
          ),
          opt(
            "Field of view",
            ...slider(
              40,
              100,
              1,
              () => prefs().view.fovDeg,
              (v) => setP("view", { fovDeg: v }),
              (v) => `${v}°`,
            ),
          ),
          opt(
            "Minimap",
            check(
              () => prefs().hud.minimap,
              (v) => setP("hud", { minimap: v }),
            ),
          ),
          opt(
            "Dials (not in FPV)",
            check(
              () => prefs().hud.dials,
              (v) => setP("hud", { dials: v }),
            ),
          ),
        ),
    },
    {
      id: "video",
      name: "Video",
      body: () =>
        h(
          "div",
          null,
          h("h4", null, "Video"),
          h(
            "p",
            { class: "muted" },
            "Analog: NTSC over a 5.8 GHz FM link, the OSD chip's 30×16 grid. Digital: a clean 16:9 picture with an HD OSD. Changing it restarts the vehicle at the start: the firmware sets its OSD grid as it boots.",
          ),
          opt(
            "Video system",
            segmented(
              [
                ["analog", "Analog"],
                ["digital", "Digital"],
              ],
              () => prefs().video.system,
              (v) => setP("video", { system: v }),
            ),
          ),
          opt(
            "VTX power",
            ...slider(
              25,
              800,
              25,
              () => prefs().video.vtxMw,
              (v) => setP("video", { vtxMw: v }),
              (v) => `${v} mW`,
            ),
          ),
          opt(
            "OSD",
            check(
              () => prefs().video.osd,
              (v) => setP("video", { osd: v }),
            ),
          ),
          opt(
            "Camera tilt",
            ...slider(
              -1,
              60,
              1,
              () => prefs().video.tiltDeg ?? -1,
              (v) => setP("video", { tiltDeg: v < 0 ? null : v }),
              (v) => (v < 0 ? "airframe's" : `${v}°`),
            ),
          ),
        ),
    },
    {
      id: "wind",
      name: "Wind",
      body: () => {
        const w = () =>
          wind() ?? { speed_ms: 0, from_deg: 0, turbulence: "none" as const, gusts: false };
        const set = (c: Partial<ReturnType<typeof w>>) => {
          wind({ ...w(), ...c });
          sendWind();
        };
        return h(
          "div",
          null,
          h("h4", null, "Wind"),
          h(
            "p",
            { class: "muted" },
            "MIL-F-8785C: the wind at 6 m, less near the ground, Dryden turbulence, 1−cos gusts. Changes reach the physics at once.",
          ),
          opt(
            "Speed",
            ...slider(
              0,
              20,
              0.5,
              () => w().speed_ms,
              (v) => set({ speed_ms: v }),
              (v) => `${v} m/s`,
            ),
          ),
          opt(
            "From",
            ...slider(
              0,
              355,
              5,
              () => w().from_deg,
              (v) => set({ from_deg: v }),
              (v) => `${v}°`,
            ),
          ),
          opt(
            "Turbulence",
            segmented(
              [
                ["none", "None"],
                ["light", "Light"],
                ["moderate", "Moderate"],
                ["severe", "Severe"],
              ],
              () => w().turbulence,
              (v) => set({ turbulence: v }),
            ),
          ),
          opt(
            "Gusts",
            check(
              () => w().gusts,
              (v) => set({ gusts: v }),
            ),
          ),
          opt("", h("button", { onclick: () => sendWind(true) }, "Gust now (G)")),
        );
      },
    },
    {
      id: "controls",
      name: "Controls",
      body: () => {
        const bars = h("div", { class: "ch-bars" });
        const who = h("span", { class: "num" });
        const wizardHost = h("div");
        disposers.push(
          effect(() => {
            const ch = channels();
            who.textContent = `${controllerName() ?? "none"}${inputSource() === "server" ? " (read by the server, through Linux)" : ""}`;
            bars.replaceChildren(
              ...ch
                .slice(0, 8)
                .map((v, i) =>
                  h(
                    "div",
                    { class: "ch-bar" },
                    h(
                      "div",
                      { class: "track" },
                      h("div", { class: "fill", style: `height:${((v + 1) / 2) * 100}%` }),
                    ),
                    `CH${i + 1}`,
                  ),
                ),
            );
          }),
        );
        const id = () => radio.raw?.id ?? "";
        return h(
          "div",
          null,
          h("h4", null, "Controls"),
          opt("Controller", who),
          bars,
          h(
            "p",
            { class: "muted" },
            "Channels: 1 roll, 2 pitch, 3 throttle, 4 yaw, 5 arm, 6 flight mode, 8 launch (the wing). Profiles for the LiteRadio 2, TX15 and T8L are built in; calibrate any radio once and it is kept.",
          ),
          h(
            "div",
            { class: "actions" },
            h(
              "button",
              {
                class: "primary",
                onclick: () =>
                  wizardHost.replaceChildren(calibrationWizard(() => wizardHost.replaceChildren())),
              },
              "Calibrate",
            ),
            h("button", { onclick: () => forgetCalibration(id()) }, "Use the profile"),
          ),
          wizardHost,
          opt(
            "Keyboard flying",
            check(
              () => radio.keyboard,
              (v) => (radio.keyboard = v),
            ),
            h("span", { class: "muted" }, "W/S throttle, arrows, Q/E yaw, Enter arm, M mode"),
          ),
        );
      },
    },
    { id: "inputs", name: "Inputs", body: inputsTab },
    {
      id: "graphics",
      name: "Graphics",
      body: () =>
        h(
          "div",
          null,
          h("h4", null, "Graphics"),
          opt(
            "Map",
            h(
              "span",
              null,
              config()?.map_keys.google_maps_api_key
                ? "Google Photorealistic 3D Tiles"
                : config()?.map_keys.cesium_ion_token
                  ? "Cesium ion"
                  : "Esri imagery (set GOOGLE_MAPS_API_KEY in .env.local for 3D tiles)",
            ),
          ),
          opt(
            "Detail",
            segmented(
              [
                ["low", "Low"],
                ["medium", "Medium"],
                ["high", "High"],
              ],
              () => prefs().graphics.quality,
              (v) => setP("graphics", { quality: v }),
            ),
          ),
          opt(
            "Units",
            segmented(
              [
                ["metric", "Metric"],
                ["imperial", "Imperial"],
              ],
              () => prefs().hud.units,
              (v) => setP("hud", { units: v }),
            ),
          ),
        ),
    },
    {
      id: "audio",
      name: "Audio",
      body: () =>
        h(
          "div",
          null,
          h("h4", null, "Audio"),
          opt(
            "Sound",
            check(
              () => prefs().audio.on,
              (v) => setP("audio", { on: v }),
            ),
          ),
          opt(
            "Volume",
            ...slider(
              0,
              1,
              0.05,
              () => prefs().audio.master,
              (v) => setP("audio", { master: v }),
              (v) => `${Math.round(v * 100)}%`,
            ),
          ),
          opt(
            "Motors",
            ...slider(
              0,
              1,
              0.05,
              () => prefs().audio.motors,
              (v) => setP("audio", { motors: v }),
              (v) => `${Math.round(v * 100)}%`,
            ),
          ),
          opt(
            "Effects",
            ...slider(
              0,
              1,
              0.05,
              () => prefs().audio.effects,
              (v) => setP("audio", { effects: v }),
              (v) => `${Math.round(v * 100)}%`,
            ),
          ),
        ),
    },
    {
      id: "session",
      name: "Session",
      flight: true,
      body: () =>
        h(
          "div",
          null,
          h("h4", null, "Session"),
          opt(
            "Vehicle",
            h(
              "span",
              { class: "num" },
              `${status().vehicle ?? "-"} · ${status().state}${status().step ? ` · ${status().step}` : ""}`,
            ),
          ),
          opt(
            "QGroundControl",
            h(
              "span",
              { class: "num" },
              `UDP ${config()?.qgc_port ?? 14550}: plan and upload missions; the Map view draws them`,
            ),
          ),
          opt(
            "Player name",
            (() => {
              const input = h("input", { value: prefs().player.name }) as HTMLInputElement;
              input.addEventListener("change", () =>
                setP("player", { name: input.value.trim() || "Pilot" }),
              );
              return input;
            })(),
          ),
          h("h4", { style: "margin-top:18px" }, "Log"),
          h(
            "div",
            {
              class: "num",
              style: "font-size:12px; max-height: 220px; overflow:auto; color: var(--muted)",
            },
            log()
              .slice(-30)
              .map((l) =>
                h(
                  "div",
                  { style: l.level === "error" ? "color: var(--rose)" : "" },
                  `${l.source}: ${l.text}`,
                ),
              ),
          ),
        ),
    },
  ];

  const shown = tabs.filter((t) => flight || !t.flight);
  let current = shown.find((t) => t.id === tab) ?? shown[0]!;
  const body = h("div", { class: "tab-body" });
  const buttons = shown.map((t) => h("button", { onclick: () => select(t) }, t.name));
  const select = (t: Tab) => {
    current = t;
    for (const d of disposers.splice(0)) d();
    buttons.forEach((b, i) => b.classList.toggle("active", shown[i] === t));
    body.replaceChildren(t.body());
  };
  const close = () => {
    for (const d of disposers.splice(0)) d();
    overlay.remove();
    window.removeEventListener("keydown", onKey, { capture: true });
    paused(false);
  };
  const onKey = (e: KeyboardEvent) => {
    if (e.key === "Escape") {
      e.stopImmediatePropagation();
      close();
    }
  };
  const bottom = h(
    "div",
    { class: "bottom" },
    h("button", { class: "primary", onclick: close }, flight ? "Resume" : "Done"),
    ...(flight && onQuit
      ? [h("button", { onclick: () => (close(), onQuit()) }, "Quit to menu")]
      : []),
  );
  const overlay = h(
    "div",
    { class: "pause", onclick: (e: Event) => e.target === overlay && close() },
    h(
      "div",
      { class: "pause-box panel" },
      h("div", { class: "tabs" }, h("h3", null, flight ? "Paused" : "Options"), buttons, bottom),
      body,
    ),
  );
  root.append(overlay);
  window.addEventListener("keydown", onKey, { capture: true });
  select(current);
}
