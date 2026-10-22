// The sound lab (sounds.html): hear each sound's variants and choose one (sounds.js). A variant
// plays as the game would play it: the last seconds count down, the bump hits soft then hard, a
// drone closes in, the engine revs up through its gears. W or the up arrow drives the engine.

import { PROXIMITY_M, proximityGap, sfx, SOUNDS } from "./sounds.js";

const $ = (id) => document.getElementById(id);

/** The truck's top speed, m/s, and the lab's rough acceleration and braking, m/s². */
const TOP_SPEED_MS = 33;
const ACCEL = 7;
const BRAKE = 9;
const COAST = 1.2;
const ENGINE_HZ = 30;

/** What plays now: its timers, its engine and its card. Something new stops it. */
let demo = { timers: [], engine: null, card: null };

function stopDemo() {
  for (const t of demo.timers) clearTimeout(t);
  demo.engine?.stop();
  demo.card?.classList.remove("playing");
  demo = { timers: [], engine: null, card: null };
}

function later(ms, fn) {
  demo.timers.push(setTimeout(fn, ms));
}

function lightUp(id, ms = 600) {
  const card = cards.get(id);
  card.classList.add("playing");
  later(ms, () => card.classList.remove("playing"));
}

/** How each sound is heard here; the others play once. */
const DEMOS = {
  tick(variant) {
    for (const [i, left] of [5, 4, 3, 2, 1].entries()) {
      later(i * 500, () => sfx.preview("tick", variant, { urgent: left <= 3 }));
    }
  },
  bump(variant) {
    for (const [i, intensity] of [0.2, 0.6, 1].entries()) {
      later(i * 450, () => sfx.preview("bump", variant, { intensity }));
    }
  },
  proximity(variant) {
    // A drone just in range, closing to 4 m in 6 s.
    const range = (t) => PROXIMITY_M - 1 - ((PROXIMITY_M - 5) * t) / 6;
    for (let t = 0, gap; t < 6 && (gap = proximityGap(range(t))) !== null; t += gap) {
      const near = 1 - range(t) / PROXIMITY_M;
      later(t * 1000, () => sfx.preview("proximity", variant, { near }));
    }
  },
  engine(variant) {
    // Idle, flat out through the gears, off the throttle and braking, idle.
    const script = (t) => (t < 0.8 ? 0 : t < 4.6 ? 1 : t < 6 ? -0.6 : 0);
    drive(variant, script, 7);
  },
};

/** Run the engine `variant` for `seconds`, its throttle `script(t)` -1..1 (-1 brakes). */
function drive(variant, script, seconds) {
  const engine = sfx.engine(variant);
  demo.engine = engine;
  let v = 0;
  const start = performance.now();
  const step = () => {
    const t = (performance.now() - start) / 1000;
    if (t > seconds) return stopDemo();
    v = speedAfter(v, script(t), 1 / ENGINE_HZ);
    engine.update(v, script(t));
    showSpeed(v);
    later(1000 / ENGINE_HZ, step);
  };
  step();
}

function speedAfter(v, throttle, dt) {
  const a = throttle > 0 ? ACCEL * throttle * (1 - v / TOP_SPEED_MS) : throttle < 0 ? BRAKE * throttle : -COAST;
  return Math.max(0, v + a * dt);
}

function showSpeed(v) {
  $("engine-speed").textContent = `${Math.round(v * 3.6)} km/h`;
}

function preview(id, variant) {
  stopDemo();
  const card = cards.get(id);
  card.classList.add("playing");
  demo.card = card;
  if (DEMOS[id]) DEMOS[id](variant);
  else {
    sfx.preview(id, variant);
    later(900, stopDemo);
  }
}

// -- W, up: drive the chosen engine yourself ------------------------------------------

const held = new Set();
let selfDrive = null;

function driveYourself() {
  const variant = sfx.choice("engine");
  if (selfDrive || variant === "off") return;
  stopDemo();
  const engine = sfx.engine(variant);
  cards.get("engine").classList.add("playing");
  selfDrive = { engine, v: 0, idleSince: null };
  const step = () => {
    const throttle = held.has("up") ? 1 : held.has("down") ? -1 : 0;
    const d = selfDrive;
    d.v = speedAfter(d.v, throttle, 1 / ENGINE_HZ);
    d.engine.update(d.v, throttle);
    showSpeed(d.v);
    d.idleSince = d.v === 0 && throttle === 0 ? (d.idleSince ?? performance.now()) : null;
    if (d.idleSince !== null && performance.now() - d.idleSince > 1500) {
      d.engine.stop();
      cards.get("engine").classList.remove("playing");
      selfDrive = null;
      return;
    }
    d.timer = setTimeout(step, 1000 / ENGINE_HZ);
  };
  step();
}

const KEYS = { KeyW: "up", ArrowUp: "up", KeyS: "down", ArrowDown: "down" };
addEventListener("keydown", (e) => {
  const key = KEYS[e.code];
  if (!key || e.target instanceof HTMLInputElement) return;
  e.preventDefault();
  held.add(key);
  if (key === "up") driveYourself();
});
addEventListener("keyup", (e) => held.delete(KEYS[e.code]));
addEventListener("blur", () => held.clear());

// -- a turn, in the chosen sounds -------------------------------------------------

/** [ms, sound, opts]: an opponent comes in, a turn, a drone closes and strikes, another
 * crashes, the clock runs out, a win. */
const TURN = [
  [0, "join"],
  [1300, "turnStart"],
  [2800, "proximity", { near: 0.3 }],
  [3700, "proximity", { near: 0.6 }],
  [4250, "proximity", { near: 0.85 }],
  [4600, "strike"],
  [4800, "score"],
  [6100, "crash"],
  [7400, "tick"],
  [7900, "tick"],
  [8400, "tick", { urgent: true }],
  [8900, "tick", { urgent: true }],
  [9400, "tick", { urgent: true }],
  [10100, "turnEnd"],
  [11500, "win"],
];

function playTurn() {
  stopDemo();
  for (const [ms, id, opts] of TURN) {
    later(ms, () => {
      const variant = sfx.choice(id);
      if (variant === "off") return;
      sfx.preview(id, variant, opts);
      lightUp(id);
    });
  }
}

// -- the page ---------------------------------------------------------------------

/** id -> its card. */
const cards = new Map();

function buildCards() {
  const grid = $("cards");
  for (const [id, sound] of Object.entries(SOUNDS)) {
    const card = document.createElement("section");
    card.className = "sound-card";
    const title = document.createElement("h2");
    title.textContent = sound.label;
    const when = document.createElement("p");
    when.textContent = sound.when;
    const chips = document.createElement("div");
    chips.className = "chips";
    for (const [variant, { label }] of [...Object.entries(sound.variants), ["off", { label: "Off" }]]) {
      const chip = document.createElement("button");
      chip.className = `chip-btn${variant === "off" ? " off" : ""}`;
      chip.dataset.variant = variant;
      chip.textContent = label;
      if (variant === sound.default) {
        const tag = document.createElement("small");
        tag.textContent = "default";
        chip.append(tag);
      }
      chip.addEventListener("click", () => {
        sfx.choose(id, variant);
        if (variant === "off") stopDemo();
        else preview(id, variant);
      });
      chips.append(chip);
    }
    card.append(title, when, chips);
    if (id === "engine") {
      const hint = document.createElement("p");
      hint.className = "hint";
      hint.innerHTML = 'Hold <kbd>W</kbd> or <kbd>↑</kbd> to drive it, <kbd>S</kbd> to brake · <span class="meter" id="engine-speed">0 km/h</span>';
      card.append(hint);
    }
    cards.set(id, card);
    grid.append(card);
  }
}

function render() {
  for (const [id, card] of cards) {
    const chosen = sfx.choice(id);
    for (const chip of card.querySelectorAll(".chip-btn")) {
      chip.classList.toggle("chosen", chip.dataset.variant === chosen);
    }
  }
  $("enabled").textContent = sfx.enabled ? "Sound on" : "Sound off";
  $("enabled").classList.toggle("on", sfx.enabled);
  $("off-note").hidden = sfx.enabled;
  const volume = Math.round(sfx.volume * 100);
  if (document.activeElement !== $("volume")) $("volume").value = volume;
  $("volume-out").textContent = `${volume}%`;
}

buildCards();
$("enabled").addEventListener("click", () => sfx.toggle());
$("volume").addEventListener("input", () => sfx.setVolume(Number($("volume").value) / 100));
$("play-turn").addEventListener("click", playTurn);
$("reset").addEventListener("click", () => sfx.resetChoices());
sfx.onChange(render);
render();

// For headless checks and the browser console.
window.soundLab = { sfx, SOUNDS, preview, playTurn, stopDemo };
