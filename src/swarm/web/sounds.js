// The game's sounds, made in the browser with Web Audio: no files to fetch, and each sound has
// a few variants to choose from on the sound lab (sounds.html). The choices, the volume and
// whether sound is on are kept in localStorage, shared by the viewer's pages (the COP, the
// drive, the lab): a change on one reaches the others as it is made.
//
// Browsers start audio only after a click or a key on the page; until then nothing plays. A
// hidden page is silent, and drops what happens meanwhile rather than playing it on return.

const STORE_KEY = "drone-duel-sounds";
const DEFAULT_VOLUME = 0.72;
/** A sound plays at most once this often: a burst of events is one sound. */
const MIN_GAP_MS = { strike: 60, crash: 80, bump: 220, tick: 250, proximity: 100, fence: 500 };
const DEFAULT_GAP_MS = 40;
/** The truck's top speed in each gear, m/s: the revs climb through a gear and drop at a change. */
const GEARS = [6, 13, 21, 34];
/** Every engine's level: it runs the whole drive, so it stays under the other sounds. */
const ENGINE_LEVEL = 0.5;

const hz = (midi) => 440 * 2 ** ((midi - 69) / 12);
const clamp01 = (x) => Math.min(1, Math.max(0, Number(x) || 0));

// -- building blocks ---------------------------------------------------------------
// Each takes `a`, a sound being played: {ctx, out, t}, its AudioContext, the node it plays
// into, and when it starts. `at` is seconds after that.

/** A note's loudness from `t`: up to `peak` in `attack`, held `hold`, gone by `t + dur`. */
function envelope(a, t, { attack = 0.005, hold = 0, dur, peak }) {
  const g = a.ctx.createGain();
  g.gain.setValueAtTime(0, t);
  g.gain.linearRampToValueAtTime(peak, t + attack);
  if (hold > 0) g.gain.setValueAtTime(peak, t + attack + hold);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  return g;
}

/** A filter from `f` Hz, gliding to `f1` over `dur`. */
function biquad(a, t, dur, { type = "lowpass", f, f1 = f, q = 0.7 }) {
  const node = a.ctx.createBiquadFilter();
  node.type = type;
  node.Q.value = q;
  node.frequency.setValueAtTime(f, t);
  if (f1 !== f) node.frequency.exponentialRampToValueAtTime(f1, t + dur);
  return node;
}

/** One oscillator note, gliding from `f` to `f1`; `vibrato`: {rate, depth} in Hz. */
function tone(a, { type = "sine", f, f1 = f, at = 0, dur, gain, attack, hold, detune = 0, filter, vibrato }) {
  const t = a.t + at;
  const osc = a.ctx.createOscillator();
  osc.type = type;
  osc.detune.value = detune;
  osc.frequency.setValueAtTime(f, t);
  if (f1 !== f) osc.frequency.exponentialRampToValueAtTime(f1, t + dur);
  const sources = [osc];
  if (vibrato) {
    const lfo = a.ctx.createOscillator();
    const depth = a.ctx.createGain();
    lfo.frequency.value = vibrato.rate;
    depth.gain.value = vibrato.depth;
    lfo.connect(depth).connect(osc.frequency);
    sources.push(lfo);
  }
  const head = filter ? osc.connect(biquad(a, t, dur, filter)) : osc;
  head.connect(envelope(a, t, { attack, hold, dur, peak: gain })).connect(a.out);
  for (const s of sources) {
    s.start(t);
    s.stop(t + dur + 0.05);
  }
}

const noiseBuffers = new WeakMap();
/** Two seconds of white noise, made once for each context. */
function noiseBuffer(ctx) {
  let buffer = noiseBuffers.get(ctx);
  if (!buffer) {
    buffer = ctx.createBuffer(1, 2 * ctx.sampleRate, ctx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    noiseBuffers.set(ctx, buffer);
  }
  return buffer;
}

function noiseSource(ctx) {
  const src = ctx.createBufferSource();
  src.buffer = noiseBuffer(ctx);
  src.loop = true;
  return src;
}

/** A burst of filtered noise. */
function noise(a, { at = 0, dur, gain, attack, hold, filter }) {
  const t = a.t + at;
  const src = noiseSource(a.ctx);
  const head = filter ? src.connect(biquad(a, t, dur, filter)) : src;
  head.connect(envelope(a, t, { attack, hold, dur, peak: gain })).connect(a.out);
  src.start(t, Math.random() * 1.5);
  src.stop(t + dur + 0.05);
}

/** A struck bell (FM): a whole-number `ratio` rings sweet, another clangs. */
function bell(a, { f, at = 0, dur, gain, ratio = 2, index = 1.5 }) {
  const t = a.t + at;
  const carrier = a.ctx.createOscillator();
  const mod = a.ctx.createOscillator();
  const depth = a.ctx.createGain();
  carrier.frequency.value = f;
  mod.frequency.value = f * ratio;
  depth.gain.setValueAtTime(f * index, t);
  depth.gain.exponentialRampToValueAtTime(f * index * 0.02 + 0.01, t + dur);
  mod.connect(depth).connect(carrier.frequency);
  carrier.connect(envelope(a, t, { attack: 0.002, dur, peak: gain })).connect(a.out);
  for (const s of [carrier, mod]) {
    s.start(t);
    s.stop(t + dur + 0.05);
  }
}

/** A plucked note: bright at first, mellowing as it fades. */
function pluck(a, { f, at = 0, dur, gain }) {
  tone(a, { type: "triangle", f, at, dur, gain, attack: 0.002, filter: { f: f * 6, f1: f * 1.5 } });
  tone(a, { f: f * 2, at, dur: dur * 0.4, gain: gain * 0.2, attack: 0.002 });
}

/** A chord of slightly detuned pairs, slow to rise. */
function pad(a, { notes, type = "sawtooth", at = 0, dur, gain, attack = 0.25, hold = 0.3, filter }) {
  for (const f of notes) {
    for (const detune of [-7, 7]) tone(a, { type, f, at, dur, gain: gain / 2, attack, hold, detune, filter });
  }
}

/** A knuckle on a door. */
function knock(a, { at = 0, f = 180, gain = 0.4 }) {
  tone(a, { f, f1: f * 0.7, at, dur: 0.09, gain, attack: 0.001 });
  noise(a, { at, dur: 0.05, gain: gain * 0.5, filter: { type: "bandpass", f: 700, q: 1 } });
}

// -- the engine: a voice that runs while the truck is on the map ----------------------

/** A voice's nodes, faded out and stopped. */
function fadeOut(ctx, bus, sources) {
  const t = ctx.currentTime;
  if (bus.gain.cancelAndHoldAtTime) bus.gain.cancelAndHoldAtTime(t);
  else bus.gain.cancelScheduledValues(t);
  bus.gain.setTargetAtTime(0, t, 0.05);
  for (const s of sources) s.stop(t + 0.4);
}

function engineBus(ctx, out, filterType, q) {
  const bus = ctx.createGain();
  bus.gain.value = 0;
  const level = ctx.createGain();
  level.gain.value = ENGINE_LEVEL;
  bus.connect(level).connect(out);
  const filter = ctx.createBiquadFilter();
  filter.type = filterType;
  filter.Q.value = q;
  filter.connect(bus);
  return { bus, filter };
}

function startAll(sources) {
  for (const s of sources) s.start();
}

/** A six-cylinder diesel: a sawtooth at the firing rate, an octave under it, filtered. */
function dieselEngine(ctx, out) {
  const { bus, filter } = engineBus(ctx, out, "lowpass", 1.8);
  const saw = ctx.createOscillator();
  saw.type = "sawtooth";
  saw.connect(filter);
  const sub = ctx.createOscillator();
  sub.type = "square";
  const subGain = ctx.createGain();
  subGain.gain.value = 0.35;
  sub.connect(subGain).connect(filter);
  const sources = [saw, sub];
  startAll(sources);
  return {
    set({ rpm, load }, now = false) {
      const t = ctx.currentTime;
      const k = now ? 0.005 : 0.07;
      const f = 34 + 110 * rpm;
      saw.frequency.setTargetAtTime(f, t, k);
      sub.frequency.setTargetAtTime(f / 2, t, k);
      filter.frequency.setTargetAtTime(160 + 3 * f + 700 * load, t, k);
      bus.gain.setTargetAtTime(0.045 + 0.03 * rpm + 0.06 * load, t, 0.12);
    },
    stop: () => fadeOut(ctx, bus, sources),
  };
}

/** Low filtered noise and a hum: an engine heard from inside the cab. */
function rumbleEngine(ctx, out) {
  const { bus, filter } = engineBus(ctx, out, "lowpass", 1);
  const grit = noiseSource(ctx);
  grit.connect(filter);
  const hum = ctx.createOscillator();
  const humGain = ctx.createGain();
  humGain.gain.value = 0.5;
  hum.connect(humGain).connect(bus);
  const sources = [grit, hum];
  startAll(sources);
  return {
    set({ rpm, load }, now = false) {
      const t = ctx.currentTime;
      const k = now ? 0.005 : 0.08;
      filter.frequency.setTargetAtTime(90 + 350 * rpm + 250 * load, t, k);
      hum.frequency.setTargetAtTime(28 + 70 * rpm, t, k);
      bus.gain.setTargetAtTime(0.075 + 0.04 * rpm + 0.065 * load, t, 0.12);
    },
    stop: () => fadeOut(ctx, bus, sources),
  };
}

/** An electric motor's whine, rising with the road speed rather than the revs. */
function electricEngine(ctx, out) {
  const { bus, filter } = engineBus(ctx, out, "lowpass", 0.7);
  filter.frequency.value = 2500;
  const whine = ctx.createOscillator();
  const over = ctx.createOscillator();
  const body = ctx.createOscillator();
  body.type = "triangle";
  const overGain = ctx.createGain();
  overGain.gain.value = 0.25;
  const bodyGain = ctx.createGain();
  bodyGain.gain.value = 0.6;
  whine.connect(filter);
  over.connect(overGain).connect(filter);
  body.connect(bodyGain).connect(filter);
  const sources = [whine, over, body];
  startAll(sources);
  return {
    set({ speed, load }, now = false) {
      const t = ctx.currentTime;
      const k = now ? 0.005 : 0.1;
      const f = 140 + 900 * speed;
      whine.frequency.setTargetAtTime(f, t, k);
      over.frequency.setTargetAtTime(f * 2.02, t, k);
      body.frequency.setTargetAtTime(f / 4, t, k);
      bus.gain.setTargetAtTime(0.025 + 0.04 * speed + 0.035 * load, t, 0.12);
    },
    stop: () => fadeOut(ctx, bus, sources),
  };
}

/** Drone closing pings once the nearest drone is this near the truck, m. */
export const PROXIMITY_M = 150;

/** Seconds between Drone closing's pings with the nearest drone `range` m from the truck; null
 * when it is too far to ping. */
export function proximityGap(range) {
  if (!(range < PROXIMITY_M)) return null;
  return 0.22 + (1.6 * range) / PROXIMITY_M;
}

/** What the engine does at `speed` m/s and `throttle` -1..1: revs 0 (idle) to 1, load 0..1,
 * and the speed as a fraction of the top. */
export function revs(speed, throttle) {
  const v = Math.abs(speed);
  let gear = 0;
  let low = 0;
  while (gear < GEARS.length - 1 && v >= GEARS[gear]) low = GEARS[gear++];
  const frac = Math.min(1, (v - low) / (GEARS[gear] - low));
  const load = clamp01(Math.abs(throttle));
  const rpm = (gear === 0 ? 0.8 * frac : 0.3 + 0.65 * frac) + 0.12 * load;
  return { rpm: Math.min(1, rpm), load, speed: Math.min(1, v / GEARS.at(-1)) };
}

// -- the sounds ------------------------------------------------------------------
// Each: label, when it plays, its default variant, and the variants: {label, play(a, opts)},
// or for the engine {label, engine(ctx, out)}. Every sound can also be "off".

export const SOUNDS = {
  strike: {
    label: "Strike",
    when: "A drone strikes the truck. Both players hear it.",
    default: "boom",
    variants: {
      thump: {
        label: "Heavy thump",
        play(a) {
          tone(a, { f: 150, f1: 42, dur: 0.55, gain: 0.8, attack: 0.003 });
          tone(a, { type: "triangle", f: 95, f1: 50, dur: 0.3, gain: 0.25, attack: 0.003 });
          noise(a, { dur: 0.25, gain: 0.35, filter: { f: 1800, f1: 250 } });
        },
      },
      clang: {
        label: "Metal clang",
        play(a) {
          for (const [ratio, gain, dur] of [
            [1, 0.22, 1],
            [2.76, 0.14, 0.6],
            [5.4, 0.08, 0.4],
            [8.93, 0.04, 0.25],
          ]) {
            tone(a, { f: 196 * ratio, dur, gain, attack: 0.002 });
          }
          tone(a, { f: 110, f1: 55, dur: 0.3, gain: 0.5, attack: 0.002 });
          noise(a, { dur: 0.05, gain: 0.25, filter: { type: "bandpass", f: 3200, q: 1.2 } });
        },
      },
      boom: {
        label: "Distant boom",
        play(a) {
          noise(a, { dur: 1.5, gain: 0.55, attack: 0.008, filter: { f: 1100, f1: 90 } });
          tone(a, { f: 72, f1: 32, dur: 1.1, gain: 0.55, attack: 0.005 });
        },
      },
      zap: {
        label: "Arcade hit",
        play(a) {
          tone(a, { type: "triangle", f: 880, f1: 110, dur: 0.28, gain: 0.55, attack: 0.002 });
          tone(a, { type: "square", f: 440, f1: 55, dur: 0.22, gain: 0.11, attack: 0.002, filter: { f: 2200 } });
          noise(a, { dur: 0.12, gain: 0.4, filter: { type: "bandpass", f: 1400, q: 0.8 } });
        },
      },
    },
  },
  score: {
    label: "Score",
    when: "Your drone struck the truck: +1. Only the pilot hears it, after the strike.",
    default: "coin",
    variants: {
      chime: {
        label: "Two-note chime",
        play(a) {
          for (const [i, n] of [76, 83].entries()) {
            tone(a, { f: hz(n), at: i * 0.09, dur: 0.9, gain: 0.2, attack: 0.004 });
            tone(a, { f: hz(n) * 2, at: i * 0.09, dur: 0.35, gain: 0.05, attack: 0.004 });
          }
        },
      },
      coin: {
        label: "Coin",
        play(a) {
          tone(a, { type: "triangle", f: hz(83), dur: 0.09, gain: 0.22, attack: 0.002, hold: 0.05 });
          tone(a, { type: "triangle", f: hz(88), at: 0.08, dur: 0.45, gain: 0.22, attack: 0.002 });
        },
      },
      arpeggio: {
        label: "Rising pluck",
        play(a) {
          for (const [i, n] of [72, 76, 79, 84].entries()) {
            pluck(a, { f: hz(n), at: i * 0.065, dur: i === 3 ? 0.8 : 0.3, gain: 0.22 });
          }
        },
      },
      bell: {
        label: "Bright bell",
        play(a) {
          bell(a, { f: hz(81), dur: 1.4, gain: 0.18, ratio: 3.5, index: 1.2 });
        },
      },
    },
  },
  crash: {
    label: "Drone down",
    when: "A drone is out without striking: the ground, the scenery, or another drone.",
    default: "poof",
    variants: {
      poof: {
        label: "Soft poof",
        play(a) {
          noise(a, { dur: 0.7, gain: 0.45, attack: 0.03, filter: { f: 2400, f1: 250 } });
          tone(a, { f: 85, f1: 40, dur: 0.4, gain: 0.35, attack: 0.01 });
        },
      },
      crunch: {
        label: "Crunch",
        play(a) {
          noise(a, { dur: 0.3, gain: 0.5, filter: { type: "bandpass", f: 900, f1: 500, q: 0.9 } });
          noise(a, { dur: 0.5, gain: 0.35, filter: { f: 400 } });
          tone(a, { f: 110, f1: 48, dur: 0.3, gain: 0.45, attack: 0.003 });
        },
      },
      spindown: {
        label: "Motors die",
        play(a) {
          tone(a, { type: "sawtooth", f: 420, f1: 70, dur: 0.6, gain: 0.12, filter: { f: 1600, f1: 300 } });
          tone(a, { f: 840, f1: 140, dur: 0.5, gain: 0.04 });
          tone(a, { f: 120, f1: 45, at: 0.5, dur: 0.3, gain: 0.5, attack: 0.003 });
          noise(a, { at: 0.5, dur: 0.2, gain: 0.2, filter: { f: 700 } });
        },
      },
    },
  },
  turnStart: {
    label: "Turn starts",
    when: "A turn goes live: the pilot's drones may go.",
    default: "lights",
    variants: {
      lights: {
        label: "Start lights",
        play(a) {
          for (const at of [0, 0.32]) {
            tone(a, { type: "triangle", f: hz(69), at, dur: 0.16, gain: 0.16, attack: 0.004, hold: 0.06, filter: { f: 2400 } });
          }
          tone(a, { type: "triangle", f: hz(81), at: 0.64, dur: 0.6, gain: 0.18, attack: 0.004, hold: 0.2, filter: { f: 3000 } });
        },
      },
      horn: {
        label: "Soft horn",
        play(a) {
          pad(a, { notes: [57, 64, 69].map(hz), dur: 0.8, gain: 0.12, attack: 0.04, hold: 0.35, filter: { f: 700, f1: 1400 } });
        },
      },
      rise: {
        label: "Sweep and bell",
        play(a) {
          noise(a, { dur: 0.6, gain: 0.24, attack: 0.5, filter: { type: "bandpass", f: 300, f1: 3000, q: 1.5 } });
          bell(a, { f: hz(81), at: 0.52, dur: 1.2, gain: 0.24, ratio: 2, index: 1 });
        },
      },
    },
  },
  turnEnd: {
    label: "Turn over",
    when: "A turn ends (time up, every drone out) or is called off.",
    default: "gong",
    variants: {
      fall: {
        label: "Two tones down",
        play(a) {
          for (const [i, n] of [76, 69].entries()) {
            tone(a, { f: hz(n), at: i * 0.2, dur: 0.5, gain: 0.2, attack: 0.006 });
            tone(a, { f: hz(n) * 2, at: i * 0.2, dur: 0.25, gain: 0.04, attack: 0.006 });
          }
        },
      },
      whistle: {
        label: "Whistle",
        play(a) {
          tone(a, { f: 1650, dur: 0.6, gain: 0.1, attack: 0.02, hold: 0.35, vibrato: { rate: 28, depth: 45 } });
        },
      },
      gong: {
        label: "Gong",
        play(a) {
          bell(a, { f: 110, dur: 2.4, gain: 0.16, ratio: 1.41, index: 3 });
          tone(a, { f: 55, dur: 1.6, gain: 0.12, attack: 0.01 });
        },
      },
    },
  },
  tick: {
    label: "Last seconds",
    when: "Each of a turn's last 10 seconds; higher for the last 3.",
    default: "beep",
    variants: {
      wood: {
        label: "Wood block",
        play(a, { urgent } = {}) {
          const f = urgent ? 1250 : 900;
          tone(a, { f, f1: f * 0.85, dur: 0.07, gain: 0.3, attack: 0.001 });
          noise(a, { dur: 0.02, gain: 0.08, filter: { type: "bandpass", f: f * 2, q: 3 } });
        },
      },
      beep: {
        label: "Soft beep",
        play(a, { urgent } = {}) {
          tone(a, { f: urgent ? 1320 : 880, dur: 0.1, gain: 0.22, attack: 0.004 });
        },
      },
      click: {
        label: "Clock tick",
        play(a, { urgent } = {}) {
          const f = urgent ? 4200 : 2600;
          noise(a, { dur: 0.03, gain: 1.1, filter: { type: "bandpass", f, q: 2.5 } });
          if (urgent) noise(a, { at: 0.06, dur: 0.03, gain: 0.9, filter: { type: "bandpass", f, q: 2.5 } });
        },
      },
    },
  },
  win: {
    label: "You win",
    when: "The match is over and you won.",
    default: "fanfare",
    variants: {
      bells: {
        label: "Bells",
        play(a) {
          for (const [i, n] of [72, 76, 79, 84, 88].entries()) {
            bell(a, { f: hz(n), at: i * 0.11, dur: i === 4 ? 1.8 : 1, gain: 0.14, ratio: 2, index: 1.2 });
          }
        },
      },
      fanfare: {
        label: "Fanfare",
        play(a) {
          for (const [i, n] of [72, 76, 79].entries()) {
            tone(a, { type: "triangle", f: hz(n), at: i * 0.12, dur: 0.24, gain: 0.18, attack: 0.005, hold: 0.06, filter: { f: 3000 } });
          }
          pad(a, { type: "triangle", notes: [72, 76, 79, 84].map(hz), at: 0.38, dur: 1.4, gain: 0.12, attack: 0.02, hold: 0.4, filter: { f: 3200, f1: 1200 } });
        },
      },
      swell: {
        label: "Chord swell",
        play(a) {
          pad(a, { notes: [60, 64, 67, 72].map(hz), dur: 2.2, gain: 0.08, attack: 0.45, hold: 0.5, filter: { f: 500, f1: 2600 } });
          bell(a, { f: hz(84), at: 0.45, dur: 1.5, gain: 0.1, ratio: 2, index: 0.8 });
        },
      },
    },
  },
  lose: {
    label: "You lose",
    when: "The match is over and the other player won. A draw plays Turn over.",
    default: "trombone",
    variants: {
      fall: {
        label: "Falling notes",
        play(a) {
          for (const [i, n] of [64, 62, 60].entries()) {
            tone(a, { type: "triangle", f: hz(n), at: i * 0.22, dur: 0.35, gain: 0.18, attack: 0.006, filter: { f: 1800 } });
          }
          tone(a, { type: "triangle", f: hz(57), at: 0.66, dur: 1.2, gain: 0.18, attack: 0.006, hold: 0.2, filter: { f: 1800, f1: 600 } });
        },
      },
      trombone: {
        label: "Sad trombone",
        play(a) {
          for (const [i, n] of [55, 54, 53].entries()) {
            tone(a, { type: "sawtooth", f: hz(n), at: i * 0.3, dur: 0.32, gain: 0.16, attack: 0.03, hold: 0.15, filter: { f: 900, q: 2 } });
          }
          tone(a, {
            type: "sawtooth",
            f: hz(52),
            f1: hz(51),
            at: 0.9,
            dur: 1.1,
            gain: 0.16,
            attack: 0.03,
            hold: 0.6,
            filter: { f: 900, f1: 400, q: 2 },
            vibrato: { rate: 6, depth: 4 },
          });
        },
      },
      pad: {
        label: "Low pad",
        play(a) {
          pad(a, { notes: [45, 48, 52].map(hz), dur: 2.4, gain: 0.13, attack: 0.3, hold: 0.6, filter: { f: 900, f1: 300 } });
        },
      },
    },
  },
  join: {
    label: "Opponent joins",
    when: "The other player comes into the room.",
    default: "up",
    variants: {
      up: {
        label: "Two notes up",
        play(a) {
          tone(a, { f: hz(79), dur: 0.2, gain: 0.16, attack: 0.004 });
          tone(a, { f: hz(86), at: 0.1, dur: 0.35, gain: 0.16, attack: 0.004 });
        },
      },
      knock: {
        label: "Knock knock",
        play(a) {
          knock(a, {});
          knock(a, { at: 0.13 });
        },
      },
      bell: {
        label: "Bell",
        play(a) {
          bell(a, { f: hz(86), dur: 0.9, gain: 0.12, ratio: 2, index: 0.8 });
        },
      },
    },
  },
  leave: {
    label: "Opponent leaves",
    when: "The other player leaves the room.",
    default: "down",
    variants: {
      down: {
        label: "Two notes down",
        play(a) {
          tone(a, { f: hz(86), dur: 0.2, gain: 0.16, attack: 0.004 });
          tone(a, { f: hz(79), at: 0.1, dur: 0.35, gain: 0.16, attack: 0.004 });
        },
      },
      knock: {
        label: "One knock",
        play(a) {
          knock(a, { f: 140 });
        },
      },
      bell: {
        label: "Bells down",
        play(a) {
          bell(a, { f: hz(79), dur: 0.6, gain: 0.12, ratio: 2, index: 0.8 });
          bell(a, { f: hz(74), at: 0.14, dur: 0.9, gain: 0.12, ratio: 2, index: 0.8 });
        },
      },
    },
  },
  engine: {
    label: "Truck engine",
    when: "While you drive, following the truck's speed and throttle.",
    default: "diesel",
    variants: {
      diesel: { label: "Diesel", engine: dieselEngine },
      rumble: { label: "Cab rumble", engine: rumbleEngine },
      electric: { label: "Electric", engine: electricEngine },
    },
  },
  bump: {
    label: "Truck bump",
    when: "The truck hits something hard; louder the harder.",
    default: "thud",
    variants: {
      thud: {
        label: "Thud",
        play(a, { intensity } = {}) {
          const k = 0.35 + 0.65 * clamp01(intensity ?? 0.6);
          tone(a, { f: 95, f1: 42, dur: 0.28, gain: 0.7 * k, attack: 0.002 });
          noise(a, { dur: 0.12, gain: 0.35 * k, filter: { f: 700 } });
        },
      },
      clunk: {
        label: "Clunk",
        play(a, { intensity } = {}) {
          const k = 0.35 + 0.65 * clamp01(intensity ?? 0.6);
          for (const [f, gain, dur] of [
            [170, 0.25, 0.25],
            [410, 0.12, 0.16],
            [690, 0.06, 0.1],
          ]) {
            tone(a, { f, dur, gain: gain * k, attack: 0.002 });
          }
          tone(a, { f: 85, f1: 50, dur: 0.2, gain: 0.4 * k, attack: 0.002 });
          noise(a, { dur: 0.04, gain: 0.15 * k, filter: { type: "bandpass", f: 1600, q: 1 } });
        },
      },
      crunch: {
        label: "Crunch",
        play(a, { intensity } = {}) {
          const k = 0.35 + 0.65 * clamp01(intensity ?? 0.6);
          noise(a, { dur: 0.25, gain: 0.7 * k, filter: { type: "bandpass", f: 800, f1: 400, q: 0.9 } });
          noise(a, { dur: 0.07, gain: 0.2 * k, filter: { type: "highpass", f: 2500 } });
          tone(a, { f: 70, f1: 40, dur: 0.2, gain: 0.4 * k, attack: 0.002 });
        },
      },
    },
  },
  proximity: {
    label: "Drone closing",
    when: "Driving in a turn: a drone near the truck. Quicker and higher the nearer it is.",
    default: "ping",
    variants: {
      blip: {
        label: "Soft blip",
        play(a, { near } = {}) {
          tone(a, { type: "triangle", f: 620 * (1 + 0.3 * clamp01(near)), dur: 0.07, gain: 0.18, attack: 0.003 });
        },
      },
      ping: {
        label: "Sonar ping",
        play(a, { near } = {}) {
          const f = 1050 * (1 + 0.15 * clamp01(near));
          tone(a, { f, dur: 0.35, gain: 0.1, attack: 0.002 });
          tone(a, { f, at: 0.17, dur: 0.3, gain: 0.03, attack: 0.002 });
        },
      },
      heartbeat: {
        label: "Heartbeat",
        play(a) {
          tone(a, { f: 62, f1: 40, dur: 0.13, gain: 0.35, attack: 0.004 });
          tone(a, { f: 62, f1: 40, at: 0.17, dur: 0.13, gain: 0.25, attack: 0.004 });
        },
      },
    },
  },
  fence: {
    label: "Outside the fence",
    when: "Driving: each second the truck is outside the fence, before it goes back.",
    default: "buzz",
    variants: {
      alert: {
        label: "Two-tone",
        play(a) {
          tone(a, { f: 740, dur: 0.13, gain: 0.14, attack: 0.004 });
          tone(a, { f: 587, at: 0.15, dur: 0.2, gain: 0.14, attack: 0.004 });
        },
      },
      buzz: {
        label: "Low buzz",
        play(a) {
          tone(a, { type: "square", f: 110, dur: 0.3, gain: 0.06, attack: 0.01, hold: 0.12, filter: { f: 500 } });
        },
      },
      blips: {
        label: "Three blips",
        play(a) {
          for (const at of [0, 0.09, 0.18]) tone(a, { type: "triangle", f: 1046, at, dur: 0.06, gain: 0.18, attack: 0.002 });
        },
      },
    },
  },
};

// -- the player ------------------------------------------------------------------

function loadSettings() {
  try {
    const saved = JSON.parse(localStorage.getItem(STORE_KEY) ?? "{}");
    return {
      enabled: saved.enabled !== false,
      volume: Number.isFinite(saved.volume) ? clamp01(saved.volume) : DEFAULT_VOLUME,
      choices: typeof saved.choices === "object" && saved.choices ? saved.choices : {},
    };
  } catch {
    return { enabled: true, volume: DEFAULT_VOLUME, choices: {} };
  }
}

class Sfx {
  constructor() {
    /** @type {AudioContext | null} made at the page's first click or key. */
    this.ctx = null;
    this._settings = loadSettings();
    this._last = new Map();
    this._listeners = new Set();
    const unlock = () => this._unlock();
    addEventListener("pointerdown", unlock, { capture: true });
    addEventListener("keydown", unlock, { capture: true });
    addEventListener("storage", (e) => {
      if (e.key !== STORE_KEY) return;
      this._settings = loadSettings();
      this._changed(false);
    });
    document.addEventListener("visibilitychange", () => {
      if (!this.ctx) return;
      if (document.hidden) this.ctx.suspend();
      else this.ctx.resume();
    });
  }

  get enabled() {
    return this._settings.enabled;
  }

  get volume() {
    return this._settings.volume;
  }

  /** Sound `id`'s chosen variant, or "off". */
  choice(id) {
    const sound = SOUNDS[id];
    const chosen = this._settings.choices[id];
    return chosen === "off" || chosen in (sound?.variants ?? {}) ? chosen : sound?.default;
  }

  choose(id, variant) {
    this._settings.choices[id] = variant;
    this._changed();
  }

  setEnabled(on) {
    this._settings.enabled = Boolean(on);
    this._changed();
  }

  toggle() {
    this.setEnabled(!this.enabled);
  }

  setVolume(volume) {
    this._settings.volume = clamp01(volume);
    this._changed();
  }

  /** Every sound back to its default variant; the volume and on/off stay. */
  resetChoices() {
    this._settings.choices = {};
    this._changed();
  }

  /** Call `fn` on every change, here or on another page; returns a function to stop. */
  onChange(fn) {
    this._listeners.add(fn);
    return () => this._listeners.delete(fn);
  }

  /** Play sound `id` as chosen, if sound is on. `opts` as the sound takes them. */
  play(id, opts = {}) {
    if (this.enabled) this._play(id, this.choice(id), opts, this._game, false);
  }

  /** Play variant `variant` of sound `id` whether sound is on or not: the sound lab's. */
  preview(id, variant, opts = {}) {
    this._unlock();
    this._play(id, variant, opts, this._master, true);
  }

  /** The truck's engine, following its chosen variant (or `variant`, sound on or off). */
  engine(variant = null) {
    return new EngineSound(this, variant);
  }

  /** Whether sounds can play now: started by a click or key, the page shown. */
  get live() {
    return this.ctx?.state === "running";
  }

  _play(id, variant, opts, out, preview) {
    const play = SOUNDS[id]?.variants[variant]?.play;
    if (!play || !this.ctx || (!this.live && !preview)) return;
    const now = performance.now();
    if (!preview && now - (this._last.get(id) ?? -Infinity) < (MIN_GAP_MS[id] ?? DEFAULT_GAP_MS)) return;
    this._last.set(id, now);
    play({ ctx: this.ctx, out, t: this.ctx.currentTime + 0.01 }, opts);
  }

  _unlock() {
    if (!this.ctx) {
      const ctx = new AudioContext();
      // Sounds stacking up stay clean: a gentle limiter at the end.
      const limiter = ctx.createDynamicsCompressor();
      limiter.threshold.value = -16;
      limiter.knee.value = 10;
      limiter.ratio.value = 4;
      limiter.attack.value = 0.003;
      limiter.release.value = 0.25;
      limiter.connect(ctx.destination);
      this._master = ctx.createGain();
      this._master.connect(limiter);
      this._game = ctx.createGain();
      this._game.connect(this._master);
      this.ctx = ctx;
      this._applyLevels();
    }
    if (this.ctx.state === "suspended" && !document.hidden) this.ctx.resume();
  }

  _applyLevels() {
    if (!this.ctx) return;
    const t = this.ctx.currentTime;
    this._master.gain.setTargetAtTime(this.volume, t, 0.02);
    this._game.gain.setTargetAtTime(this.enabled ? 1 : 0, t, 0.02);
  }

  _changed(save = true) {
    if (save) {
      try {
        localStorage.setItem(STORE_KEY, JSON.stringify(this._settings));
      } catch {
        // Storage blocked: the choices last for this page only.
      }
    }
    this._applyLevels();
    for (const fn of this._listeners) fn();
  }
}

/** The engine's voice, made when it can be heard and dropped when it cannot (sound off, the
 * truck off the map, the page hidden), and rebuilt when its variant changes. */
class EngineSound {
  constructor(sfx, variant) {
    this.sfx = sfx;
    this.forced = variant;
    this.voice = null;
    this.variant = null;
    this.on = false;
    this.state = revs(0, 0);
    this._unsubscribe = sfx.onChange(() => this._sync());
  }

  /** Each frame: the truck's speed, m/s, and throttle -1..1; `on` false silences it. */
  update(speed, throttle, on = true) {
    this.on = on;
    this.state = revs(speed, throttle);
    this._sync();
    this.voice?.set(this.state);
  }

  stop() {
    this._unsubscribe();
    this.on = false;
    this._sync();
  }

  _wanted() {
    const sfx = this.sfx;
    if (!this.on || !sfx.live) return null;
    const variant = this.forced ?? (sfx.enabled ? sfx.choice("engine") : null);
    return SOUNDS.engine.variants[variant]?.engine ? variant : null;
  }

  _sync() {
    const want = this._wanted();
    if (want === this.variant) return;
    this.voice?.stop();
    this.voice = null;
    this.variant = want;
    if (!want) return;
    const sfx = this.sfx;
    this.voice = SOUNDS.engine.variants[want].engine(sfx.ctx, this.forced ? sfx._master : sfx._game);
    this.voice.set(this.state, true);
  }
}

/** The page's sounds. */
export const sfx = new Sfx();
