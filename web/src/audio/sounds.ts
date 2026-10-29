// The game's sounds, made in the browser with Web Audio: no files to fetch (the old sounds.js,
// ported). Loudness follows prefs.audio: `on`, `master`, and the `motors` and `effects` buses.
//
// Browsers start audio only after a click or a key on the page; until then nothing plays.
// `sounds.unlock()` on the first gesture starts it (the page's first pointerdown or keydown
// does too). A hidden page is silent, and drops what happens meanwhile rather than playing it
// on return.
//
// Two kinds of sound:
// - continuous: the player's own motor or engine (`motor`, every frame; `stopMotor`), and the
//   other vehicles heard from where they are (`heard`, every frame);
// - one-shot effects (`strike`, `crash`, `gate`, …), each at most once in a short gap, so a
//   burst of events is one sound.

import { effect } from "alien-signals";
import { prefs, updatePrefs } from "../state/prefs.ts";
import { clamp01, doppler, droneGain, hz, QUAD_MOTOR, truckGain, WING_MOTOR } from "./levels.ts";

export type MotorKind = "multirotor" | "fixedwing" | "ground";

/** The whole mix at master 1, under a gentle limiter. */
const VOLUME = 0.72;
/** A sound plays at most once this often: a burst of events is one sound. */
const MIN_GAP_MS: Record<string, number> = {
  strike: 60,
  crash: 80,
  bump: 220,
  tick: 250,
  proximity: 100,
  fence: 500,
  gate: 120,
  click: 30,
};
const DEFAULT_GAP_MS = 40;
/** The truck's engine runs the whole drive, so it stays under the other sounds. */
const ENGINE_LEVEL = 0.5;
/** A quad in FPV cruising is about as loud as the truck's engine to its driver, flat out still
 * under a strike. */
const DRONE_LEVEL = 0.22;
/** The wing's single motor, a little under the quad's four. */
const WING_LEVEL = 0.2;
/** Each motor a little off the others, cents: four never quite in tune, so they beat. */
const MOTOR_DETUNE = [-14, 9, -5, 17];
/** Each other vehicle a little off the rest, cents at most: never locked in phase. */
const VOICE_SPREAD = 25;
/** Vehicles heard together are turned down by their summed loudness to this power: five as near
 * as one are about 2 dB louder than it, not 7. */
const CROWD = 0.75;
/** The nearest others are heard, this many at most, out to this far. */
const VOICES = 6;
const HEAR_M = 600;
/** A range changing faster than this, m/s, is a jump (a respawn), not speed. */
const JUMP_MS = 120;

// -- building blocks ---------------------------------------------------------------
// Each takes `a`, a sound being played: its AudioContext, the node it plays into, and when it
// starts. `at` is seconds after that.

interface Play {
  ctx: AudioContext;
  out: AudioNode;
  t: number;
}

interface Shape {
  attack?: number;
  hold?: number;
  dur: number;
  peak: number;
}

/** A note's loudness from `t`: up to `peak` in `attack`, held `hold`, gone by `t + dur`. */
function envelope(a: Play, t: number, { attack = 0.005, hold = 0, dur, peak }: Shape): GainNode {
  const g = a.ctx.createGain();
  g.gain.setValueAtTime(0, t);
  g.gain.linearRampToValueAtTime(peak, t + attack);
  if (hold > 0) g.gain.setValueAtTime(peak, t + attack + hold);
  g.gain.exponentialRampToValueAtTime(0.0001, t + dur);
  return g;
}

interface Filter {
  type?: BiquadFilterType;
  f: number;
  f1?: number;
  q?: number;
}

/** A filter from `f` Hz, gliding to `f1` over `dur`. */
function biquad(a: Play, t: number, dur: number, { type = "lowpass", f, f1 = f, q = 0.7 }: Filter) {
  const node = a.ctx.createBiquadFilter();
  node.type = type;
  node.Q.value = q;
  node.frequency.setValueAtTime(f, t);
  if (f1 !== f) node.frequency.exponentialRampToValueAtTime(f1, t + dur);
  return node;
}

interface Tone {
  type?: OscillatorType;
  f: number;
  f1?: number;
  at?: number;
  dur: number;
  gain: number;
  attack?: number;
  hold?: number;
  detune?: number;
  filter?: Filter;
  vibrato?: { rate: number; depth: number };
}

/** One oscillator note, gliding from `f` to `f1`; `vibrato` in Hz. */
function tone(a: Play, o: Tone): void {
  const {
    type = "sine",
    f,
    f1 = f,
    at = 0,
    dur,
    gain,
    attack,
    hold,
    detune = 0,
    filter,
    vibrato,
  } = o;
  const t = a.t + at;
  const osc = a.ctx.createOscillator();
  osc.type = type;
  osc.detune.value = detune;
  osc.frequency.setValueAtTime(f, t);
  if (f1 !== f) osc.frequency.exponentialRampToValueAtTime(f1, t + dur);
  const sources: OscillatorNode[] = [osc];
  if (vibrato) {
    const lfo = a.ctx.createOscillator();
    const depth = a.ctx.createGain();
    lfo.frequency.value = vibrato.rate;
    depth.gain.value = vibrato.depth;
    lfo.connect(depth).connect(osc.frequency);
    sources.push(lfo);
  }
  const head: AudioNode = filter ? osc.connect(biquad(a, t, dur, filter)) : osc;
  head.connect(envelope(a, t, { attack, hold, dur, peak: gain })).connect(a.out);
  for (const s of sources) {
    s.start(t);
    s.stop(t + dur + 0.05);
  }
}

const noiseBuffers = new WeakMap<BaseAudioContext, AudioBuffer>();
/** Two seconds of white noise, made once for each context. */
function noiseBuffer(ctx: BaseAudioContext): AudioBuffer {
  let buffer = noiseBuffers.get(ctx);
  if (!buffer) {
    buffer = ctx.createBuffer(1, 2 * ctx.sampleRate, ctx.sampleRate);
    const data = buffer.getChannelData(0);
    for (let i = 0; i < data.length; i++) data[i] = Math.random() * 2 - 1;
    noiseBuffers.set(ctx, buffer);
  }
  return buffer;
}

function noiseSource(ctx: BaseAudioContext): AudioBufferSourceNode {
  const src = ctx.createBufferSource();
  src.buffer = noiseBuffer(ctx);
  src.loop = true;
  return src;
}

/** A burst of filtered noise. */
function noise(
  a: Play,
  { at = 0, dur, gain, attack, hold, filter }: Omit<Tone, "f" | "type"> & { filter?: Filter },
): void {
  const t = a.t + at;
  const src = noiseSource(a.ctx);
  const head: AudioNode = filter ? src.connect(biquad(a, t, dur, filter)) : src;
  head.connect(envelope(a, t, { attack, hold, dur, peak: gain })).connect(a.out);
  src.start(t, Math.random() * 1.5);
  src.stop(t + dur + 0.05);
}

/** A struck bell (FM): a whole-number `ratio` rings sweet, another clangs. */
function bell(
  a: Play,
  {
    f,
    at = 0,
    dur,
    gain,
    ratio = 2,
    index = 1.5,
  }: { f: number; at?: number; dur: number; gain: number; ratio?: number; index?: number },
): void {
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

/** A chord of slightly detuned pairs, slow to rise. */
function pad(
  a: Play,
  {
    notes,
    type = "sawtooth",
    at = 0,
    dur,
    gain,
    attack = 0.25,
    hold = 0.3,
    filter,
  }: Omit<Tone, "f"> & { notes: number[] },
): void {
  for (const f of notes) {
    for (const detune of [-7, 7])
      tone(a, { type, f, at, dur, gain: gain / 2, attack, hold, detune, filter });
  }
}

// -- continuous voices ---------------------------------------------------------------

interface VoiceSet {
  /** 0..1: the throttle or load (louder, brighter). */
  level: number;
  /** 0..1: the motor or engine speed (pitch). */
  rpm: number;
  /** Each motor's speed 0..1, when known (the quad's four). */
  each?: readonly number[] | undefined;
  gain: number;
  pan: number;
  doppler: number;
}

interface Voice {
  set(s: VoiceSet, now?: boolean): void;
  stop(): void;
}

/** A voice's nodes, faded out and stopped. */
function fadeOut(ctx: AudioContext, bus: GainNode, sources: AudioScheduledSourceNode[]): void {
  const t = ctx.currentTime;
  bus.gain.cancelAndHoldAtTime(t);
  bus.gain.setTargetAtTime(0, t, 0.05);
  for (const s of sources) s.stop(t + 0.4);
}

/** A voice's output: its gain (for distance) and pan, then its fixed level. */
function voiceBus(ctx: AudioContext, out: AudioNode, level: number) {
  const bus = ctx.createGain();
  bus.gain.value = 0;
  const panner = ctx.createStereoPanner();
  const fixed = ctx.createGain();
  fixed.gain.value = level;
  bus.connect(panner).connect(fixed).connect(out);
  return { bus, panner };
}

/** A six-cylinder diesel: a sawtooth at the firing rate, an octave under it, filtered. */
function dieselVoice(ctx: AudioContext, out: AudioNode): Voice {
  const { bus, panner } = voiceBus(ctx, out, ENGINE_LEVEL);
  const filter = ctx.createBiquadFilter();
  filter.type = "lowpass";
  filter.Q.value = 1.8;
  const body = ctx.createGain();
  body.gain.value = 0;
  filter.connect(body).connect(bus);
  const saw = ctx.createOscillator();
  saw.type = "sawtooth";
  saw.connect(filter);
  const sub = ctx.createOscillator();
  sub.type = "square";
  const subGain = ctx.createGain();
  subGain.gain.value = 0.35;
  sub.connect(subGain).connect(filter);
  const sources = [saw, sub];
  for (const s of sources) s.start();
  return {
    set({ level, rpm, gain, pan, doppler }, now = false) {
      const t = ctx.currentTime;
      const k = now ? 0.005 : 0.07;
      const f = (34 + 110 * rpm) * doppler;
      saw.frequency.setTargetAtTime(f, t, k);
      sub.frequency.setTargetAtTime(f / 2, t, k);
      filter.frequency.setTargetAtTime(160 + 3 * f + 700 * level, t, k);
      body.gain.setTargetAtTime(0.045 + 0.03 * rpm + 0.06 * level, t, 0.12);
      bus.gain.setTargetAtTime(gain, t, now ? 0.005 : 0.05);
      panner.pan.setTargetAtTime(pan, t, now ? 0.005 : 0.05);
    },
    stop: () => fadeOut(ctx, bus, sources),
  };
}

/** Electric motors on props: each a sawtooth at its blade rate, through a lowpass that opens as
 * they speed up, with the motor's own whine above it, and the prop wash as noise. The quad has
 * four, a little out of tune; the wing one. */
function propVoice(
  ctx: AudioContext,
  out: AudioNode,
  motor: { maxRevS: number; blades: number; polePairs: number },
  detunes: number[],
  level: number,
  spread = 0,
): Voice {
  const { bus, panner } = voiceBus(ctx, out, level);
  const filter = ctx.createBiquadFilter();
  filter.type = "lowpass";
  filter.Q.value = 0.9;
  filter.connect(bus);
  const share = 4 / detunes.length;
  const motors = detunes.map((detune) => {
    const blade = ctx.createOscillator();
    blade.type = "sawtooth";
    blade.detune.value = detune + spread;
    const whine = ctx.createOscillator();
    whine.type = "triangle";
    whine.detune.value = detune + spread;
    const bladeGain = ctx.createGain();
    bladeGain.gain.value = 0;
    const whineGain = ctx.createGain();
    whineGain.gain.value = 0;
    blade.connect(bladeGain).connect(filter);
    whine.connect(whineGain).connect(bus);
    return { blade, whine, bladeGain, whineGain };
  });
  const wash = noiseSource(ctx);
  const washFilter = ctx.createBiquadFilter();
  washFilter.type = "bandpass";
  washFilter.Q.value = 0.7;
  const washGain = ctx.createGain();
  washGain.gain.value = 0;
  wash.connect(washFilter).connect(washGain).connect(bus);
  const sources: AudioScheduledSourceNode[] = [...motors.flatMap((m) => [m.blade, m.whine]), wash];
  for (const s of sources) s.start();
  let lastMean = 0;
  return {
    set({ level: load, rpm, each, gain, pan, doppler }, now = false) {
      const t = ctx.currentTime;
      const revs = motors.map((_, i) => clamp01(each?.[i] ?? rpm));
      const mean = revs.reduce((sum, r) => sum + r, 0) / revs.length;
      // Props spin up quicker than they run down.
      const k = now ? 0.005 : mean >= lastMean ? 0.04 : 0.15;
      lastMean = mean;
      for (const [i, m] of motors.entries()) {
        const r = revs[i]!;
        const revS = Math.max(1, motor.maxRevS * r * doppler);
        m.blade.frequency.setTargetAtTime(revS * motor.blades, t, k);
        m.whine.frequency.setTargetAtTime(revS * motor.polePairs, t, k);
        // Quiet at idle, near full by hover, a little louder still flat out.
        m.bladeGain.gain.setTargetAtTime(
          share * 0.16 * Math.min(1, 3 * r) ** 1.2 * (0.4 + r),
          t,
          k,
        );
        m.whineGain.gain.setTargetAtTime(share * 0.02 * r, t, k);
      }
      filter.frequency.setTargetAtTime((700 + 4200 * mean + 800 * clamp01(load)) * doppler, t, k);
      washFilter.frequency.setTargetAtTime((350 + 1600 * mean) * doppler, t, k);
      washGain.gain.setTargetAtTime(0.5 * mean * mean, t, k);
      bus.gain.setTargetAtTime(gain, t, now ? 0.005 : 0.05);
      panner.pan.setTargetAtTime(pan, t, now ? 0.005 : 0.05);
    },
    stop: () => fadeOut(ctx, bus, sources),
  };
}

function makeVoice(ctx: AudioContext, out: AudioNode, kind: MotorKind, spread = 0): Voice {
  if (kind === "ground") return dieselVoice(ctx, out);
  if (kind === "fixedwing") return propVoice(ctx, out, WING_MOTOR, [0], WING_LEVEL, spread);
  return propVoice(ctx, out, QUAD_MOTOR, MOTOR_DETUNE, DRONE_LEVEL, spread);
}

// -- the effects ---------------------------------------------------------------------

const EFFECTS = {
  /** A drone strikes the truck. Both players hear it. */
  strike(a: Play) {
    noise(a, { dur: 1.5, gain: 0.55, attack: 0.008, filter: { f: 1100, f1: 90 } });
    tone(a, { f: 72, f1: 32, dur: 1.1, gain: 0.55, attack: 0.005 });
  },
  /** Your drone struck the truck: +1. Only the pilot hears it, after the strike. */
  score(a: Play) {
    tone(a, { type: "triangle", f: hz(83), dur: 0.09, gain: 0.22, attack: 0.002, hold: 0.05 });
    tone(a, { type: "triangle", f: hz(88), at: 0.08, dur: 0.45, gain: 0.22, attack: 0.002 });
  },
  /** A vehicle is out without striking: the ground, the scenery, or another vehicle. */
  crash(a: Play) {
    noise(a, { dur: 0.7, gain: 0.45, attack: 0.03, filter: { f: 2400, f1: 250 } });
    tone(a, { f: 85, f1: 40, dur: 0.4, gain: 0.35, attack: 0.01 });
  },
  /** A race gate passed: a bright rising pair. */
  gate(a: Play) {
    tone(a, {
      type: "triangle",
      f: hz(88),
      dur: 0.1,
      gain: 0.18,
      attack: 0.002,
      hold: 0.04,
      filter: { f: 4000 },
    });
    tone(a, {
      type: "triangle",
      f: hz(95),
      at: 0.07,
      dur: 0.35,
      gain: 0.16,
      attack: 0.002,
      filter: { f: 4500 },
    });
  },
  /** A lap done: three rising notes and a bell. */
  lap(a: Play) {
    for (const [i, n] of [79, 83, 86].entries()) {
      tone(a, {
        type: "triangle",
        f: hz(n),
        at: i * 0.09,
        dur: 0.16,
        gain: 0.16,
        attack: 0.003,
        hold: 0.04,
        filter: { f: 3500 },
      });
    }
    bell(a, { f: hz(91), at: 0.27, dur: 1.2, gain: 0.12, ratio: 2, index: 1.2 });
  },
  /** The race start's lights: a low beep on each of 3, 2, 1, and a long high one to go. */
  countdown(a: Play, n: number) {
    if (n > 0) {
      tone(a, {
        type: "triangle",
        f: 880,
        dur: 0.22,
        gain: 0.22,
        attack: 0.003,
        hold: 0.12,
        filter: { f: 3000 },
      });
    } else {
      tone(a, {
        type: "triangle",
        f: 1760,
        dur: 0.8,
        gain: 0.24,
        attack: 0.003,
        hold: 0.4,
        filter: { f: 5000 },
      });
      tone(a, {
        type: "triangle",
        f: 880,
        dur: 0.8,
        gain: 0.1,
        attack: 0.003,
        hold: 0.4,
        filter: { f: 3000 },
      });
    }
  },
  /** Over the line: bells climbing to a held chord. */
  finish(a: Play) {
    for (const [i, n] of [79, 84, 88].entries()) {
      bell(a, { f: hz(n), at: i * 0.14, dur: 1.0, gain: 0.12, ratio: 2, index: 1.2 });
    }
    pad(a, {
      type: "triangle",
      notes: [67, 72, 76, 79].map(hz),
      at: 0.42,
      dur: 1.8,
      gain: 0.1,
      attack: 0.03,
      hold: 0.5,
      filter: { f: 2800, f1: 1000 },
    });
  },
  /** A menu or button: a soft tick. */
  click(a: Play) {
    tone(a, { f: 1800, f1: 1200, dur: 0.04, gain: 0.07, attack: 0.001 });
    noise(a, { dur: 0.02, gain: 0.03, attack: 0.001, filter: { type: "highpass", f: 3000 } });
  },
  /** A turn goes live: the pilot may go. */
  turnStart(a: Play) {
    for (const at of [0, 0.32]) {
      tone(a, {
        type: "triangle",
        f: hz(69),
        at,
        dur: 0.16,
        gain: 0.16,
        attack: 0.004,
        hold: 0.06,
        filter: { f: 2400 },
      });
    }
    tone(a, {
      type: "triangle",
      f: hz(81),
      at: 0.64,
      dur: 0.6,
      gain: 0.18,
      attack: 0.004,
      hold: 0.2,
      filter: { f: 3000 },
    });
  },
  /** A turn ends (time up, the aircraft spent) or is called off. */
  turnEnd(a: Play) {
    bell(a, { f: 110, dur: 2.4, gain: 0.16, ratio: 1.41, index: 3 });
    tone(a, { f: 55, dur: 1.6, gain: 0.12, attack: 0.01 });
  },
  /** Each of a turn's last 10 seconds; higher for the last 3. */
  tick(a: Play, urgent: boolean) {
    tone(a, { f: urgent ? 1320 : 880, dur: 0.1, gain: 0.22, attack: 0.004 });
  },
  /** The match is over and you won. */
  win(a: Play) {
    for (const [i, n] of [72, 76, 79].entries()) {
      tone(a, {
        type: "triangle",
        f: hz(n),
        at: i * 0.12,
        dur: 0.24,
        gain: 0.18,
        attack: 0.005,
        hold: 0.06,
        filter: { f: 3000 },
      });
    }
    pad(a, {
      type: "triangle",
      notes: [72, 76, 79, 84].map(hz),
      at: 0.38,
      dur: 1.4,
      gain: 0.12,
      attack: 0.02,
      hold: 0.4,
      filter: { f: 3200, f1: 1200 },
    });
  },
  /** The match is over and the other player won. A draw plays turnEnd. */
  lose(a: Play) {
    for (const [i, n] of [55, 54, 53].entries()) {
      tone(a, {
        type: "sawtooth",
        f: hz(n),
        at: i * 0.3,
        dur: 0.32,
        gain: 0.16,
        attack: 0.03,
        hold: 0.15,
        filter: { f: 900, q: 2 },
      });
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
  /** The other player comes into the room. */
  join(a: Play) {
    tone(a, { f: hz(79), dur: 0.2, gain: 0.16, attack: 0.004 });
    tone(a, { f: hz(86), at: 0.1, dur: 0.35, gain: 0.16, attack: 0.004 });
  },
  /** The other player leaves the room. */
  leave(a: Play) {
    tone(a, { f: hz(86), dur: 0.2, gain: 0.16, attack: 0.004 });
    tone(a, { f: hz(79), at: 0.1, dur: 0.35, gain: 0.16, attack: 0.004 });
  },
  /** The truck hits something hard; louder the harder (0..1). */
  bump(a: Play, intensity: number) {
    const k = 0.35 + 0.65 * clamp01(intensity);
    tone(a, { f: 95, f1: 42, dur: 0.28, gain: 0.7 * k, attack: 0.002 });
    noise(a, { dur: 0.12, gain: 0.35 * k, filter: { f: 700 } });
  },
  /** Driving in a duel turn: a drone near the truck. Higher the nearer (0..1). */
  proximity(a: Play, near: number) {
    const f = 1050 * (1 + 0.15 * clamp01(near));
    tone(a, { f, dur: 0.35, gain: 0.1, attack: 0.002 });
    tone(a, { f, at: 0.17, dur: 0.3, gain: 0.03, attack: 0.002 });
  },
  /** Driving: each second the truck is outside the fence, before it goes back. */
  fence(a: Play) {
    tone(a, {
      type: "square",
      f: 110,
      dur: 0.3,
      gain: 0.06,
      attack: 0.01,
      hold: 0.12,
      filter: { f: 500 },
    });
  },
};

type EffectId = keyof typeof EFFECTS;

/** Another vehicle, heard from where it is (`heard`). */
export interface Heard {
  /** Stable for as long as it is heard: its voice keeps its pitch. */
  id: string;
  kind: MotorKind;
  /** 0..1: its throttle or load. */
  level: number;
  /** 0..1: its motor or engine speed (levels.ts: rotorNorm, truckEngine). */
  rpmNorm: number;
  /** Each motor's speed 0..1, when known. */
  each?: readonly number[];
  /** Metres from the listener. */
  range: number;
  /** -1 (left) .. 1 (right) of the listener. */
  pan: number;
}

class Sounds {
  private ctx: AudioContext | null = null;
  private master: GainNode | null = null;
  private motorsBus: GainNode | null = null;
  private effectsBus: GainNode | null = null;
  private last = new Map<string, number>();
  private own: { kind: MotorKind; voice: Voice } | null = null;
  private others = new Map<string, { kind: MotorKind; voice: Voice }>();
  private motion = new Map<string, { range: number; at: number; rate: number }>();

  constructor() {
    if (typeof window === "undefined") return;
    const unlock = () => this.unlock();
    addEventListener("pointerdown", unlock, { capture: true });
    addEventListener("keydown", unlock, { capture: true });
    document.addEventListener("visibilitychange", () => {
      if (!this.ctx) return;
      if (document.hidden) void this.ctx.suspend();
      else void this.ctx.resume();
    });
    effect(() => this.applyLevels(prefs().audio));
  }

  /** Start audio: call on a user gesture (the page's first click or key does it too). */
  unlock(): void {
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
      this.master = ctx.createGain();
      this.master.gain.value = 0;
      this.master.connect(limiter);
      this.motorsBus = ctx.createGain();
      this.effectsBus = ctx.createGain();
      this.motorsBus.connect(this.master);
      this.effectsBus.connect(this.master);
      this.ctx = ctx;
      this.applyLevels(prefs().audio);
    }
    if (this.ctx.state === "suspended" && !document.hidden) void this.ctx.resume();
  }

  /** Whether sounds can play now: started, the page shown, sound on. */
  get live(): boolean {
    return this.ctx?.state === "running" && prefs().audio.on;
  }

  /** Sound on or off (prefs.audio.on). */
  toggle(): void {
    updatePrefs((p) => ({ ...p, audio: { ...p.audio, on: !p.audio.on } }));
  }

  /** The player's own motor or engine, every frame: `level` the throttle or load 0..1,
   * `rpmNorm` the motor or engine speed 0..1 (levels.ts: rotorNorm for a rotor's rad/s,
   * truckEngine for the truck); `each`, the quad's four motors' speeds, when known. */
  motor(kind: MotorKind, level: number, rpmNorm: number, each?: readonly number[]): void {
    if (!this.live) {
      this.stopMotor();
      return;
    }
    const set: VoiceSet = { level, rpm: clamp01(rpmNorm), each, gain: 1, pan: 0, doppler: 1 };
    if (this.own?.kind !== kind) {
      this.stopMotor();
      const voice = makeVoice(this.ctx!, this.motorsBus!, kind);
      voice.set(set, true);
      this.own = { kind, voice };
      return;
    }
    this.own.voice.set(set);
  }

  /** Silence the player's motor (out of the vehicle, crashed, paused). */
  stopMotor(): void {
    this.own?.voice.stop();
    this.own = null;
  }

  /** Every frame: the other vehicles there are to hear (the duel's truck to its pilot, the
   * drone to the driver, a race ghost). The nearest are heard, louder the nearer, their pitch
   * shifted by their speed towards the listener. An empty list silences them. */
  heard(list: readonly Heard[]): void {
    const now = performance.now();
    const rates = new Map(list.map((d) => [d.id, this.closing(d, now)]));
    for (const id of this.motion.keys()) if (!rates.has(id)) this.motion.delete(id);
    const heard = this.live
      ? list
          .filter((d) => d.range < HEAR_M && d.rpmNorm > 0.01)
          .sort((a, b) => a.range - b.range)
          .slice(0, VOICES)
      : [];
    const gainOf = (d: Heard) => (d.kind === "ground" ? truckGain(d.range) : droneGain(d.range));
    const crowd = 1 / Math.max(1, Math.hypot(...heard.map(gainOf))) ** CROWD;
    const want = new Map(heard.map((d) => [d.id, d]));
    for (const [id, v] of this.others) {
      if (want.get(id)?.kind === v.kind) continue;
      v.voice.stop();
      this.others.delete(id);
    }
    for (const d of heard) {
      const set: VoiceSet = {
        level: d.level,
        rpm: clamp01(d.rpmNorm),
        each: d.each,
        gain: gainOf(d) * crowd,
        pan: Math.max(-1, Math.min(1, d.pan)),
        doppler: doppler(rates.get(d.id) ?? 0),
      };
      const v = this.others.get(d.id);
      if (v) {
        v.voice.set(set);
      } else {
        const spread = (Math.random() * 2 - 1) * VOICE_SPREAD;
        const voice = makeVoice(this.ctx!, this.motorsBus!, d.kind, spread);
        voice.set(set, true);
        this.others.set(d.id, { kind: d.kind, voice });
      }
    }
  }

  /** `d`'s speed towards the listener, m/s, smoothed over the frames. */
  private closing(d: Heard, now: number): number {
    const m = this.motion.get(d.id);
    if (!m) {
      this.motion.set(d.id, { range: d.range, at: now, rate: 0 });
      return 0;
    }
    const dt = (now - m.at) / 1000;
    if (!(dt > 0)) return m.rate;
    const raw = (m.range - d.range) / dt;
    m.rate = Math.abs(raw) > JUMP_MS ? 0 : m.rate + (raw - m.rate) * Math.min(1, dt / 0.25);
    m.range = d.range;
    m.at = now;
    return m.rate;
  }

  strike(): void {
    this.play("strike");
  }
  score(): void {
    this.play("score");
  }
  crash(): void {
    this.play("crash");
  }
  gate(): void {
    this.play("gate");
  }
  lap(): void {
    this.play("lap");
  }
  /** 3, 2, 1: a beep each; 0: go. */
  countdown(n: number): void {
    this.play("countdown", n, `countdown${n}`);
  }
  finish(): void {
    this.play("finish");
  }
  click(): void {
    this.play("click");
  }
  turnStart(): void {
    this.play("turnStart");
  }
  turnEnd(): void {
    this.play("turnEnd");
  }
  tick(urgent = false): void {
    this.play("tick", urgent);
  }
  win(): void {
    this.play("win");
  }
  lose(): void {
    this.play("lose");
  }
  join(): void {
    this.play("join");
  }
  leave(): void {
    this.play("leave");
  }
  /** 0..1: how hard. */
  bump(intensity = 0.6): void {
    this.play("bump", intensity);
  }
  /** 0..1: how near. */
  proximity(near: number): void {
    this.play("proximity", near);
  }
  fence(): void {
    this.play("fence");
  }

  private play<K extends EffectId>(
    id: K,
    arg?: Parameters<(typeof EFFECTS)[K]>[1],
    gapKey: string = id,
  ): void {
    if (!this.live) return;
    const now = performance.now();
    if (now - (this.last.get(gapKey) ?? -Infinity) < (MIN_GAP_MS[id] ?? DEFAULT_GAP_MS)) return;
    this.last.set(gapKey, now);
    const a: Play = { ctx: this.ctx!, out: this.effectsBus!, t: this.ctx!.currentTime + 0.01 };
    (EFFECTS[id] as (a: Play, arg: unknown) => void)(a, arg);
  }

  private applyLevels(audio: {
    on: boolean;
    master: number;
    motors: number;
    effects: number;
  }): void {
    const ctx = this.ctx;
    if (!ctx || !this.master || !this.motorsBus || !this.effectsBus) return;
    const t = ctx.currentTime;
    this.master.gain.setTargetAtTime(audio.on ? clamp01(audio.master) * VOLUME : 0, t, 0.02);
    this.motorsBus.gain.setTargetAtTime(clamp01(audio.motors), t, 0.02);
    this.effectsBus.gain.setTargetAtTime(clamp01(audio.effects), t, 0.02);
  }
}

/** The page's sounds. */
export const sounds = new Sounds();
