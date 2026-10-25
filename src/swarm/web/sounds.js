// The game's sounds, made in the browser with Web Audio: no files to fetch. Whether sound is on
// is kept in localStorage, shared by the viewer's pages (the COP, the drive): a change on one
// reaches the others as it is made.
//
// Browsers start audio only after a click or a key on the page; until then nothing plays. A
// hidden page is silent, and drops what happens meanwhile rather than playing it on return.

const STORE_KEY = "drone-duel-sounds";
const VOLUME = 0.72;
/** A sound plays at most once this often: a burst of events is one sound. */
const MIN_GAP_MS = { strike: 60, crash: 80, bump: 220, tick: 250, proximity: 100, fence: 500 };
const DEFAULT_GAP_MS = 40;
/** The truck's top speed in each gear, m/s: the revs climb through a gear and drop at a change. */
const GEARS = [6, 13, 21, 34];
/** Every engine's level: it runs the whole drive, so it stays under the other sounds. */
const ENGINE_LEVEL = 0.5;
/** The drones' motors: the sim's 5 inch quad (sitl/fpv5.json), 2207 motors on 5.1 inch
 * tri-blades, about 30,000 rpm at full output. */
const MOTOR = { maxRevS: 500, blades: 3, polePairs: 7, thrustExpo: 0.7 };
/** Each motor a little off the others, cents: four never quite in tune, so they beat. */
const MOTOR_DETUNE = [-14, 9, -5, 17];
/** Motor speed (motorRevs) of an armed drone on the ground and hovering, for a report
 * without its motor outputs (an old log). */
const IDLE_REVS = 0.06;
const HOVER_REVS = 0.33;
/** A drone in FPV cruising is about as loud as the truck's engine to its driver, flat out
 * still under a strike. */
const DRONE_LEVEL = 0.22;
/** Each drone a little off the others, cents at most: drones together never lock in phase. */
const DRONE_SPREAD = 25;
/** A drone this near is heard at full loudness; twice as far, half as loud. */
const DRONE_FULL_M = 10;
/** The other player's truck, heard by the pilot, likewise. */
const TRUCK_FULL_M = 25;
/** Drones heard together are turned down by their summed loudness to this power: five as
 * near as one are about 2 dB louder than it, not 7. */
const DRONE_CROWD = 0.75;
/** The nearest drones are heard, this many at most, out to this far. */
const DRONE_VOICES = 6;
const DRONE_HEAR_M = 600;
const SOUND_MS = 343;
/** A drone's pitch shifts with its speed towards the listener, up to this fraction. */
const DOPPLER_MAX = 0.15;
/** A range changing faster than this, m/s, is a jump (another drone in the camera), not speed. */
const JUMP_MS = 120;

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

/** A chord of slightly detuned pairs, slow to rise. */
function pad(a, { notes, type = "sawtooth", at = 0, dur, gain, attack = 0.25, hold = 0.3, filter }) {
  for (const f of notes) {
    for (const detune of [-7, 7]) tone(a, { type, f, at, dur, gain: gain / 2, attack, hold, detune, filter });
  }
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

/** A quad's four motors: each a sawtooth at its blade rate, through a lowpass that opens as
 * they speed up, with the motor's own whine above it, and the prop wash as noise. */
function quadVoice(ctx, out, spread = 0) {
  const bus = ctx.createGain();
  bus.gain.value = 0;
  const panner = ctx.createStereoPanner();
  const level = ctx.createGain();
  level.gain.value = DRONE_LEVEL;
  bus.connect(panner).connect(level).connect(out);
  const filter = ctx.createBiquadFilter();
  filter.type = "lowpass";
  filter.Q.value = 0.9;
  filter.connect(bus);
  const motors = MOTOR_DETUNE.map((detune) => {
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
  const sources = [...motors.flatMap((m) => [m.blade, m.whine]), wash];
  startAll(sources);
  let lastMean = 0;
  return {
    /** `revs`: each motor 0..1 (motorRevs); `gain` for its distance, `pan` -1..1, and
     * `doppler`, the factor on its pitch. */
    set({ revs, gain, pan, doppler }, now = false) {
      const t = ctx.currentTime;
      const mean = revs.reduce((sum, r) => sum + r, 0) / revs.length;
      // Props spin up quicker than they run down.
      const k = now ? 0.005 : mean >= lastMean ? 0.04 : 0.15;
      lastMean = mean;
      for (const [i, m] of motors.entries()) {
        const r = revs[i] ?? mean;
        const revS = Math.max(1, MOTOR.maxRevS * r * doppler);
        m.blade.frequency.setTargetAtTime(revS * MOTOR.blades, t, k);
        m.whine.frequency.setTargetAtTime(revS * MOTOR.polePairs, t, k);
        // Quiet at idle, near full by hover, a little louder still flat out.
        m.bladeGain.gain.setTargetAtTime(0.16 * Math.min(1, 3 * r) ** 1.2 * (0.4 + r), t, k);
        m.whineGain.gain.setTargetAtTime(0.02 * r, t, k);
      }
      filter.frequency.setTargetAtTime((700 + 4200 * mean) * doppler, t, k);
      washFilter.frequency.setTargetAtTime((350 + 1600 * mean) * doppler, t, k);
      washGain.gain.setTargetAtTime(0.5 * mean * mean, t, k);
      bus.gain.setTargetAtTime(gain, t, 0.05);
      panner.pan.setTargetAtTime(pan, t, now ? 0.005 : 0.05);
    },
    stop: () => fadeOut(ctx, bus, sources),
  };
}

/** A motor's speed, 0 (stopped) to 1 (full), from its output in µs (1000 to 2000): its thrust
 * on ArduPilot's thrust curve (MOT_THST_EXPO), the speed that thrust's square root. */
export function motorRevs(us) {
  const out = clamp01((us - 1000) / 1000);
  const e = MOTOR.thrustExpo;
  return Math.sqrt((1 - e) * out + e * out * out);
}

/** Each of a drone's four motors' speed (motorRevs) from its report: its motor outputs or,
 * without them, an armed drone idling on the ground or hovering. */
export function droneRevs(motors, armed, flying) {
  if (motors?.length >= 4) return motors.slice(0, 4).map(motorRevs);
  const r = !armed ? 0 : flying ? HOVER_REVS : IDLE_REVS;
  return [r, r, r, r];
}

/** A drone's loudness `range` m from the listener: full up to DRONE_FULL_M, then falling as
 * 1 / distance. */
export function droneGain(range) {
  return Math.min(1, DRONE_FULL_M / Math.max(range, 1e-3));
}

/** The truck's loudness to the pilot, `range` m from the listener. */
export function truckGain(range) {
  return Math.min(1, TRUCK_FULL_M / Math.max(range, 1e-3));
}

/** The factor on a drone's pitch closing on the listener at `rate` m/s (opening: negative). */
export function doppler(rate) {
  const f = SOUND_MS / (SOUND_MS - rate);
  return Math.min(1 + DOPPLER_MAX, Math.max(1 - DOPPLER_MAX, f));
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
// Each plays into `a` (above); the truck's engine is dieselEngine.

const SOUNDS = {
  /** A drone strikes the truck. Both players hear it. */
  strike(a) {
    noise(a, { dur: 1.5, gain: 0.55, attack: 0.008, filter: { f: 1100, f1: 90 } });
    tone(a, { f: 72, f1: 32, dur: 1.1, gain: 0.55, attack: 0.005 });
  },
  /** Your drone struck the truck: +1. Only the pilot hears it, after the strike. */
  score(a) {
    tone(a, { type: "triangle", f: hz(83), dur: 0.09, gain: 0.22, attack: 0.002, hold: 0.05 });
    tone(a, { type: "triangle", f: hz(88), at: 0.08, dur: 0.45, gain: 0.22, attack: 0.002 });
  },
  /** A drone is out without striking: the ground, the scenery, or another drone. */
  crash(a) {
    noise(a, { dur: 0.7, gain: 0.45, attack: 0.03, filter: { f: 2400, f1: 250 } });
    tone(a, { f: 85, f1: 40, dur: 0.4, gain: 0.35, attack: 0.01 });
  },
  /** A turn goes live: the pilot's drones may go. */
  turnStart(a) {
    for (const at of [0, 0.32]) {
      tone(a, { type: "triangle", f: hz(69), at, dur: 0.16, gain: 0.16, attack: 0.004, hold: 0.06, filter: { f: 2400 } });
    }
    tone(a, { type: "triangle", f: hz(81), at: 0.64, dur: 0.6, gain: 0.18, attack: 0.004, hold: 0.2, filter: { f: 3000 } });
  },
  /** A turn ends (time up, every drone out) or is called off. */
  turnEnd(a) {
    bell(a, { f: 110, dur: 2.4, gain: 0.16, ratio: 1.41, index: 3 });
    tone(a, { f: 55, dur: 1.6, gain: 0.12, attack: 0.01 });
  },
  /** Each of a turn's last 10 seconds; higher for the last 3. */
  tick(a, { urgent } = {}) {
    tone(a, { f: urgent ? 1320 : 880, dur: 0.1, gain: 0.22, attack: 0.004 });
  },
  /** The match is over and you won. */
  win(a) {
    for (const [i, n] of [72, 76, 79].entries()) {
      tone(a, { type: "triangle", f: hz(n), at: i * 0.12, dur: 0.24, gain: 0.18, attack: 0.005, hold: 0.06, filter: { f: 3000 } });
    }
    pad(a, { type: "triangle", notes: [72, 76, 79, 84].map(hz), at: 0.38, dur: 1.4, gain: 0.12, attack: 0.02, hold: 0.4, filter: { f: 3200, f1: 1200 } });
  },
  /** The match is over and the other player won. A draw plays turnEnd. */
  lose(a) {
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
  /** The other player comes into the room. */
  join(a) {
    tone(a, { f: hz(79), dur: 0.2, gain: 0.16, attack: 0.004 });
    tone(a, { f: hz(86), at: 0.1, dur: 0.35, gain: 0.16, attack: 0.004 });
  },
  /** The other player leaves the room. */
  leave(a) {
    tone(a, { f: hz(86), dur: 0.2, gain: 0.16, attack: 0.004 });
    tone(a, { f: hz(79), at: 0.1, dur: 0.35, gain: 0.16, attack: 0.004 });
  },
  /** The truck hits something hard; louder the harder. */
  bump(a, { intensity } = {}) {
    const k = 0.35 + 0.65 * clamp01(intensity ?? 0.6);
    tone(a, { f: 95, f1: 42, dur: 0.28, gain: 0.7 * k, attack: 0.002 });
    noise(a, { dur: 0.12, gain: 0.35 * k, filter: { f: 700 } });
  },
  /** Driving in a turn: a drone near the truck. Quicker and higher the nearer it is. */
  proximity(a, { near } = {}) {
    const f = 1050 * (1 + 0.15 * clamp01(near));
    tone(a, { f, dur: 0.35, gain: 0.1, attack: 0.002 });
    tone(a, { f, at: 0.17, dur: 0.3, gain: 0.03, attack: 0.002 });
  },
  /** Driving: each second the truck is outside the fence, before it goes back. */
  fence(a) {
    tone(a, { type: "square", f: 110, dur: 0.3, gain: 0.06, attack: 0.01, hold: 0.12, filter: { f: 500 } });
  },
};

// -- the player ------------------------------------------------------------------

function loadEnabled() {
  try {
    return JSON.parse(localStorage.getItem(STORE_KEY) ?? "{}").enabled !== false;
  } catch {
    return true;
  }
}

class Sfx {
  constructor() {
    /** @type {AudioContext | null} made at the page's first click or key. */
    this.ctx = null;
    this._enabled = loadEnabled();
    this._last = new Map();
    this._listeners = new Set();
    const unlock = () => this._unlock();
    addEventListener("pointerdown", unlock, { capture: true });
    addEventListener("keydown", unlock, { capture: true });
    addEventListener("storage", (e) => {
      if (e.key !== STORE_KEY) return;
      this._enabled = loadEnabled();
      this._changed(false);
    });
    document.addEventListener("visibilitychange", () => {
      if (!this.ctx) return;
      if (document.hidden) this.ctx.suspend();
      else this.ctx.resume();
    });
  }

  get enabled() {
    return this._enabled;
  }

  setEnabled(on) {
    this._enabled = Boolean(on);
    this._changed();
  }

  toggle() {
    this.setEnabled(!this.enabled);
  }

  /** Call `fn` on every change, here or on another page; returns a function to stop. */
  onChange(fn) {
    this._listeners.add(fn);
    return () => this._listeners.delete(fn);
  }

  /** Play sound `id`, if sound is on. `opts` as the sound takes them. */
  play(id, opts = {}) {
    const play = SOUNDS[id];
    if (!play || !this.enabled || !this.live) return;
    const now = performance.now();
    if (now - (this._last.get(id) ?? -Infinity) < (MIN_GAP_MS[id] ?? DEFAULT_GAP_MS)) return;
    this._last.set(id, now);
    play({ ctx: this.ctx, out: this._out, t: this.ctx.currentTime + 0.01 }, opts);
  }

  /** The truck's engine, while sound is on; `spatial`: heard from where it is (update's
   * `where`), not from the driver's seat. */
  engine({ spatial = false } = {}) {
    return new EngineSound(this, spatial);
  }

  /** The drones' motors, each heard from where it is, while sound is on. */
  drones() {
    return new DroneSounds(this);
  }

  /** Whether sounds can play now: started by a click or key, the page shown. */
  get live() {
    return this.ctx?.state === "running";
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
      this._out = ctx.createGain();
      this._out.connect(limiter);
      this.ctx = ctx;
      this._applyLevel();
    }
    if (this.ctx.state === "suspended" && !document.hidden) this.ctx.resume();
  }

  _applyLevel() {
    if (this.ctx) this._out.gain.setTargetAtTime(this.enabled ? VOLUME : 0, this.ctx.currentTime, 0.02);
  }

  _changed(save = true) {
    if (save) {
      try {
        localStorage.setItem(STORE_KEY, JSON.stringify({ enabled: this._enabled }));
      } catch {
        // Storage blocked: on or off lasts for this page only.
      }
    }
    this._applyLevel();
    for (const fn of this._listeners) fn();
  }
}

/** The engine's voice, made when it can be heard and dropped when it cannot (sound off, the
 * truck off the map, the page hidden). */
class EngineSound {
  constructor(sfx, spatial = false) {
    this.sfx = sfx;
    this.spatial = spatial;
    this.voice = null;
    /** A spatial engine's {gain, panner}, between its voice and the page's output. */
    this._where = null;
    this.on = false;
    this.state = revs(0, 0);
    this.where = { gain: 1, pan: 0 };
    this._unsubscribe = sfx.onChange(() => this._sync());
  }

  /** Each frame: the truck's speed, m/s, and throttle -1..1; `on` false silences it. A spatial
   * engine's `where`: {gain, pan} for its distance and side. */
  update(speed, throttle, on = true, where = this.where) {
    this.on = on;
    this.state = revs(speed, throttle);
    this.where = where;
    this._sync();
    this.voice?.set(this.state);
    if (this._where) {
      const t = this.sfx.ctx.currentTime;
      this._where.gain.gain.setTargetAtTime(where.gain, t, 0.05);
      this._where.panner.pan.setTargetAtTime(where.pan, t, 0.05);
    }
  }

  stop() {
    this._unsubscribe();
    this.on = false;
    this._sync();
  }

  _sync() {
    const sfx = this.sfx;
    const want = this.on && sfx.live && sfx.enabled;
    if (want === Boolean(this.voice)) return;
    this.voice?.stop();
    this.voice = null;
    this._where = null;
    if (!want) return;
    let out = sfx._out;
    if (this.spatial) {
      const gain = sfx.ctx.createGain();
      gain.gain.value = this.where.gain;
      const panner = sfx.ctx.createStereoPanner();
      panner.pan.value = this.where.pan;
      gain.connect(panner).connect(out);
      this._where = { gain, panner };
      out = gain;
    }
    this.voice = dieselEngine(sfx.ctx, out);
    this.voice.set(this.state, true);
  }
}

/** The drones' voices: the nearest ones spinning, each made when it can be heard and dropped
 * when it cannot (sound off, out of hearing, landed and disarmed, gone from the map). */
class DroneSounds {
  constructor(sfx) {
    this.sfx = sfx;
    /** id -> quadVoice, of those heard now. */
    this.voices = new Map();
    /** id -> {range, at, rate}: how fast each drone closes on the listener, m/s. */
    this._motion = new Map();
    this.heard = [];
    /** The factor on every drone's gain for how many are heard (DRONE_CROWD). */
    this._crowd = 1;
    this._unsubscribe = sfx.onChange(() => this._sync());
  }

  /** Each frame: every drone there is to hear, [{id, revs (droneRevs), range: metres from
   * the listener, pan: -1 (left) to 1}]. */
  update(drones) {
    const now = performance.now();
    for (const d of drones) d.rate = this._closing(d, now);
    for (const id of this._motion.keys()) {
      if (!drones.some((d) => d.id === id)) this._motion.delete(id);
    }
    this.heard = drones
      .filter((d) => d.range < DRONE_HEAR_M && d.revs.some((r) => r > 0.01))
      .sort((a, b) => a.range - b.range)
      .slice(0, DRONE_VOICES);
    const total = Math.hypot(...this.heard.map((d) => droneGain(d.range)));
    this._crowd = 1 / Math.max(1, total) ** DRONE_CROWD;
    this._sync();
    for (const d of this.heard) this.voices.get(d.id)?.set(this._sound(d));
  }

  stop() {
    this._unsubscribe();
    this.heard = [];
    this._sync();
  }

  _sound(d) {
    return { revs: d.revs, gain: droneGain(d.range) * this._crowd, pan: d.pan, doppler: doppler(d.rate) };
  }

  /** Drone `d`'s speed towards the listener, m/s, smoothed over the frames. */
  _closing(d, now) {
    const m = this._motion.get(d.id);
    this._motion.set(d.id, { range: d.range, at: now, rate: m?.rate ?? 0 });
    const dt = m ? (now - m.at) / 1000 : 0;
    if (!(dt > 0)) return m?.rate ?? 0;
    const raw = (m.range - d.range) / dt;
    const rate = Math.abs(raw) > JUMP_MS ? 0 : m.rate + (raw - m.rate) * Math.min(1, dt / 0.25);
    this._motion.get(d.id).rate = rate;
    return rate;
  }

  _sync() {
    const sfx = this.sfx;
    const want = sfx.live && sfx.enabled ? new Map(this.heard.map((d) => [d.id, d])) : new Map();
    for (const [id, voice] of this.voices) {
      if (want.has(id)) continue;
      voice.stop();
      this.voices.delete(id);
    }
    for (const [id, d] of want) {
      if (this.voices.has(id)) continue;
      const voice = quadVoice(sfx.ctx, sfx._out, (Math.random() * 2 - 1) * DRONE_SPREAD);
      voice.set(this._sound(d), true);
      this.voices.set(id, voice);
    }
  }
}

/** The page's sounds. */
export const sfx = new Sfx();
