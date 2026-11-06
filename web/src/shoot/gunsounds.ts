// The rifle's sounds (DESIGN.md §7, Shoot Down), made with Web Audio as the game's others are
// (audio/sounds.ts). The shooter's own shot: the muzzle's crack, the blast behind it, a thump
// felt as much as heard, the ground's reflection a few milliseconds on, the bolt cycling, the
// shot rolling back off the country round, and the brass landing. What happens out there is
// heard when its sound gets back: the range over the speed of sound later, quieter the farther.

import { noise, type Play, sounds, tone } from "../audio/sounds.ts";

/** Loudness with range: spherical spreading from `ref` m, and the air taking the highs (a
 * lowpass that closes with range, roughly ISO 9613's absorption). */
function far(range: number, ref = 10): { gain: number; lowpass: number } {
  return {
    gain: Math.min(1, ref / Math.max(ref, range)),
    lowpass: Math.max(1500, 12000 - 25 * range),
  };
}

/** The player's own shot, `crack` 0..1 how sharp (an 11.5" barrel: loud, full). */
function shot(a: Play): void {
  // The crack: a sharp broadband edge.
  noise(a, { dur: 0.012, gain: 1.0, attack: 0.0005, filter: { type: "highpass", f: 1800 } });
  // The blast: the gas, its pitch falling as it spreads.
  noise(a, { dur: 0.22, gain: 0.95, attack: 0.001, filter: { f: 7000, f1: 380 } });
  // The thump.
  tone(a, { f: 95, f1: 38, dur: 0.18, gain: 0.9, attack: 0.001 });
  // The ground's reflection, 4 ms on.
  noise(a, { at: 0.004, dur: 0.12, gain: 0.45, attack: 0.001, filter: { f: 4000, f1: 300 } });
  // The bolt: a clack and a ring of steel.
  noise(a, {
    at: 0.012,
    dur: 0.035,
    gain: 0.25,
    attack: 0.001,
    filter: { type: "bandpass", f: 3200, q: 4 },
  });
  tone(a, { type: "triangle", f: 2650, at: 0.014, dur: 0.06, gain: 0.05, attack: 0.001 });
  // The echo off the country round: a low roll a while later.
  noise(a, { at: 0.25, dur: 1.4, gain: 0.12, attack: 0.15, filter: { f: 900, f1: 180 } });
  noise(a, { at: 0.6, dur: 1.2, gain: 0.06, attack: 0.2, filter: { f: 600, f1: 150 } });
  // The case lands a metre or so away and bounces.
  const land = 0.45 + Math.random() * 0.2;
  for (let i = 0; i < 3; i++) {
    tone(a, {
      type: "sine",
      f: 5200 + Math.random() * 900,
      at: land + i * (0.07 + Math.random() * 0.05),
      dur: 0.08,
      gain: 0.025 / (i + 1),
      attack: 0.001,
    });
  }
}

/** The trigger on an empty chamber. */
function dry(a: Play): void {
  noise(a, { dur: 0.02, gain: 0.25, attack: 0.001, filter: { type: "bandpass", f: 2400, q: 3 } });
}

function magOut(a: Play): void {
  noise(a, { dur: 0.03, gain: 0.25, attack: 0.001, filter: { type: "bandpass", f: 1800, q: 2 } });
  noise(a, { at: 0.06, dur: 0.12, gain: 0.08, attack: 0.01, filter: { f: 1200 } });
}

function magIn(a: Play): void {
  noise(a, { dur: 0.05, gain: 0.18, attack: 0.002, filter: { f: 2200 } });
  noise(a, {
    at: 0.06,
    dur: 0.03,
    gain: 0.35,
    attack: 0.001,
    filter: { type: "bandpass", f: 2600, q: 3 },
  });
}

/** The bolt released onto a fresh round. */
function bolt(a: Play): void {
  noise(a, { dur: 0.04, gain: 0.4, attack: 0.001, filter: { type: "bandpass", f: 2900, q: 3 } });
  tone(a, { type: "triangle", f: 2400, at: 0.002, dur: 0.12, gain: 0.04, attack: 0.001 });
}

export const gunSounds = {
  shot: () => sounds.custom(shot),
  dry: () => sounds.custom(dry),
  magOut: () => sounds.custom(magOut),
  magIn: () => sounds.custom(magIn),
  bolt: () => sounds.custom(bolt),
  /** A bullet striking the drone `range` m away, `pan` its side: plastic and carbon breaking,
   * heard when the sound gets back. */
  hit(range: number, pan: number, heavy: boolean): void {
    const f = far(range);
    sounds.custom(
      (a) => {
        noise(a, { dur: 0.05, gain: 0.9 * f.gain, attack: 0.001, filter: { f: f.lowpass } });
        tone(a, {
          type: "square",
          f: heavy ? 420 : 900,
          dur: 0.06,
          gain: 0.08 * f.gain,
          attack: 0.001,
          filter: { f: f.lowpass },
        });
      },
      range / speedOfSound,
      pan,
    );
  },
  /** A miss landing in the ground `range` m away. */
  thud(range: number, pan: number): void {
    const f = far(range, 6);
    sounds.custom(
      (a) =>
        noise(a, {
          dur: 0.09,
          gain: 0.5 * f.gain,
          attack: 0.002,
          filter: { f: Math.min(900, f.lowpass) },
        }),
      range / speedOfSound,
      pan,
    );
  },
};

/** m/s; the shoot mode sets it from the place's air. */
let speedOfSound = 340;
export function setSpeedOfSound(c: number): void {
  speedOfSound = c;
}
