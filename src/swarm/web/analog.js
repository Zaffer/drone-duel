// The camera feed as analog goggles show it: the picture sent as NTSC composite video over a
// 5.8 GHz FM link and decoded again, signal by signal, on the GPU. A port of analog-vision
// (github.com/Synetic-Labs/Analog-Vision), whose modules each stage names; read those for
// the physics. Nothing is painted on: grain, colour noise, sparklies, torn lines, the colour
// killer, rolling and snow all come out of the chain, at the link's carrier-to-noise ratio.
//
// The signal is held as rows of one line each, in rings of a frame (525 lines): composite at
// 4fsc (910 samples a line), the radio at 8fsc (1820). Each animation frame encodes the lines
// due since the last (15 734 a second), from the latest picture, and runs them through:
//
//   encode (ntsc, encode)   picture and OSD to composite: sync, burst, band-limited Y'UV
//   tx, if, disc (fm)       frequency modulation, thermal noise, IF filter, discriminator
//   video (fm)              the receiver's video filter and clipping
//   filt ... summ (decode)  the decoder's streams: sync, luma, chroma, vertical integrator,
//                           sync edges and level measurements at every sample
//   lines (decode)          the decoder itself, line by line: flywheel, vertical sync, clamp,
//                           AGC, burst and colour killer. Sequential, so each of its fragments
//                           runs it from the last frame's state to its own line.
//   picture (decode)        each decoded line into the goggles' picture, Y'UV
//   display (pipeline)      the latest whole field, line-doubled (bob), as R'G'B'
//
// The OSD goes in as the flight controller's OSD chip puts it: its pixels switch the video
// to white or black, after the camera and before the transmitter, so the link spoils it too.

/** Colour subcarrier, Hz. */
const FSC = 315e6 / 88;
const FS = 4 * FSC;
const FS_RF = 2 * FS;
const LINE = 910;
const LINES = 525;
const LINE_RATE = FS / LINE;
const ACTIVE = 754;
const ACTIVE_LINES = 480;
/** Lines the decoder gets through in one frame at most: the lines pass's fragments. */
const MAX_LINES = 320;
/** Lines encoded in one frame at most: after a stall, the rest is skipped. */
const MAX_BATCH = 300;
const BLOCK = 14;
const KAISER_BETA = 6;

// -- link budget (fm.link_cnr_db) -----------------------------------------------------------

/** The link's defaults: a typical 5.8 GHz analog FPV pair (RTC6705 transmitter, RTC6715
 * receiver). */
const LINK = {
  deviationHz: 5e6,
  ifBandwidthHz: 18e6,
  videoBandwidthHz: 5e6,
  frequencyHz: 5.8e9,
  noiseFigureDb: 6,
};

/** CNR at the receiver, dB, over free space: received power over thermal noise (-174 dBm/Hz)
 * in the IF bandwidth, raised by the noise figure. `txGainDbi` and `rxGainDbi` the antennas'
 * gains toward each other. The RTC6715's -85 dBm sensitivity is about 10 dB. */
export function linkCnrDb(distanceM, txMw, txGainDbi = 2, rxGainDbi = 2) {
  const fspl = 20 * Math.log10((4 * Math.PI * Math.max(distanceM, 0.1) * LINK.frequencyHz) / 299_792_458);
  const rx = 10 * Math.log10(txMw) + txGainDbi + rxGainDbi - fspl;
  const noise = -174 + 10 * Math.log10(LINK.ifBandwidthHz) + LINK.noiseFigureDb;
  return rx - noise;
}

/** A half-wave dipole's gain, dBi, at `cosAxis`, the cosine of the angle from its axis: 2.15
 * broadside, a null along the axis (filled in to -20 dBi by reflections). */
export function dipoleGainDbi(cosAxis) {
  const s = Math.sqrt(Math.max(1 - cosAxis * cosAxis, 1e-6));
  const g = (Math.cos((Math.PI / 2) * cosAxis) / s) ** 2 * 1.64;
  return Math.max(10 * Math.log10(g), -20);
}

// -- filters (dsp) --------------------------------------------------------------------------

function besselI0(x) {
  let sum = 1;
  let term = 1;
  for (let k = 1; k < 50; k++) {
    term *= (x / (2 * k)) ** 2;
    sum += term;
  }
  return sum;
}

function kaiser(n) {
  return Array.from({ length: n }, (_, i) => {
    const r = (2 * i) / (n - 1) - 1;
    return besselI0(KAISER_BETA * Math.sqrt(1 - r * r)) / besselI0(KAISER_BETA);
  });
}

const sinc = (x) => (x === 0 ? 1 : Math.sin(Math.PI * x) / (Math.PI * x));

/** scipy.signal.firwin(taps, cutoff, fs=fs, window=("kaiser", 6)): unity at DC. */
function lowpass(taps, cutoffHz, fs) {
  const c = cutoffHz / (fs / 2);
  const w = kaiser(taps);
  const mid = (taps - 1) / 2;
  const h = w.map((wi, i) => c * sinc(c * (i - mid)) * wi);
  const sum = h.reduce((a, b) => a + b, 0);
  return h.map((x) => x / sum);
}

/** firwin(taps, [low, high], pass_zero=False): unity at the band's centre. */
function bandpass(taps, lowHz, highHz, fs) {
  const c1 = lowHz / (fs / 2);
  const c2 = highHz / (fs / 2);
  const w = kaiser(taps);
  const mid = (taps - 1) / 2;
  const h = w.map((wi, i) => (c2 * sinc(c2 * (i - mid)) - c1 * sinc(c1 * (i - mid))) * wi);
  const centre = (c1 + c2) / 2;
  const gain = h.reduce((a, x, i) => a + x * Math.cos(Math.PI * centre * (i - mid)), 0);
  return h.map((x) => x / gain);
}

/** A raised-cosine pulse of `taps`, summing to 1 (dsp.smoothing). */
function smoothing(taps) {
  const m = taps + 2;
  const w = Array.from({ length: m }, (_, i) => 0.5 - 0.5 * Math.cos((2 * Math.PI * i) / (m - 1))).slice(1, -1);
  const sum = w.reduce((a, b) => a + b, 0);
  return w.map((x) => x / sum);
}

function convolve(a, b) {
  const out = new Array(a.length + b.length - 1).fill(0);
  a.forEach((x, i) => b.forEach((y, j) => (out[i + j] += x * y)));
  return out;
}

/** The decoder's filters, run one after another, as single kernels on its input (decode.py's
 * delays: sync 20, bp 20, luma 15, demod 30, burst demod 20). */
function decoderKernels() {
  const bp = bandpass(41, FSC - 1.3e6, FSC + 1.3e6, FS);
  // Luma: the input delayed to match the band-pass, less it (a notch), low-passed.
  const notch = bp.map((x, i) => (i === 20 ? 1 : 0) - x);
  const luma = convolve(lowpass(31, 4.2e6, FS), notch);
  // Chroma: band-pass, to baseband against the free-running subcarrier, low-passed. Mixing by
  // 2(-i)^n between the two moves to the front as 2(-i)^n and i^j on the low-pass's taps.
  const chroma = (lp) => {
    const re = new Array(lp.length + bp.length - 1).fill(0);
    const im = new Array(re.length).fill(0);
    lp.forEach((l, j) => {
      const [cr, ci] = [
        [1, 0],
        [0, 1],
        [-1, 0],
        [0, -1],
      ][j % 4];
      bp.forEach((b, i) => {
        re[i + j] += l * b * cr;
        im[i + j] += l * b * ci;
      });
    });
    return [re, im];
  };
  const [zr, zi] = chroma(lowpass(61, 0.6e6, FS));
  const [br, bi] = chroma(lowpass(41, 1.3e6, FS));
  return { sync: lowpass(41, 1.5e6, FS), luma, zr, zi, br, bi };
}

// -- shaders --------------------------------------------------------------------------------

const HEADER = `#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
const int LINE = 910;
const int LINES = 525;
const int FRAME = 477750;
const int RLINE = 1820;
const int RFRAME = 955500;
const int BLOCK = ${BLOCK};
const int BLOCKS = FRAME / BLOCK;
const int BLOCKS_LINE = LINE / BLOCK;
const float PI = 3.14159265358979;
const float SYNC = -40.0;
const float SETUP = 7.5;
const float SPAN = 92.5;
const float BURST = 20.0;
const float MID = 30.0;
const float SLICE = -20.0;
const int HSYNC = 67;
const int EQUALIZING = 33;
const int BROAD = 388;
uniform sampler2D uTaps;
uniform int uL0;
int wrapN(int n) { n %= FRAME; return n < 0 ? n + FRAME : n; }
ivec2 at(int n) { n = wrapN(n); return ivec2(n % LINE, n / LINE); }
ivec2 atRF(int m) { m %= RFRAME; if (m < 0) m += RFRAME; return ivec2(m % RLINE, m / RLINE); }
float tap(int row, int j) { return texelFetch(uTaps, ivec2(j, row), 0).r; }
int ringRow() { return int(gl_FragCoord.y); }
/** The line this row holds now, counted from the start. */
int absLine() { int r0 = uL0 % LINES; return uL0 + (ringRow() - r0 + LINES) % LINES; }
int here() { return ringRow() * LINE + int(gl_FragCoord.x); }
int hereRF() { return ringRow() * RLINE + int(gl_FragCoord.x); }
vec2 cmul(vec2 a, vec2 b) { return vec2(a.x * b.x - a.y * b.y, a.x * b.y + a.y * b.x); }
vec2 cis(float a) { return vec2(cos(a), sin(a)); }
/** (-i)^k */
vec2 negI(int k) { k &= 3; return k == 0 ? vec2(1, 0) : k == 1 ? vec2(0, -1) : k == 2 ? vec2(-1, 0) : vec2(0, 1); }
/** The sync pulse starting half line h (0 to 1049) of the frame (ntsc.half_line). */
int pulse(int h) {
  for (int s = 0; s < 2; s++) {
    int k = h - s * 525;
    if ((k >= 0 && k < 6) || (k >= 12 && k < 18)) return EQUALIZING;
    if (k >= 6 && k < 12) return BROAD;
  }
  return h % 2 == 0 ? HSYNC : 0;
}
/** Line 1 to 525 starts with an ordinary sync, and so carries a burst. */
bool hasBurst(int line) { return pulse(2 * (line - 1)) == HSYNC; }
`;

const QUAD_VS = `#version 300 es
void main() {
  int v = gl_VertexID;
  gl_Position = vec4(v == 1 || v == 4 || v == 5 ? 1.0 : -1.0, v == 2 || v == 3 || v == 5 ? 1.0 : -1.0, 0.0, 1.0);
}`;

const TAP = { encY: 0, encC: 1, up: 2, if: 3, video: 4, sync: 5, luma: 6, zr: 7, zi: 8, br: 9, bi: 10, burst: 11 };

/** The camera's picture, scaled to the active lines: 754 x 480, top row first. */
const RASTER_FS = `${HEADER}
uniform sampler2D uCam;
out vec4 o;
void main() {
  vec2 size = vec2(${ACTIVE}.0, ${ACTIVE_LINES}.0);
  vec2 px = 1.0 / size;
  vec2 uv = gl_FragCoord.xy * px;
  // 2 x 2 bilinear taps over the pixel: the sensor's own averaging, for a downscale under 2.
  vec3 c = vec3(0);
  for (int i = 0; i < 4; i++) {
    vec2 d = (vec2(i & 1, i >> 1) - 0.5) * 0.5 * px;
    c += texture(uCam, uv + d).rgb;
  }
  o = vec4(c / 4.0, 1);
}`;

/** encode.py: one sample of composite, IRE. */
const ENCODE_FS = `${HEADER}
uniform sampler2D uRaster;
uniform sampler2D uOsd;
uniform ivec3 uOsdSize; // width, height, top line in the field; width 0: no OSD
out float o;
float syncAt(int n) {
  n = wrapN(n);
  int c = n % LINE;
  int half_ = c >= 455 ? 1 : 0;
  return c - 455 * half_ < pulse(2 * (n / LINE) + half_) ? SYNC : 0.0;
}
void main() {
  int c = int(gl_FragCoord.x);
  int line = ringRow() + 1;
  int n = here();
  float x = 0.25 * syncAt(n - 1) + 0.5 * syncAt(n) + 0.25 * syncAt(n + 1);
  int k = ((absLine() & 1) * 2 + c) & 3;
  float s = k == 1 ? 1.0 : k == 3 ? -1.0 : 0.0;
  float co = k == 0 ? 1.0 : k == 2 ? -1.0 : 0.0;
  int j = c - 76;
  if (hasBurst(line) && j >= 0 && j < 36) x -= BURST * tap(${TAP.burst}, j) * s;
  int f = -1;
  int row = -1;
  if (line >= 23 && line < 263) { f = line - 23; row = 2 * f + 1; }
  else if (line >= 286) { f = line - 286; row = 2 * f; }
  int a = c - 135;
  if (row >= 0 && a >= 0 && a < ${ACTIVE}) {
    float y = 0.0;
    vec3 rgb = vec3(0);
    for (int i = -20; i <= 20; i++) {
      vec3 p = texelFetch(uRaster, ivec2(clamp(a + i, 0, ${ACTIVE - 1}), row), 0).rgb;
      if (i >= -15 && i <= 15) y += tap(${TAP.encY}, i + 15) * dot(p, vec3(0.299, 0.587, 0.114));
      rgb += tap(${TAP.encC}, i + 20) * p;
    }
    float yc = dot(rgb, vec3(0.299, 0.587, 0.114));
    float u = 0.492111 * (rgb.b - yc);
    float v = 0.877283 * (rgb.r - yc);
    float video = SETUP + SPAN * (y + u * s + v * co);
    int oy = f - uOsdSize.z;
    if (uOsdSize.x > 0 && oy >= 0 && oy < uOsdSize.y) {
      // The OSD chip's pixels, white or black, in place of the video; edges as the chip's.
      float add = 0.0;
      for (int i = -1; i <= 1; i++) {
        int ox = clamp((a + i) * uOsdSize.x / ${ACTIVE}, 0, uOsdSize.x - 1);
        vec4 g = texelFetch(uOsd, ivec2(ox, oy), 0);
        if (g.a > 0.5) add += (i == 0 ? 0.5 : 0.25) * (SETUP + SPAN * g.r - video);
      }
      video += add;
    }
    float env = a == 0 || a == ${ACTIVE - 1} ? 0.25 : a == 1 || a == ${ACTIVE - 2} ? 0.75 : 1.0;
    x += video * env;
  }
  o = x;
}`;

/** fm.py, the transmitter and the air: up to 8fsc, the frequency (radians a sample), and the
 * received signal against the transmitted carrier: 1 plus the noise. */
const TX_FS = `${HEADER}
uniform sampler2D uComp;
uniform float uRadPerIre;
uniform float uSigma;
uniform uint uSeed;
layout(location = 0) out float oFreq;
layout(location = 1) out vec2 oRx;
uvec3 pcg3d(uvec3 v) {
  v = v * 1664525u + 1013904223u;
  v.x += v.y * v.z; v.y += v.z * v.x; v.z += v.x * v.y;
  v ^= v >> 16u;
  v.x += v.y * v.z; v.y += v.z * v.x; v.z += v.x * v.y;
  return v;
}
void main() {
  int m = hereRF();
  float video = 0.0;
  for (int j = m & 1; j < 63; j += 2) {
    int q = m - j;
    if (q < 0) q += RFRAME;
    video += tap(${TAP.up}, j) * texelFetch(uComp, at(q / 2), 0).r;
  }
  oFreq = (video - MID) * uRadPerIre;
  // The noise, white, circular: turned to the carrier's phase it is still white.
  uvec3 h = pcg3d(uvec3(uint(absLine()), uint(gl_FragCoord.x), uSeed));
  float u1 = (float(h.x >> 8u) + 0.5) / 16777216.0;
  float u2 = float(h.y >> 8u) / 16777216.0;
  oRx = vec2(1, 0) + uSigma * sqrt(-2.0 * log(u1)) * cis(2.0 * PI * u2);
}`;

/** fm.py, the receiver's IF filter, against the carrier's phase at its centre tap: only the
 * phase's local changes are needed, so no running phase is kept. */
const IF_FS = `${HEADER}
uniform sampler2D uFreq;
uniform sampler2D uRx;
out vec2 o;
void main() {
  int k = hereRF();
  vec2 b = tap(${TAP.if}, 31) * texelFetch(uRx, atRF(k - 31), 0).rg;
  float psi = 0.0;
  for (int j = 30; j >= 0; j--) {
    psi += texelFetch(uFreq, atRF(k - j), 0).r;
    b += tap(${TAP.if}, j) * cmul(cis(psi), texelFetch(uRx, atRF(k - j), 0).rg);
  }
  psi = 0.0;
  for (int j = 32; j < 63; j++) {
    psi -= texelFetch(uFreq, atRF(k - j + 1), 0).r;
    b += tap(${TAP.if}, j) * cmul(cis(psi), texelFetch(uRx, atRF(k - j), 0).rg);
  }
  o = b;
}`;

/** fm.py, the discriminator: the phase step from each sample to the next, as video, IRE. */
const DISC_FS = `${HEADER}
uniform sampler2D uIf;
uniform sampler2D uFreq;
uniform float uRadPerIre;
out float o;
void main() {
  int k = hereRF();
  vec2 b = texelFetch(uIf, atRF(k), 0).rg;
  vec2 a = texelFetch(uIf, atRF(k - 1), 0).rg;
  vec2 step_ = cmul(cmul(b, vec2(a.x, -a.y)), cis(texelFetch(uFreq, atRF(k - 31), 0).r));
  o = atan(step_.y, step_.x) / uRadPerIre + MID;
}`;

/** fm.py, the receiver's video amplifier: filtered, back to 4fsc, clipped. */
const VIDEO_FS = `${HEADER}
uniform sampler2D uDisc;
out float o;
void main() {
  int m = 2 * here();
  float v = 0.0;
  for (int j = 0; j < 63; j++) v += tap(${TAP.video}, j) * texelFetch(uDisc, atRF(m - j), 0).r;
  o = clamp(v, -60.0, 140.0);
}`;

/** decode.py's streams from the video: sync (low-passed), luma (notch), chroma and burst
 * chroma at baseband. */
const FILT_FS = `${HEADER}
uniform sampler2D uVideo;
layout(location = 0) out vec2 oSyncY;
layout(location = 1) out vec4 oChroma;
void main() {
  int n = here();
  float sync = 0.0, y = 0.0;
  vec2 z = vec2(0), zb = vec2(0);
  for (int m = 0; m < 101; m++) {
    float x = texelFetch(uVideo, at(n - m), 0).r;
    if (m < 41) sync += tap(${TAP.sync}, m) * x;
    if (m < 71) y += tap(${TAP.luma}, m) * x;
    z += x * vec2(tap(${TAP.zr}, m), tap(${TAP.zi}, m));
    if (m < 81) zb += x * vec2(tap(${TAP.br}, m), tap(${TAP.bi}, m));
  }
  vec2 turn = 2.0 * negI((absLine() & 1) * 2 + int(gl_FragCoord.x) + 400 - 20);
  oSyncY = vec2(sync, y);
  oChroma = vec4(cmul(turn, z), cmul(turn, zb));
}`;

/** The vertical integrator (351 samples), in two steps: 27, then 13 of those. */
const SLICE_FS = `${HEADER}
uniform sampler2D uSyncY;
out float o;
void main() {
  int n = here();
  float a = 0.0;
  for (int j = 0; j < 27; j++) a += texelFetch(uSyncY, at(n - j), 0).r < SLICE ? 1.0 : 0.0;
  o = a;
}`;

const VINT_FS = `${HEADER}
uniform sampler2D uSlice;
out float o;
void main() {
  int n = here();
  float a = 0.0;
  for (int q = 0; q < 13; q++) a += texelFetch(uSlice, at(n - 27 * q), 0).r;
  o = a / 351.0;
}`;

/** At every sample, what the decoder would measure from there: a sync edge (decode
 * _crossings: falling through the slice, and staying down 2.5 us; where between this sample
 * and the next, or -1), and the means for the back porch, sync tip and burst. */
const MEAS_FS = `${HEADER}
uniform sampler2D uSyncY;
uniform sampler2D uChroma;
layout(location = 0) out float oEdge;
layout(location = 1) out vec4 oMeas;
void main() {
  int n = here();
  float s0 = texelFetch(uSyncY, at(n), 0).r;
  float s1 = texelFetch(uSyncY, at(n + 1), 0).r;
  float edge = -1.0;
  if (s0 >= SLICE && s1 < SLICE) {
    float down = 0.0;
    for (int i = 6; i < 41; i++) down += texelFetch(uSyncY, at(n + i), 0).r;
    if (down / 35.0 < SLICE) edge = (s0 - SLICE) / (s0 - s1);
  }
  oEdge = edge;
  float porch = 0.0, tip = 0.0;
  for (int i = 0; i < 51; i++) {
    float y = texelFetch(uSyncY, at(n + i), 0).g;
    tip += y;
    if (i < 15) porch += y;
  }
  vec2 burst = vec2(0);
  for (int i = 0; i < 20; i++) burst += texelFetch(uChroma, at(n + i), 0).ba;
  oMeas = vec4(porch / 15.0, tip / 51.0, burst / 20.0);
}`;

/** Each block of samples: its first and last sync edge (offset in the block, with the
 * fraction) and the first sample past the vertical integrator's threshold; -1 for none. */
const SUMM_FS = `${HEADER}
uniform sampler2D uEdge;
uniform sampler2D uV;
out vec4 o;
void main() {
  int n0 = ringRow() * LINE + int(gl_FragCoord.x) * BLOCK;
  vec4 s = vec4(-1);
  for (int i = 0; i < BLOCK; i++) {
    float e = texelFetch(uEdge, at(n0 + i), 0).r;
    if (e >= 0.0) {
      if (s.x < 0.0) s.x = float(i) + e;
      s.y = float(i) + e;
    }
    if (s.z < 0.0 && texelFetch(uV, at(n0 + i), 0).r > 0.6) s.z = float(i);
  }
  o = s;
}`;

/** decode.VideoDecoder, line by line. State, as each line is drawn:
 *   s0: where its sync is (whole sample in the ring, fraction), line period, lines missed
 *   s1: horizontal lock, line number, lines since the vertical retrace, field (0 none, 1, 2)
 *   s2: clamp, AGC (-1 none yet), colour reference (the burst, complex)
 *   s3: lock detector, burst noise, colour flags (on, ok, good), 1 if valid */
const LINES_FS = `${HEADER}
uniform sampler2D uS0, uS1, uS2, uS3;
uniform sampler2D uEdge, uSumm, uMeas, uV;
uniform int uEnd;
layout(location = 0) out vec4 o0;
layout(location = 1) out vec4 o1;
layout(location = 2) out vec4 o2;
layout(location = 3) out vec4 o3;
const int DELAY_SYNC = 20, DELAY_Y = 35, DELAY_ZB = 40, DELAY_V = 195;
const int MARGIN = LINE + LINE + DELAY_V + 2;
const int H_WINDOW = 40, SEARCH_AFTER = 8, V_ACCEPT = 230, V_FREE = 272;

int pI; float pF; float period; int missed;
float hLock; int lineNo; int since; int parity;
float clampL; float agc; vec2 burst;
float lock; float burstNoise; bool colour, colourOk, colourGood;

vec4 summ(int blk) { blk %= BLOCKS; if (blk < 0) blk += BLOCKS; return texelFetch(uSumm, ivec2(blk % BLOCKS_LINE, blk / BLOCKS_LINE), 0); }
vec4 measAt(int i, float f) { return mix(texelFetch(uMeas, at(i), 0), texelFetch(uMeas, at(i + 1), 0), f); }
void advance(float d) { float t = pF + d; float fl = floor(t); pI = wrapN(pI + int(fl)); pF = t - fl; }

/** The edge nearest to p + period within the window (decode _next_sync), as an offset from
 * it in samples; or 1e9 for none. Local coordinates: 0 is the expected sync's sample in the
 * edge stream; candidates from -w to w - 1. */
float nearestEdge(int base, float ef, int w) {
  int bs = -(base % BLOCK);
  int blk = (base - base % BLOCK) / BLOCK;
  float best = 1e9;
  for (int i = 0; i < BLOCK; i++) {
    int s = bs + i;
    if (s < -w || s >= w) continue;
    float e = texelFetch(uEdge, at(base + s), 0).r;
    if (e >= 0.0 && abs(float(s) + e - ef) < abs(best)) best = float(s) + e - ef;
  }
  for (int q = 1; q <= 80; q++) {
    int start = bs + BLOCK * q;
    if (start >= w || float(start) - ef >= abs(best)) break;
    float first = summ(blk + q).x;
    if (first >= 0.0) {
      if (start + int(first) < w && float(start) + first - ef < abs(best)) best = float(start) + first - ef;
      break;
    }
  }
  for (int q = 1; q <= 80; q++) {
    int start = bs - BLOCK * q;
    if (start + BLOCK <= -w || ef - float(start + BLOCK) >= abs(best)) break;
    float last = summ(blk - q).y;
    if (last >= 0.0) {
      if (start + int(last) >= -w && ef - float(start) - last < abs(best)) best = float(start) + last - ef;
      break;
    }
  }
  return best;
}

void nextSync() {
  float expect = period;
  int w = missed >= SEARCH_AFTER ? 455 + H_WINDOW : H_WINDOW;
  // The expected sync, in the edge stream.
  float t = pF + expect;
  int base = wrapN(pI + int(floor(t)) + DELAY_SYNC);
  float ef = fract(t);
  float err = nearestEdge(base, ef, w);
  if (err > 1e8) {
    missed++;
    advance(expect);
    return;
  }
  missed = 0;
  if (w > H_WINDOW) { advance(expect + err); return; }
  period = clamp(period + 0.01 * err, 890.0, 930.0);
  advance(expect + 0.3 * err);
}

/** The first sample from p, along a line, past the vertical integrator's threshold; -1. */
int vsyncHit() {
  int base = wrapN(pI + DELAY_V);
  int bs = -(base % BLOCK);
  int blk = (base - base % BLOCK) / BLOCK;
  for (int i = 0; i < BLOCK + bs; i++)
    if (texelFetch(uV, at(base + i), 0).r > 0.6) return i;
  for (int q = 1; bs + BLOCK * q < LINE; q++) {
    float first = summ(blk + q).z;
    if (first >= 0.0) return bs + BLOCK * q + int(first);
  }
  return -1;
}

void retrace(int p) {
  colour = colourGood || (colour && colourOk);
  parity = p;
  lineNo = p == 1 ? 4 : 266;
  since = 0;
}

void decodeLine() {
  if (since >= V_ACCEPT) {
    int hit = vsyncHit();
    if (hit >= 0) retrace(hit < 455 ? 1 : 2);
    else if (since >= V_FREE) retrace(parity == 1 ? 2 : 1);
  }
  int frameLine = ((lineNo - 1) % LINES + LINES) % LINES + 1;
  bool ordinary = hasBurst(frameLine);
  if (ordinary && missed == 0) {
    float blank = measAt(pI + DELAY_Y + 116, pF).x;
    float tip = measAt(pI + DELAY_Y + 8, pF).y;
    clampL += 0.5 * (blank - clampL);
    float gain = clamp(40.0 / max(blank - tip, 1.0), 0.25, 4.0);
    agc = agc < 0.0 ? gain : agc + 0.05 * (gain - agc);
  }
  float g = agc < 0.0 ? 1.0 : agc;
  if (ordinary) {
    vec2 b = measAt(pI + DELAY_ZB + 84, pF).zw;
    vec2 ref = burst / max(length(burst), 1e-9);
    vec2 step_ = cmul(b, vec2(ref.x, -ref.y));
    lock += 0.02 * (step_.x - lock);
    burstNoise += 0.02 * (step_.y * step_.y - burstNoise);
    burst += 0.1 * (b - burst);
  }
  hLock += 0.05 * ((missed == 0 ? 1.0 : 0.0) - hLock);
  float strength = hLock > 0.5 ? lock * g / BURST : 0.0;
  float snr = lock / sqrt(max(burstNoise, 1e-9));
  colourOk = strength > 0.25 && snr > 2.2;
  colourGood = strength > 0.35 && snr > 2.5;
}

void main() {
  int slot = int(gl_FragCoord.x);
  vec4 s0 = texelFetch(uS0, ivec2(0), 0), s1 = texelFetch(uS1, ivec2(0), 0);
  vec4 s2 = texelFetch(uS2, ivec2(0), 0), s3 = texelFetch(uS3, ivec2(0), 0);
  pI = int(s0.x); pF = s0.y; period = s0.z; missed = int(s0.w);
  hLock = s1.x; lineNo = int(s1.y); since = int(s1.z); parity = int(s1.w);
  clampL = s2.x; agc = s2.y; burst = s2.zw;
  lock = s3.x; burstNoise = s3.y;
  int flags = int(s3.z);
  colour = (flags & 1) != 0; colourOk = (flags & 2) != 0; colourGood = (flags & 4) != 0;
  bool started = s3.w > 0.5;
  bool valid = false;
  for (int k = 0; k < ${MAX_LINES}; k++) {
    if (k > slot) break;
    if (started) {
      lineNo++;
      since++;
      nextSync();
    } else {
      // Acquire: the first edge in a line of signal already here.
      int start = wrapN(uEnd - MARGIN - 2 * LINE);
      pI = start; pF = 0.0;
      int blk = start / BLOCK;
      for (int q = 0; q < BLOCKS_LINE; q++) {
        float first = summ(blk + q).x;
        if (first >= 0.0) { pI = wrapN((blk + q) * BLOCK + int(first) - DELAY_SYNC); pF = fract(first); break; }
      }
    }
    if (wrapN(uEnd - pI) <= MARGIN) { valid = false; break; }
    decodeLine();
    started = true;
    valid = true;
  }
  int fl = (colour ? 1 : 0) + (colourOk ? 2 : 0) + (colourGood ? 4 : 0);
  o0 = vec4(float(pI), pF, period, float(min(missed, 100000)));
  o1 = vec4(hLock, float(lineNo), float(since), float(parity));
  o2 = vec4(clampL, agc, burst);
  o3 = vec4(lock, burstNoise, float(fl), valid ? 1.0 : 0.0);
}`;

/** The decoder's state after this frame: the last line it got to; or as it was. */
const CARRY_FS = `${HEADER}
uniform sampler2D uL0t, uL1t, uL2t, uL3t;
uniform sampler2D uS0, uS1, uS2, uS3;
layout(location = 0) out vec4 o0;
layout(location = 1) out vec4 o1;
layout(location = 2) out vec4 o2;
layout(location = 3) out vec4 o3;
void main() {
  for (int i = ${MAX_LINES - 1}; i >= 0; i--) {
    if (texelFetch(uL3t, ivec2(i, 0), 0).w > 0.5) {
      o0 = texelFetch(uL0t, ivec2(i, 0), 0); o1 = texelFetch(uL1t, ivec2(i, 0), 0);
      o2 = texelFetch(uL2t, ivec2(i, 0), 0); o3 = texelFetch(uL3t, ivec2(i, 0), 0);
      return;
    }
  }
  o0 = texelFetch(uS0, ivec2(0), 0); o1 = texelFetch(uS1, ivec2(0), 0);
  o2 = texelFetch(uS2, ivec2(0), 0); o3 = texelFetch(uS3, ivec2(0), 0);
}`;

/** Each decoded line that falls in the picture, as one row of the goggles' frame. */
const PICTURE_VS = `#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
uniform sampler2D uL0t, uL1t, uL2t, uL3t;
flat out int vP;
flat out float vF, vClamp, vAgc;
flat out vec2 vChroma;
void main() {
  int i = gl_InstanceID;
  vec4 s0 = texelFetch(uL0t, ivec2(i, 0), 0), s1 = texelFetch(uL1t, ivec2(i, 0), 0);
  vec4 s2 = texelFetch(uL2t, ivec2(i, 0), 0), s3 = texelFetch(uL3t, ivec2(i, 0), 0);
  int parity = int(s1.w);
  int row = int(s1.y) - (parity == 1 ? 23 : 286);
  gl_Position = vec4(2.0, 2.0, 0.0, 1.0);
  if (s3.w < 0.5 || parity == 0 || row < 0 || row >= 240) return;
  float frameRow = float(2 * row + (parity == 2 ? 0 : 1));
  int v = gl_VertexID;
  float x = v == 1 || v == 2 || v == 4 ? 1.0 : -1.0;
  float y = v == 2 || v == 4 || v == 5 ? 1.0 : 0.0;
  gl_Position = vec4(x, (frameRow + y) / 240.0 - 1.0, 0.0, 1.0);
  vP = int(s0.x);
  vF = s0.y;
  vClamp = s2.x;
  vAgc = s2.y < 0.0 ? 1.0 : s2.y;
  vChroma = vec2(0);
  if ((int(s3.z) & 1) != 0) {
    // Colour: turned so the burst is at 180 degrees on U, its gain set by the burst (ACC).
    vec2 b = s2.zw;
    float acc = min(20.0 / max(length(b), 1e-6), 4.0);
    float a = 3.14159265358979 / 2.0 - atan(b.y, b.x);
    vChroma = vec2(cos(a), sin(a)) * acc / 92.5;
  }
}`;

const PICTURE_FS = `${HEADER}
uniform sampler2D uSyncY;
uniform sampler2D uChroma;
flat in int vP;
flat in float vF, vClamp, vAgc;
flat in vec2 vChroma;
out vec4 o;
void main() {
  int t = vP + 135 + int(gl_FragCoord.x);
  float y = mix(texelFetch(uSyncY, at(t + 35), 0).g, texelFetch(uSyncY, at(t + 36), 0).g, vF);
  vec2 z = mix(texelFetch(uChroma, at(t + 50), 0).rg, texelFetch(uChroma, at(t + 51), 0).rg, vF);
  vec2 c = cmul(z, vChroma);
  o = vec4(((y - vClamp) * vAgc - SETUP) / SPAN, -c.y, c.x, 1);
}`;

/** The latest whole field, line-doubled to the screen, as R'G'B'. */
const DISPLAY_FS = `#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
uniform sampler2D uScreen;
uniform sampler2D uS1;
uniform vec2 uSize;
out vec4 o;
void main() {
  int drawing = int(texelFetch(uS1, ivec2(0), 0).w);
  if (drawing == 0) { o = vec4(0, 0, 0, 1); return; }
  int parity = drawing == 1 ? 2 : 1;
  float off = parity == 2 ? 0.0 : 1.0;
  vec2 uv = gl_FragCoord.xy / uSize;
  float frameRow = (1.0 - uv.y) * ${ACTIVE_LINES}.0 - 0.5;
  float f = clamp((frameRow - off) / 2.0, 0.0, 239.0);
  float f0 = floor(f);
  float f1 = min(f0 + 1.0, 239.0);
  vec3 a = texture(uScreen, vec2(uv.x, (2.0 * f0 + off + 0.5) / ${ACTIVE_LINES}.0)).rgb;
  vec3 b = texture(uScreen, vec2(uv.x, (2.0 * f1 + off + 0.5) / ${ACTIVE_LINES}.0)).rgb;
  vec3 yuv = mix(a, b, f - f0);
  float r = yuv.x + yuv.z / 0.877283;
  float bl = yuv.x + yuv.y / 0.492111;
  float g = (yuv.x - 0.299 * r - 0.114 * bl) / 0.587;
  o = vec4(clamp(vec3(r, g, bl), 0.0, 1.0), 1);
}`;

// -- the chain ------------------------------------------------------------------------------

export class AnalogFeed {
  /** @param {HTMLCanvasElement} canvas where the goggles' picture goes */
  constructor(canvas) {
    this.canvas = canvas;
    const gl = canvas.getContext("webgl2", { alpha: false, antialias: false, depth: false, stencil: false });
    this.gl = gl;
    /** False where WebGL2 with float render targets is missing: no analog feed. */
    this.ok = Boolean(gl?.getExtension("EXT_color_buffer_float"));
    if (!this.ok) return;
    /** Carrier to noise, dB. */
    this.cnrDb = 30;
    this.seed = (Math.random() * 2 ** 32) >>> 0;
    try {
      this._programs();
      this._textures();
    } catch (err) {
      console.error("analog feed off:", err);
      this.ok = false;
      return;
    }
    this.reset();
  }

  /** Start the signal again: the decoder has to find it. */
  reset() {
    const gl = this.gl;
    this._line = 0;
    this._carry = 0;
    this._last = null;
    for (const fb of this._clearable) {
      gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
      gl.clearColor(0, 0, 0, 0);
      gl.clear(gl.COLOR_BUFFER_BIT);
    }
    // The decoder's state before it has seen anything (decode.VideoDecoder.__init__).
    const initial = [
      [0, 0, LINE, 0],
      [1, 0, 230, 0],
      [0, -1, 0, 20],
      [20, 0, 7, 0],
    ];
    for (const st of this.state) {
      initial.forEach((v, i) => {
        gl.bindTexture(gl.TEXTURE_2D, st.tex[i]);
        gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, 1, 1, gl.RGBA, gl.FLOAT, new Float32Array(v));
      });
    }
  }

  _programs() {
    const gl = this.gl;
    const compile = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s) ?? "shader");
      return s;
    };
    const program = (fs, vs = QUAD_VS) => {
      const p = gl.createProgram();
      gl.attachShader(p, compile(gl.VERTEX_SHADER, vs));
      gl.attachShader(p, compile(gl.FRAGMENT_SHADER, fs));
      gl.linkProgram(p);
      if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p) ?? "program");
      const uniforms = {};
      const n = gl.getProgramParameter(p, gl.ACTIVE_UNIFORMS);
      for (let i = 0; i < n; i++) {
        const name = gl.getActiveUniform(p, i).name;
        uniforms[name] = gl.getUniformLocation(p, name);
      }
      return { p, u: uniforms };
    };
    this.prog = {
      raster: program(RASTER_FS),
      encode: program(ENCODE_FS),
      tx: program(TX_FS),
      if: program(IF_FS),
      disc: program(DISC_FS),
      video: program(VIDEO_FS),
      filt: program(FILT_FS),
      slice: program(SLICE_FS),
      vint: program(VINT_FS),
      meas: program(MEAS_FS),
      summ: program(SUMM_FS),
      lines: program(LINES_FS),
      carry: program(CARRY_FS),
      picture: program(PICTURE_FS, PICTURE_VS),
      display: program(DISPLAY_FS),
    };
  }

  _textures() {
    const gl = this.gl;
    const formats = {
      r: [gl.R32F, gl.RED],
      rg: [gl.RG32F, gl.RG],
      rgba: [gl.RGBA32F, gl.RGBA],
      half: [gl.RGBA16F, gl.RGBA],
      rgba8: [gl.RGBA8, gl.RGBA],
    };
    const texture = (w, h, format, linear = false) => {
      const t = gl.createTexture();
      gl.bindTexture(gl.TEXTURE_2D, t);
      const [internal] = formats[format];
      gl.texStorage2D(gl.TEXTURE_2D, 1, internal, w, h);
      const filter = linear ? gl.LINEAR : gl.NEAREST;
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, filter);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, filter);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      return t;
    };
    this._clearable = [];
    const target = (w, h, ...formatList) => {
      const tex = formatList.map((f) => texture(w, h, f, f === "half" || f === "rgba8"));
      const fb = gl.createFramebuffer();
      gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
      tex.forEach((t, i) => gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + i, gl.TEXTURE_2D, t, 0));
      gl.drawBuffers(tex.map((_, i) => gl.COLOR_ATTACHMENT0 + i));
      this._clearable.push(fb);
      return { fb, tex, w, h };
    };
    const RL = 2 * LINE;
    this.t = {
      raster: target(ACTIVE, ACTIVE_LINES, "rgba8"),
      comp: target(LINE, LINES, "r"),
      tx: target(RL, LINES, "r", "rg"),
      if: target(RL, LINES, "rg"),
      disc: target(RL, LINES, "r"),
      video: target(LINE, LINES, "r"),
      filt: target(LINE, LINES, "rg", "rgba"),
      slice: target(LINE, LINES, "r"),
      vint: target(LINE, LINES, "r"),
      meas: target(LINE, LINES, "r", "rgba"),
      summ: target(LINE / BLOCK, LINES, "rgba"),
      lines: target(MAX_LINES, 1, "rgba", "rgba", "rgba", "rgba"),
      screen: target(ACTIVE, ACTIVE_LINES, "half"),
    };
    this.state = [target(1, 1, "rgba", "rgba", "rgba", "rgba"), target(1, 1, "rgba", "rgba", "rgba", "rgba")];
    this._clearable = this._clearable.filter((fb) => !this.state.some((s) => s.fb === fb));
    this.cam = null;
    this._camSize = [0, 0];
    this.osd = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.osd);
    for (const [k, v] of [
      [gl.TEXTURE_MIN_FILTER, gl.NEAREST],
      [gl.TEXTURE_MAG_FILTER, gl.NEAREST],
      [gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE],
      [gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE],
    ])
      gl.texParameteri(gl.TEXTURE_2D, k, v);
    this._osdSize = [0, 0];
    // Every filter's taps, a row each (TAP).
    const dec = decoderKernels();
    const burstEnvelope = convolve(new Array(32).fill(1), smoothing(5));
    const rows = [];
    rows[TAP.encY] = lowpass(31, 4.2e6, FS);
    rows[TAP.encC] = lowpass(41, 1.3e6, FS);
    rows[TAP.up] = lowpass(63, 5.5e6, FS_RF).map((x) => 2 * x);
    rows[TAP.if] = lowpass(63, LINK.ifBandwidthHz / 2, FS_RF);
    rows[TAP.video] = lowpass(63, LINK.videoBandwidthHz, FS_RF);
    rows[TAP.sync] = dec.sync;
    rows[TAP.luma] = dec.luma;
    rows[TAP.zr] = dec.zr;
    rows[TAP.zi] = dec.zi;
    rows[TAP.br] = dec.br;
    rows[TAP.bi] = dec.bi;
    rows[TAP.burst] = burstEnvelope;
    const width = 128;
    const data = new Float32Array(width * rows.length);
    rows.forEach((r, i) => data.set(r, i * width));
    this.taps = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.taps);
    gl.texStorage2D(gl.TEXTURE_2D, 1, gl.R32F, width, rows.length);
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, width, rows.length, gl.RED, gl.FLOAT, data);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
  }

  /**
   * One animation frame: the lines due since the last, from `source` (the camera's picture,
   * a canvas or image, 4:3), with `osd` (a canvas of the OSD chip's pixels, one per field
   * line, OsdRaster; or null) put in as the chip does; then the picture to the canvas.
   * @param {TexImageSource} source
   * @param {{canvas: HTMLCanvasElement, top: number, changed: boolean} | null} osd
   * @param {number} now performance.now()
   */
  frame(source, osd, now) {
    if (!this.ok) return;
    const gl = this.gl;
    const due = this._last === null ? 263 : ((now - this._last) / 1000) * LINE_RATE + this._carry;
    this._last = now;
    const n = Math.min(Math.floor(due), MAX_BATCH);
    this._carry = Math.min(due - Math.floor(due), 1);
    gl.disable(gl.BLEND);
    gl.disable(gl.DEPTH_TEST);
    if (n > 0) {
      this._camera(source);
      this._osd(osd);
      this._run(n, osd);
    }
    this._display();
  }

  _camera(source) {
    const gl = this.gl;
    const w = source.width;
    const h = source.height;
    if (!this.cam || w !== this._camSize[0] || h !== this._camSize[1]) {
      if (this.cam) gl.deleteTexture(this.cam);
      this.cam = gl.createTexture();
      gl.bindTexture(gl.TEXTURE_2D, this.cam);
      gl.texStorage2D(gl.TEXTURE_2D, 1, gl.RGBA8, w, h);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      this._camSize = [w, h];
    }
    gl.bindTexture(gl.TEXTURE_2D, this.cam);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, gl.RGBA, gl.UNSIGNED_BYTE, source);
    this._pass("raster", this.t.raster, { uCam: this.cam });
  }

  _osd(osd) {
    if (!osd?.changed) return;
    const gl = this.gl;
    gl.bindTexture(gl.TEXTURE_2D, this.osd);
    gl.pixelStorei(gl.UNPACK_FLIP_Y_WEBGL, false);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, gl.RGBA, gl.UNSIGNED_BYTE, osd.canvas);
    this._osdSize = [osd.canvas.width, osd.canvas.height];
    osd.changed = false;
  }

  _run(n, osd) {
    const t = this.t;
    const start = this._line;
    this._line += n;
    const end = this._line;
    const radPerIre = ((LINK.deviationHz / 140) * 2 * Math.PI) / FS_RF;
    const n0 = 1 / (10 ** (this.cnrDb / 10) * LINK.ifBandwidthHz);
    const sigma = Math.sqrt((n0 * FS_RF) / 2);
    // The streams that look ahead are redone a line back, where they lacked the new lines.
    const back = Math.max(start - 1, 0);
    const osdSize = osd ? [this._osdSize[0], this._osdSize[1], osd.top] : [0, 0, 0];
    this._rows("encode", t.comp, start, end, { uRaster: t.raster.tex[0], uOsd: this.osd, uOsdSize: osdSize });
    this._rows("tx", t.tx, start, end, { uComp: t.comp.tex[0], uRadPerIre: radPerIre, uSigma: sigma, uSeed: this.seed });
    this._rows("if", t.if, start, end, { uFreq: t.tx.tex[0], uRx: t.tx.tex[1] });
    this._rows("disc", t.disc, start, end, { uIf: t.if.tex[0], uFreq: t.tx.tex[0], uRadPerIre: radPerIre });
    this._rows("video", t.video, start, end, { uDisc: t.disc.tex[0] });
    this._rows("filt", t.filt, start, end, { uVideo: t.video.tex[0] });
    this._rows("slice", t.slice, start, end, { uSyncY: t.filt.tex[0] });
    this._rows("vint", t.vint, start, end, { uSlice: t.slice.tex[0] });
    this._rows("meas", t.meas, back, end, { uSyncY: t.filt.tex[0], uChroma: t.filt.tex[1] });
    this._rows("summ", t.summ, back, end, { uEdge: t.meas.tex[0], uV: t.vint.tex[0] });
    // The decoder, from its state after the last frame, then that state carried on.
    const [from, to] = this.state;
    const s = { uS0: from.tex[0], uS1: from.tex[1], uS2: from.tex[2], uS3: from.tex[3] };
    this._pass("lines", t.lines, {
      ...s,
      uEdge: t.meas.tex[0],
      uSumm: t.summ.tex[0],
      uMeas: t.meas.tex[1],
      uV: t.vint.tex[0],
      uEnd: (end % LINES) * LINE,
    });
    const lines = { uL0t: t.lines.tex[0], uL1t: t.lines.tex[1], uL2t: t.lines.tex[2], uL3t: t.lines.tex[3] };
    this._pass("carry", to, { ...lines, ...s });
    this.state = [to, from];
    const gl = this.gl;
    this._bind("picture", t.screen, { ...lines, uSyncY: t.filt.tex[0], uChroma: t.filt.tex[1] });
    gl.viewport(0, 0, ACTIVE, ACTIVE_LINES);
    gl.drawArraysInstanced(gl.TRIANGLES, 0, 6, MAX_LINES);
  }

  _display() {
    const gl = this.gl;
    const c = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(c.clientWidth * dpr));
    const h = Math.max(1, Math.round(c.clientHeight * dpr));
    if (c.width !== w || c.height !== h) {
      c.width = w;
      c.height = h;
    }
    this._bind("display", null, { uScreen: this.t.screen.tex[0], uS1: this.state[0].tex[1], uSize: [w, h] });
    gl.viewport(0, 0, w, h);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
  }

  /** Program `name` drawing to `target` (null: the canvas), with `uniforms`: textures bound
   * in turn, numbers and arrays by the uniform's type. */
  _bind(name, target, uniforms) {
    const gl = this.gl;
    const { p, u } = this.prog[name];
    gl.useProgram(p);
    gl.bindFramebuffer(gl.FRAMEBUFFER, target ? target.fb : null);
    let unit = 0;
    const all = { uTaps: this.taps, ...uniforms };
    for (const [key, value] of Object.entries(all)) {
      const loc = u[key];
      if (loc === undefined) continue;
      if (value instanceof WebGLTexture) {
        gl.activeTexture(gl.TEXTURE0 + unit);
        gl.bindTexture(gl.TEXTURE_2D, value);
        gl.uniform1i(loc, unit++);
      } else if (key === "uSeed") gl.uniform1ui(loc, value);
      else if (Array.isArray(value) && value.length === 3) gl.uniform3iv(loc, value);
      else if (Array.isArray(value)) gl.uniform2fv(loc, value);
      else if (key === "uEnd" || key === "uL0") gl.uniform1i(loc, value);
      else gl.uniform1f(loc, value);
    }
    return u;
  }

  _pass(name, target, uniforms) {
    const gl = this.gl;
    this._bind(name, target, uniforms);
    gl.viewport(0, 0, target.w, target.h);
    gl.drawArrays(gl.TRIANGLES, 0, 6);
  }

  /** Program `name` over the ring's rows for lines `from` to `to`, in at most two pieces. */
  _rows(name, target, from, to, uniforms) {
    const gl = this.gl;
    const u = this._bind(name, target, uniforms);
    for (let l = from; l < to; ) {
      const r0 = l % LINES;
      const count = Math.min(to - l, LINES - r0);
      gl.uniform1i(u.uL0, l);
      gl.viewport(0, r0, target.w, count);
      gl.drawArrays(gl.TRIANGLES, 0, 6);
      l += count;
    }
  }
}
