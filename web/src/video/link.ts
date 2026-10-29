// The camera feed's geometry, apart from any drawing so it can be tested: the picture's frame in
// the window, and the video link's antennas (DESIGN.md §10).

/** A box on the page, px. */
export interface Box {
  left: number;
  top: number;
  width: number;
  height: number;
}

/** The largest `aspect` box that fits a `width` × `height` area, centred in it. */
export function fitFrame(width: number, height: number, aspect: number): Box {
  const w = Math.max(Math.min(width, height * aspect), 0);
  const h = w / aspect;
  return { left: (width - w) / 2, top: (height - h) / 2, width: w, height: h };
}

/** A half-wave dipole's gain, dBi, at `cosAxis`, the cosine of the angle from its axis: 2.15
 * broadside, a null along the axis (filled in to -20 dBi by reflections). */
export const NULL_FLOOR_DBI = -8;

export function dipoleGainDbi(cosAxis: number): number {
  const s = Math.sqrt(Math.max(1 - cosAxis * cosAxis, 1e-6));
  const g = (Math.cos((Math.PI / 2) * cosAxis) / s) ** 2 * 1.64;
  // A real dipole's null is never empty: the ground's reflection and the antenna's own
  // imperfection fill it to about -8 dBi, so a quad straight over the pilot still gets through.
  return Math.max(10 * Math.log10(g), NULL_FLOOR_DBI);
}

/** The link's geometry: `d`, the transmitter from the receiver (east, north, up, m), and `up`,
 * the transmitter's dipole axis (its frame's up, east-north-up). The range, and each end's gain:
 * the receiver's dipole stands straight up, the transmitter's leans with its frame, so flying
 * overhead or banked hard toward the pilot puts the picture in a null. */
export function linkGeometry(
  d: [number, number, number],
  up: [number, number, number],
): { range: number; txGainDbi: number; rxGainDbi: number } {
  const range = Math.max(Math.hypot(d[0], d[1], d[2]), 1);
  const txCos = -(up[0] * d[0] + up[1] * d[1] + up[2] * d[2]) / range;
  return { range, txGainDbi: dipoleGainDbi(txCos), rxGainDbi: dipoleGainDbi(d[2] / range) };
}

/** Slow fading on top of the link budget (bodies, the ground, the antennas' angles): a random
 * walk back toward 0 dB, its spread `spreadDb`, wandering over `tauS`. `gauss` a standard
 * normal draw. */
export function fadeStep(db: number, dtS: number, gauss: number, spreadDb = 2, tauS = 0.6): number {
  const dt = Math.min(Math.max(dtS, 0), 0.5);
  return db * (1 - dt / tauS) + spreadDb * Math.sqrt((2 * dt) / tauS) * gauss;
}
