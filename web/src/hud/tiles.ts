// The minimap's map maths (hud/minimap.ts), apart from the drawing so it can be tested: Web
// Mercator pixels, the zoom that fits a span, and the tiles a view needs.

/** A map tile's side, px. */
export const TILE = 256;
/** Metres a pixel at zoom 0 on the equator (2π × 6378137 / 256). */
export const M_PER_PX_Z0 = 156543.03392804097;
export const MAX_ZOOM = 19;

export const ESRI_TILE_URL =
  "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}";

export function tileUrl(z: number, x: number, y: number): string {
  return ESRI_TILE_URL.replace("{z}", String(z))
    .replace("{y}", String(y))
    .replace("{x}", String(x));
}

/** Web Mercator pixel coordinates of (lat, lon) at zoom `z`: x east, y south, from the
 * north-west corner of the world. */
export function worldPx(lat: number, lon: number, z: number): [number, number] {
  const size = TILE * 2 ** z;
  const phi = (Math.max(Math.min(lat, 85.05112878), -85.05112878) * Math.PI) / 180;
  const x = ((lon + 180) / 360) * size;
  const y = (0.5 - Math.log((1 + Math.sin(phi)) / (1 - Math.sin(phi))) / (4 * Math.PI)) * size;
  return [x, y];
}

/** Ground metres per map pixel at `lat`, zoom `z`. */
export function metresPerPx(lat: number, z: number): number {
  return (M_PER_PX_Z0 * Math.cos((lat * Math.PI) / 180)) / 2 ** z;
}

/** The least whole zoom whose pixels are no larger than a device pixel when `metresAcross`
 * spans `cssPx` CSS px at `dpr` device px each: sharp, and no more tiles than that needs. */
export function zoomFor(
  lat: number,
  metresAcross: number,
  cssPx: number,
  dpr = 1,
  maxZoom = MAX_ZOOM,
): number {
  const mPerDevicePx = metresAcross / (cssPx * dpr);
  const z = Math.ceil(Math.log2((M_PER_PX_Z0 * Math.cos((lat * Math.PI) / 180)) / mPerDevicePx));
  return Math.max(0, Math.min(maxZoom, z));
}

/** A tile to draw: its zoom and indices (x wrapped round the world), and where its north-west
 * corner falls, in map px at its zoom, from the view's centre. */
export interface TileAt {
  z: number;
  x: number;
  y: number;
  dx: number;
  dy: number;
}

/** The tiles under a view centred on map px (`cx`, `cy`) at zoom `z`, `halfW` × `halfH` map px
 * either side of it. */
export function visibleTiles(
  cx: number,
  cy: number,
  halfW: number,
  halfH: number,
  z: number,
): TileAt[] {
  const n = 2 ** z;
  const out: TileAt[] = [];
  const x0 = Math.floor((cx - halfW) / TILE);
  const x1 = Math.floor((cx + halfW) / TILE);
  const y0 = Math.max(0, Math.floor((cy - halfH) / TILE));
  const y1 = Math.min(n - 1, Math.floor((cy + halfH) / TILE));
  for (let ty = y0; ty <= y1; ty++) {
    for (let tx = x0; tx <= x1; tx++) {
      out.push({ z, x: ((tx % n) + n) % n, y: ty, dx: tx * TILE - cx, dy: ty * TILE - cy });
    }
  }
  return out;
}

/** The view's projection: (lat, lon) to CSS px from its centre, north up, `scale` CSS px per
 * map px at zoom `z`. */
export function projector(
  centreLat: number,
  centreLon: number,
  z: number,
  scale: number,
): (lat: number, lon: number) => [number, number] {
  const [cx, cy] = worldPx(centreLat, centreLon, z);
  const world = TILE * 2 ** z;
  return (lat, lon) => {
    const [x, y] = worldPx(lat, lon, z);
    // The short way round, across the antimeridian.
    let dx = x - cx;
    if (dx > world / 2) dx -= world;
    else if (dx < -world / 2) dx += world;
    return [dx * scale, (y - cy) * scale];
  };
}

/** A point `x`, `y` from the centre pulled in to the view's edge, `inset` px inside it: a round
 * view of radius `r`, or a square of half-side `r`. Unchanged when inside. */
export function clampToEdge(
  x: number,
  y: number,
  r: number,
  round: boolean,
  inset = 0,
): { x: number; y: number; clamped: boolean } {
  const lim = r - inset;
  if (round) {
    const d = Math.hypot(x, y);
    if (d <= lim) return { x, y, clamped: false };
    return { x: (x / d) * lim, y: (y / d) * lim, clamped: true };
  }
  const m = Math.max(Math.abs(x), Math.abs(y));
  if (m <= lim) return { x, y, clamped: false };
  return { x: (x / m) * lim, y: (y / m) * lim, clamped: true };
}

/** A round scale-bar length, m, no longer than `maxM`: 1, 2 or 5 times a power of ten. */
export function niceLength(maxM: number): number {
  if (!(maxM > 0)) return 0;
  const p = 10 ** Math.floor(Math.log10(maxM));
  for (const k of [5, 2, 1]) if (k * p <= maxM) return k * p;
  return p;
}
