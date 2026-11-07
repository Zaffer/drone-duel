// The ground on the free map (engine/ground.ts). Without Google's 3D tiles the map is the globe:
// Esri's terrain (or Cesium ion's). Its tiles are big for colliders (Esri's are 257 × 257
// heights, 131 072 triangles: some 40 ms to read and up to 630 ms for Rapier to take a few in
// at once, measured 2026-11-07), so the ground round each focus is sampled instead, PATCH_M square
// patches on a CELL_M grid, from the very heights the globe draws: each drawn terrain tile's
// own data, interpolated (a microsecond a point; the globe's getHeight walks a tile's
// triangles, milliseconds). A patch is built from the finest drawn tile under it, and again
// when a finer one is drawn there.

import type * as C from "cesium";
import { EARTH_R, moveBy, type Vec3 } from "../world/geo.ts";
import type { Builder, Piece, PieceSink } from "./ground.ts";

/** A patch's side, m, and its cells across. */
export const PATCH_M = 64;
const CELLS = 32;
/** Terrain coarser than this is never built from (tens of metres out at the coarse levels: a
 * patch built from one and then from the fine tile left the truck under the ground). */
const MAX_ERROR_M = 8;

interface TerrainDataLike {
  interpolateHeight(rectangle: C.Rectangle, lon: number, lat: number): number | undefined;
}
interface QuadTile {
  level: number;
  rectangle: C.Rectangle;
  data: { terrainData: TerrainDataLike | undefined } | undefined;
}
interface GlobeInternals {
  _surface: {
    _tilesToRender: QuadTile[];
    tileProvider: { getLevelMaximumGeometricError(level: number): number };
  };
}

interface Patch {
  piece: Piece;
  /** The drawn tiles its heights came from. */
  from: Set<QuadTile>;
  /** Its middle, radians: lon, lat. */
  mid: [number, number];
}

/** A grid's triangles, (CELLS+1)² vertices row by row, two a cell; shared by every patch. */
const INDICES = (() => {
  const n = CELLS + 1;
  const out = new Uint32Array(CELLS * CELLS * 6);
  let k = 0;
  for (let r = 0; r < CELLS; r++) {
    for (let c = 0; c < CELLS; c++) {
      const a = r * n + c;
      out.set([a, a + 1, a + n + 1, a, a + n + 1, a + n], k);
      k += 6;
    }
  }
  return out;
})();

const toRad = Math.PI / 180;

export class PatchBuilder implements Builder {
  private readonly patches = new Map<string, Patch>();
  /** The tiles drawn last frame, with terrain data, finest first. */
  private drawn: QuadTile[] = [];
  /** Whether a level is fine enough to build from. */
  private fine: (level: number) => boolean = () => false;

  constructor(
    private readonly scene: C.Scene,
    private readonly sink: PieceSink,
    private readonly buildM: number,
    private readonly dropM: number,
  ) {}

  dispose(): void {}

  build(foci: Vec3[], budgetMs: number): number {
    const start = performance.now();
    const surface = (this.scene.globe as unknown as GlobeInternals)._surface;
    const fine = (level: number) =>
      surface.tileProvider.getLevelMaximumGeometricError(level) <= MAX_ERROR_M;
    this.drawn = surface._tilesToRender
      .filter((t) => t.data?.terrainData)
      .sort((a, b) => b.level - a.level);
    this.fine = fine;
    const centre = (i: number, j: number): [number, number] => [
      (i + 0.5) * PATCH_M,
      (j + 0.5) * PATCH_M,
    ];
    const near = (i: number, j: number, r: number) =>
      foci.some((f) => {
        const [x, y] = centre(i, j);
        return Math.hypot(x - f[0], y - f[1]) < r + PATCH_M;
      });
    // Gone: far from every focus.
    for (const [k, p] of this.patches) {
      const [i, j] = k.split(",").map(Number) as [number, number];
      if (foci.length && !near(i, j, this.dropM)) this.drop(k, p);
    }
    // Wanted: not built, or built from a coarser tile than is drawn there now; nearest first.
    const wanted: [number, number, number][] = [];
    const seen = new Set<string>();
    for (const f of foci) {
      const lo = (x: number) => Math.floor((x - this.buildM) / PATCH_M);
      const hi = (x: number) => Math.floor((x + this.buildM) / PATCH_M);
      for (let i = lo(f[0]); i <= hi(f[0]); i++) {
        for (let j = lo(f[1]); j <= hi(f[1]); j++) {
          const k = `${i},${j}`;
          if (seen.has(k) || !near(i, j, this.buildM)) continue;
          seen.add(k);
          const was = this.patches.get(k);
          const under = was && this.tileAt(was.mid[0], was.mid[1]);
          if (under && was.from.has(under)) continue;
          const [x, y] = centre(i, j);
          const d = Math.min(...foci.map((g) => Math.hypot(x - g[0], y - g[1])));
          wanted.push([d, i, j]);
        }
      }
    }
    wanted.sort((a, b) => a[0] - b[0]);
    let pending = 0;
    for (const [, i, j] of wanted) {
      // Wanted but not drawn there (behind the camera): nothing to build it from, not pending.
      if (performance.now() - start > budgetMs) pending++;
      else if (this.make(i, j) === "waiting") pending++;
    }
    return pending;
  }

  /** The finest drawn tile with terrain data over (`lon`, `lat`), radians. */
  private tileAt(lon: number, lat: number): QuadTile | undefined {
    for (const t of this.drawn) {
      const r = t.rectangle;
      if (lon >= r.west && lon <= r.east && lat >= r.south && lat <= r.north) return t;
    }
    return undefined;
  }

  /** Patch (i, j) from the drawn terrain: built; not drawn there at all; or drawn, its heights
   * not ready yet. */
  private make(i: number, j: number): "built" | "undrawn" | "waiting" {
    const n = CELLS + 1;
    const step = PATCH_M / CELLS;
    const f = this.sink.frame;
    const vertices = new Float32Array(n * n * 3);
    const from = new Set<QuadTile>();
    let tile: QuadTile | undefined;
    for (let r = 0; r < n; r++) {
      const north = j * PATCH_M + r * step;
      for (let c = 0; c < n; c++) {
        const east = i * PATCH_M + c * step;
        const [latDeg, lonDeg] = moveBy(f.lat, f.lon, north, east);
        const lon = lonDeg * toRad;
        const lat = latDeg * toRad;
        const tr = tile?.rectangle;
        if (!tile || !tr || lon < tr.west || lon > tr.east || lat < tr.south || lat > tr.north) {
          tile = this.tileAt(lon, lat);
          if (!tile) return "undrawn";
          // Drawn coarse for now: the fine tile is coming.
          if (!this.fine(tile.level)) return "waiting";
          from.add(tile);
        }
        const h = tile.data?.terrainData?.interpolateHeight(tile.rectangle, lon, lat);
        if (h === undefined) return "waiting";
        const v = (r * n + c) * 3;
        vertices[v] = east;
        vertices[v + 1] = north;
        // Up in the tangent plane falls below the curved ground with distance.
        vertices[v + 2] = h - f.h - (east * east + north * north) / (2 * EARTH_R);
      }
    }
    const k = `${i},${j}`;
    const was = this.patches.get(k);
    if (was) this.drop(k, was);
    const [midLat, midLon] = moveBy(f.lat, f.lon, (j + 0.5) * PATCH_M, (i + 0.5) * PATCH_M);
    this.patches.set(k, {
      piece: this.sink.add({ vertices, indices: INDICES }),
      from,
      mid: [midLon * toRad, midLat * toRad],
    });
    return "built";
  }

  private drop(k: string, p: Patch): void {
    this.sink.remove(p.piece);
    this.patches.delete(k);
  }
}
