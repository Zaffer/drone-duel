// The ground from Google's photorealistic 3D tiles (engine/ground.ts): each tile Cesium drew last
// frame near a focus, its glTF model's triangles read back as drawn, a trimesh collider. The
// colliders follow what Cesium draws, level of detail and all: a tile is built when it is drawn
// within BUILD_RADIUS_M of a focus, again when it is drawn from newer data, and a collider whose
// tile has since been replaced on screen by a finer or coarser one goes once everything
// replacing it is built, so the ground never has a hole and settles with no tile twice. A
// collider out of view stays until every focus is far from it: a chase camera does not see
// beside or behind. Coarse tiles (metres out) are never built. Cesium keeps much of this
// private; it is read here and nowhere else.

import * as C from "cesium";
import type { Vec3 } from "../world/geo.ts";
import type { Builder, Piece, PieceSink } from "./ground.ts";

export interface Triangles {
  /** Local (the frame's) x, y, z, float32: ample once local. */
  vertices: Float32Array;
  indices: Uint32Array;
}

export interface TileSource<T extends object> {
  /** The tiles drawn in the last frame drawn. */
  readonly drawn: ReadonlySet<T>;
  parent(t: T): T | undefined;
  /** Its bounds, earth-fixed. */
  sphere(t: T): C.BoundingSphere;
  /** Whether it can be built now: loaded, and fine enough (a coarse tile is metres out). */
  buildable(t: T): boolean;
  /** What it is drawn from now: a tile built from older data is built again. */
  version(t: T): object | undefined;
  /** Its triangles, in the frame whose earth-fixed-to-local matrix is `toLocal`. */
  read(t: T, toLocal: C.Matrix4): Triangles;
  dispose(): void;
}

/** Google's finest tiles are about 2 m out, 33 m across; coarser are never built. */
const MAX_TILE_ERROR_M = 8;

// -- Google's 3D tiles ----------------------------------------------------------------------

interface Tile3D {
  parent: Tile3D | undefined;
  geometricError: number;
  boundingSphere: C.BoundingSphere;
  contentReady: boolean;
  content: { _model?: C.Model } | undefined;
}
interface Accessor {
  semantic: string;
}
interface ModelReaderApi {
  forEachPrimitive(
    model: C.Model,
    options: object,
    callback: (
      runtime: unknown,
      primitive: { primitiveType: C.PrimitiveType; indices?: unknown; attributes: Accessor[] },
      instances: { transform: C.Matrix4 }[],
    ) => void,
  ): void;
  readAttributeAsTypedArray(attribute: Accessor): Float32Array;
  readIndicesAsTypedArray(indices: unknown): Uint8Array | Uint16Array | Uint32Array;
}
const ModelReader = (C as unknown as { ModelReader: ModelReaderApi }).ModelReader;

/** The glTF model of a tile's content, when it has one loaded. */
function tileModel(tile: Tile3D): C.Model | undefined {
  const content = tile.contentReady ? tile.content : undefined;
  const model = content?._model;
  return model?.ready ? model : undefined;
}

/** Each frame's drawn set, swapped in Cesium's postRender. */
function drawnSets<T>(scene: C.Scene, fill: (into: Set<T>) => void) {
  let drawing = new Set<T>();
  let drawn = new Set<T>();
  const off = scene.postRender.addEventListener(() => {
    fill(drawing);
    [drawn, drawing] = [drawing, drawn];
    drawing.clear();
  });
  return { get: () => drawn as ReadonlySet<T>, adding: () => drawing, off };
}

export function googleTiles(scene: C.Scene, tileset: C.Cesium3DTileset): TileSource<Tile3D> {
  const sets = drawnSets<Tile3D>(scene, () => {});
  const offVisible = tileset.tileVisible.addEventListener((tile: Tile3D) => {
    sets.adding().add(tile);
  });
  return {
    get drawn() {
      return sets.get();
    },
    parent: (t) => t.parent,
    sphere: (t) => t.boundingSphere,
    buildable: (t) => t.geometricError <= MAX_TILE_ERROR_M && !!tileModel(t),
    version: (t) => tileModel(t),
    read(t, toLocal) {
      const parts: {
        vertices: Float32Array;
        indices: ArrayLike<number>;
        transform: C.Matrix4;
        base: number;
      }[] = [];
      let vertexCount = 0;
      ModelReader.forEachPrimitive(tileModel(t)!, {}, (_runtime, primitive, instances) => {
        if (primitive.primitiveType !== C.PrimitiveType.TRIANGLES || !primitive.indices) return;
        const position = primitive.attributes.find((a) => a.semantic === "POSITION");
        if (!position) return;
        const vertices = ModelReader.readAttributeAsTypedArray(position);
        const indices = ModelReader.readIndicesAsTypedArray(primitive.indices);
        for (const instance of instances) {
          parts.push({ vertices, indices, transform: instance.transform, base: vertexCount });
          vertexCount += vertices.length / 3;
        }
      });
      const vertices = new Float32Array(vertexCount * 3);
      let indexCount = 0;
      for (const p of parts) indexCount += p.indices.length;
      const indices = new Uint32Array(indexCount);
      let iv = 0;
      let ii = 0;
      const m = new C.Matrix4();
      for (const p of parts) {
        // Vertex -> earth-fixed -> local, in doubles.
        C.Matrix4.multiply(toLocal, p.transform, m);
        const v = p.vertices;
        for (let k = 0; k < v.length; k += 3) {
          const x = v[k]!;
          const y = v[k + 1]!;
          const z = v[k + 2]!;
          vertices[iv++] = m[0]! * x + m[4]! * y + m[8]! * z + m[12]!;
          vertices[iv++] = m[1]! * x + m[5]! * y + m[9]! * z + m[13]!;
          vertices[iv++] = m[2]! * x + m[6]! * y + m[10]! * z + m[14]!;
        }
        for (let k = 0; k < p.indices.length; k++) indices[ii++] = p.indices[k]! + p.base;
      }
      return { vertices, indices };
    },
    dispose() {
      offVisible();
      sets.off();
    },
  };
}

// -- building -------------------------------------------------------------------------------

interface Built {
  piece: Piece;
  version: object | undefined;
  /** Its bounds, local east and north: min, max. */
  box: [number, number, number, number];
}

const scratchCentre = new C.Cartesian3();

export class TileBuilder<T extends object> implements Builder {
  private readonly built = new Map<T, Built>();
  private foci: Vec3[] = [];

  constructor(
    private readonly source: TileSource<T>,
    private readonly sink: PieceSink,
    private readonly buildM: number,
    private readonly dropM: number,
  ) {}

  dispose(): void {
    this.source.dispose();
  }

  build(foci: Vec3[], budgetMs: number): number {
    this.foci = foci;
    const wanted = this.wanted();
    const start = performance.now();
    let i = 0;
    for (; i < wanted.length && performance.now() - start < budgetMs; i++) {
      this.buildTile(wanted[i]!);
    }
    const pending = wanted.slice(i);
    this.prune(pending);
    return pending.length;
  }

  private ancestors(tile: T): T[] {
    const out: T[] = [];
    for (let t = this.source.parent(tile); t; t = this.source.parent(t)) out.push(t);
    return out;
  }

  /** The tiles drawn last frame that a focus needs and that are not built (or were built from
   * older data), nearest first. */
  private wanted(): T[] {
    if (this.foci.length === 0) return [];
    const src = this.source;
    const wanted: [number, T][] = [];
    for (const tile of src.drawn) {
      if (!src.buildable(tile)) continue;
      const was = this.built.get(tile);
      if (was && was.version === src.version(tile)) continue;
      const d = this.distance(tile);
      // A tile drawn in place of a built one is wanted further out, so the built one can go.
      const replacing = d < this.dropM && this.ancestors(tile).some((a) => this.built.has(a));
      if (d < this.buildM || replacing || was) wanted.push([d, tile]);
    }
    return wanted.sort((a, b) => a[0] - b[0]).map(([, tile]) => tile);
  }

  /** Distance from the nearest focus to a tile's bounding sphere, metres (0 inside it). */
  private distance(tile: T): number {
    const sphere = this.source.sphere(tile);
    const c = C.Matrix4.multiplyByPoint(this.sink.frame.toLocalM, sphere.center, scratchCentre);
    let best = Number.POSITIVE_INFINITY;
    for (const f of this.foci) {
      best = Math.min(best, Math.hypot(c.x - f[0], c.y - f[1], c.z - f[2]) - sphere.radius);
    }
    return Math.max(0, best);
  }

  /** Drop colliders far from every focus, and colliders whose tile is no longer drawn because a
   * built tile related to it (an ancestor or a descendant) is, once no tile that is still to be
   * built is related to it either. */
  private prune(pending: T[]): void {
    const drawn = this.source.drawn;
    const drawnBuilt = new Set<T>();
    const aboveDrawnBuilt = new Set<T>();
    for (const tile of this.built.keys()) {
      if (!drawn.has(tile)) continue;
      drawnBuilt.add(tile);
      for (const a of this.ancestors(tile)) aboveDrawnBuilt.add(a);
    }
    const pendingSet = new Set(pending);
    const abovePending = new Set<T>();
    for (const tile of pending) for (const a of this.ancestors(tile)) abovePending.add(a);
    for (const [tile, entry] of [...this.built]) {
      if (this.foci.length > 0 && this.far(entry)) {
        this.drop(tile);
        continue;
      }
      if (drawn.has(tile) || abovePending.has(tile)) continue;
      const up = this.ancestors(tile);
      if (up.some((a) => pendingSet.has(a))) continue;
      const replaced = aboveDrawnBuilt.has(tile) || up.some((a) => drawnBuilt.has(a));
      if (replaced) this.drop(tile);
    }
  }

  /** Whether a built tile is `dropM` from every focus, by its own bounds. */
  private far(entry: Built): boolean {
    const [x0, y0, x1, y1] = entry.box;
    for (const f of this.foci) {
      const dx = Math.max(x0 - f[0], 0, f[0] - x1);
      const dy = Math.max(y0 - f[1], 0, f[1] - y1);
      if (Math.hypot(dx, dy) < this.dropM) return false;
    }
    return true;
  }

  private buildTile(tile: T): void {
    this.drop(tile);
    const version = this.source.version(tile);
    const tri = this.source.read(tile, this.sink.frame.toLocalM);
    this.built.set(tile, { piece: this.sink.add(tri), version, box: bounds(tri.vertices) });
  }

  private drop(tile: T): void {
    const entry = this.built.get(tile);
    if (!entry) return;
    this.sink.remove(entry.piece);
    this.built.delete(tile);
  }
}

/** Local vertices' east and north bounds: min, max. */
function bounds(v: Float32Array): [number, number, number, number] {
  if (v.length < 3) return [0, 0, 0, 0];
  let [x0, y0, x1, y1] = [v[0]!, v[1]!, v[0]!, v[1]!];
  for (let k = 3; k < v.length; k += 3) {
    x0 = Math.min(x0, v[k]!);
    x1 = Math.max(x1, v[k]!);
    y0 = Math.min(y0, v[k + 1]!);
    y1 = Math.max(y1, v[k + 1]!);
  }
  return [x0, y0, x1, y1];
}
