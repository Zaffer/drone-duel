// The truck's ground (ported from the old drive's tilecolliders.js): each 3D tile drawn near
// the truck becomes a Rapier trimesh collider holding that tile's own triangles, so the truck
// drives on, and runs into, exactly what the map shows. The physics world is a local
// east-north-up frame in metres.
//
// The colliders follow what Cesium draws, level of detail and all: a tile is built when it is
// drawn near the truck, and a collider whose tile has since been replaced on screen by a finer
// or coarser one goes once everything replacing it is built, so the ground never has a hole and
// settles with no tile twice. A collider out of view stays until the truck is far from it: the
// chase camera does not see beside or behind the truck.
//
// Without 3D tiles (the globe's terrain), the ground is a flat plane at the start's height.

import type RAPIER from "@dimforge/rapier3d-compat";
import * as C from "cesium";
import type { XYZ } from "./frame.ts";

type Rapier = typeof RAPIER;

/** Build tiles drawn within this distance of the truck, metres from the tile's bounds. */
const BUILD_RADIUS_M = 150;
/** Drop colliders this far from the truck. */
const DROP_RADIUS_M = 400;
/** Coarser tiles are never built. Google's finest are about 2 m, 33 m across; the coarse ones
 * above them reach whole continents. */
const MAX_GEOMETRIC_ERROR_M = 8;
/** Milliseconds of tile building per frame. A Google tile takes about 2. */
const BUILD_BUDGET_MS = 6;
/** The flat ground's half size, m. */
const FLAT_HALF_M = 5000;
const DEBUG_COLOR = C.Color.fromCssColorString("#22C7B8").withAlpha(0.55);

export interface GroundStats {
  tiles: number;
  triangles: number;
  buildMs: number;
  /** Tiles wanted but not built yet: the ground is still settling. */
  pending: number;
}

/** What the truck drives on. */
export interface Ground {
  /** The truck's position, local frame. */
  focus: C.Cartesian3;
  readonly stats: GroundStats;
  readonly debug: boolean;
  /** Draw every collider's triangle edges over the map, or stop. */
  setDebug(on: boolean): void;
  /** The distance along a ray (local frame, `dir` a unit vector) to the first fixed collider,
   * or null within `maxToi`. */
  castRay(from: XYZ, dir: XYZ, maxToi: number): number | null;
  destroy(): void;
}

// Cesium keeps these private or untyped; the old drive read them the same way.
interface TileLike {
  parent: TileLike | undefined;
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
      primitive: {
        primitiveType: C.PrimitiveType;
        indices?: unknown;
        attributes: Accessor[];
      },
      instances: { transform: C.Matrix4 }[],
    ) => void,
  ): void;
  readAttributeAsTypedArray(attribute: Accessor): Float32Array;
  readIndicesAsTypedArray(indices: unknown): Uint8Array | Uint16Array | Uint32Array;
}
const ModelReader = (C as unknown as { ModelReader: ModelReaderApi }).ModelReader;

/** A tile's ancestors, nearest first. */
function ancestors(tile: TileLike): TileLike[] {
  const out: TileLike[] = [];
  for (let t = tile.parent; t; t = t.parent) out.push(t);
  return out;
}

/** The glTF model of a tile's content, when it has one loaded. */
function tileModel(tile: TileLike): C.Model | undefined {
  const content = tile.contentReady ? tile.content : undefined;
  const model = content?._model;
  return model?.ready ? model : undefined;
}

interface Built {
  collider: RAPIER.Collider | null;
  triangles: number;
  vertices: Float32Array;
  indices: Uint32Array;
  debug?: C.Primitive;
}

const scratchCentre = new C.Cartesian3();

function castRay(R: Rapier, world: RAPIER.World, from: XYZ, dir: XYZ, maxToi: number) {
  const hit = world.castRay(new R.Ray(from, dir), maxToi, true, R.QueryFilterFlags.ONLY_FIXED);
  return hit ? hit.timeOfImpact : null;
}

export class TileColliders implements Ground {
  readonly built = new Map<TileLike, Built>();
  /** Tiles drawn in the current frame, and in the last finished one. */
  private drawing = new Set<TileLike>();
  private drawn = new Set<TileLike>();
  private readonly fixedToLocal: C.Matrix4;
  private readonly unlisten: (() => void)[];
  focus = new C.Cartesian3();
  debug = false;
  readonly stats: GroundStats = { tiles: 0, triangles: 0, buildMs: 0, pending: 0 };

  constructor(
    private readonly scene: C.Scene,
    tileset: C.Cesium3DTileset,
    private readonly R: Rapier,
    private readonly world: RAPIER.World,
    private readonly localToFixed: C.Matrix4,
  ) {
    this.fixedToLocal = C.Matrix4.inverseTransformation(localToFixed, new C.Matrix4());
    this.unlisten = [
      tileset.tileVisible.addEventListener((tile: TileLike) => {
        this.drawing.add(tile);
      }),
      scene.postRender.addEventListener(() => this.frame()),
    ];
  }

  destroy(): void {
    for (const off of this.unlisten) off();
    this.setDebug(false);
    this.built.clear();
  }

  castRay(from: XYZ, dir: XYZ, maxToi: number): number | null {
    return castRay(this.R, this.world, from, dir, maxToi);
  }

  /** The tiles drawn this frame that the truck needs and that are not built, nearest first. */
  private wanted(): TileLike[] {
    const wanted: [number, TileLike][] = [];
    for (const tile of this.drawn) {
      if (this.built.has(tile) || tile.geometricError > MAX_GEOMETRIC_ERROR_M) continue;
      if (!tileModel(tile)) continue;
      const d = this.distance(tile);
      // A tile drawn in place of a built one is wanted further out, so the built one can go.
      const replacing = d < DROP_RADIUS_M && ancestors(tile).some((a) => this.built.has(a));
      if (d < BUILD_RADIUS_M || replacing) wanted.push([d, tile]);
    }
    return wanted.sort((a, b) => a[0] - b[0]).map(([, tile]) => tile);
  }

  /** Distance from the truck to a tile's bounding sphere, metres (0 inside it). */
  private distance(tile: TileLike): number {
    const sphere = tile.boundingSphere;
    const centre = C.Matrix4.multiplyByPoint(this.fixedToLocal, sphere.center, scratchCentre);
    return Math.max(0, C.Cartesian3.distance(centre, this.focus) - sphere.radius);
  }

  private frame(): void {
    [this.drawn, this.drawing] = [this.drawing, this.drawn];
    this.drawing.clear();
    const wanted = this.wanted();
    const start = performance.now();
    let i = 0;
    for (; i < wanted.length && performance.now() - start < BUILD_BUDGET_MS; i++) {
      this.build(wanted[i]!);
    }
    const pending = wanted.slice(i);
    this.prune(pending);
    this.stats.pending = pending.length;
    this.stats.buildMs = performance.now() - start;
  }

  /** Drop colliders far from the truck, and colliders whose tile is no longer drawn because a
   * built tile related to it (an ancestor or a descendant) is, once no tile that is still to be
   * built is related to it either. */
  private prune(pending: TileLike[]): void {
    const drawnBuilt = new Set<TileLike>();
    const aboveDrawnBuilt = new Set<TileLike>();
    for (const tile of this.built.keys()) {
      if (!this.drawn.has(tile)) continue;
      drawnBuilt.add(tile);
      for (const a of ancestors(tile)) aboveDrawnBuilt.add(a);
    }
    const pendingSet = new Set(pending);
    const abovePending = new Set<TileLike>();
    for (const tile of pending) for (const a of ancestors(tile)) abovePending.add(a);
    for (const tile of [...this.built.keys()]) {
      if (this.distance(tile) > DROP_RADIUS_M) {
        this.drop(tile);
        continue;
      }
      if (this.drawn.has(tile) || abovePending.has(tile)) continue;
      const up = ancestors(tile);
      if (up.some((a) => pendingSet.has(a))) continue;
      const replaced = aboveDrawnBuilt.has(tile) || up.some((a) => drawnBuilt.has(a));
      if (replaced) this.drop(tile);
    }
  }

  /** Read a tile's triangles back into a trimesh collider. */
  private build(tile: TileLike): void {
    const model = tileModel(tile)!;
    const parts: {
      vertices: Float32Array;
      indices: ArrayLike<number>;
      transform: C.Matrix4;
      base: number;
    }[] = [];
    let vertexCount = 0;
    ModelReader.forEachPrimitive(model, {}, (_runtime, primitive, instances) => {
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
      // Vertex -> earth-fixed -> local, in doubles; float32 is ample once local.
      C.Matrix4.multiply(this.fixedToLocal, p.transform, m);
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
    const triangles = indices.length / 3;
    const collider =
      triangles > 0
        ? this.world.createCollider(this.R.ColliderDesc.trimesh(vertices, indices).setFriction(0.9))
        : null;
    const entry: Built = { collider, triangles, vertices, indices };
    this.built.set(tile, entry);
    this.stats.tiles = this.built.size;
    this.stats.triangles += triangles;
    if (this.debug) this.showDebug(entry);
  }

  private drop(tile: TileLike): void {
    const entry = this.built.get(tile);
    if (!entry) return;
    if (entry.collider) this.world.removeCollider(entry.collider, false);
    if (entry.debug) this.scene.primitives.remove(entry.debug);
    this.built.delete(tile);
    this.stats.tiles = this.built.size;
    this.stats.triangles -= entry.triangles;
  }

  setDebug(on: boolean): void {
    this.debug = on;
    for (const entry of this.built.values()) {
      if (on) this.showDebug(entry);
      else if (entry.debug) {
        this.scene.primitives.remove(entry.debug);
        entry.debug = undefined;
      }
    }
  }

  private showDebug(entry: Built): void {
    if (entry.debug || entry.triangles === 0) return;
    entry.debug = this.scene.primitives.add(
      wireframe(entry.vertices, entry.indices, this.localToFixed),
    ) as C.Primitive;
  }
}

/** A triangle mesh's edges as a line primitive. */
function wireframe(vertices: Float32Array, indices: Uint32Array, modelMatrix: C.Matrix4) {
  const lines = new Uint32Array(indices.length * 2);
  for (let t = 0, j = 0; t < indices.length; t += 3) {
    const a = indices[t]!;
    const b = indices[t + 1]!;
    const c = indices[t + 2]!;
    lines[j++] = a;
    lines[j++] = b;
    lines[j++] = b;
    lines[j++] = c;
    lines[j++] = c;
    lines[j++] = a;
  }
  const geometry = new C.Geometry({
    attributes: {
      position: new C.GeometryAttribute({
        componentDatatype: C.ComponentDatatype.DOUBLE,
        componentsPerAttribute: 3,
        values: Float64Array.from(vertices),
      }),
    } as C.GeometryAttributes,
    indices: lines,
    primitiveType: C.PrimitiveType.LINES,
    boundingSphere: C.BoundingSphere.fromVertices(Array.from(vertices)),
  });
  return new C.Primitive({
    geometryInstances: new C.GeometryInstance({
      geometry,
      attributes: { color: C.ColorGeometryInstanceAttribute.fromColor(DEBUG_COLOR) },
    }),
    appearance: new C.PerInstanceColorAppearance({ flat: true, translucent: true }),
    modelMatrix: C.Matrix4.clone(modelMatrix),
    asynchronous: false,
  });
}

/** A flat plane at the local frame's height, for a map without 3D tiles. */
export class FlatGround implements Ground {
  focus = new C.Cartesian3();
  readonly debug = false;
  readonly stats: GroundStats = { tiles: 0, triangles: 2, buildMs: 0, pending: 0 };

  constructor(
    private readonly R: Rapier,
    private readonly world: RAPIER.World,
  ) {
    world.createCollider(
      R.ColliderDesc.cuboid(FLAT_HALF_M, FLAT_HALF_M, 1).setTranslation(0, 0, -1).setFriction(0.9),
    );
  }

  setDebug(): void {}

  castRay(from: XYZ, dir: XYZ, maxToi: number): number | null {
    return castRay(this.R, this.world, from, dir, maxToi);
  }

  destroy(): void {}
}
