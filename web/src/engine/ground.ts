// The ground (DESIGN.md §3.2): what everything in the page stands on, runs into and hits, asked
// in microseconds. Each of Google's 3D tiles drawn near a focus (the vehicle, the shooter, the
// truck) is read back into a Rapier trimesh collider holding that tile's own triangles, so the
// ground is exactly what the map shows, and a question of it (a ray, a segment, a height) is a
// walk of Rapier's tree in memory: no render, no wait. (Asking Cesium instead, sampleHeight and
// its kin, is a render of its own each time: about a millisecond on Google's tiles. Those stay
// in world.ts, for the few places nothing is built yet.)
//
// The colliders follow what Cesium draws, level of detail and all: a tile is built when it is
// drawn within BUILD_RADIUS_M of a focus, and a collider whose tile has since been replaced on
// screen by a finer or coarser one goes once everything replacing it is built, so the ground
// never has a hole and settles with no tile twice. A collider out of view stays until every
// focus is far from it: a chase camera does not see beside or behind. Coarse tiles (kilometres
// across, metres out) are never built: past the built ground, there is none to ask. Building is
// a few milliseconds a frame at most, timed as "ground build".
//
// Without 3D tiles (the globe's terrain), the ground is a flat plane at the frame's origin.
//
// The ground owns the page's one physics world, and steps it at a fixed PHYSICS_HZ (the frame
// loop's "physics"): a new collider reaches Rapier's queries only with a step. What drives a body
// in it (the truck) does so in `onStep`, just before each.

import RAPIER from "@dimforge/rapier3d-compat";
import * as C from "cesium";
import type { Vec3 } from "../world/geo.ts";
import type { World } from "../world/world.ts";
import type { LocalFrame } from "./frame.ts";
import { loop } from "./loop.ts";

type Rapier = typeof RAPIER;

/** Build tiles drawn within this distance of a focus, metres from the tile's bounds. */
const BUILD_RADIUS_M = 150;
/** Drop colliders this far from every focus. */
const DROP_RADIUS_M = 400;
/** Coarser tiles are never built. Google's finest are about 2 m, 33 m across; the coarse ones
 * above them reach whole continents. */
const MAX_GEOMETRIC_ERROR_M = 8;
/** Milliseconds of tile building a frame. A Google tile takes about 2. */
const BUILD_BUDGET_MS = 6;
/** The physics' fixed rate, Hz. */
export const PHYSICS_HZ = 60;
/** The flat ground's half size, m. */
const FLAT_HALF_M = 5000;
/** A height is looked for from this high, down this far, m (local). */
const TOP_M = 3000;
const DEBUG_COLOR = C.Color.fromCssColorString("#22C7B8").withAlpha(0.55);

export interface GroundStats {
  tiles: number;
  triangles: number;
  /** Tiles wanted but not built yet: the ground is still settling. */
  pending: number;
}

export interface Hit {
  /** Distance along the way, m. */
  toi: number;
  point: Vec3;
  /** The surface's normal there, unit. */
  normal: Vec3;
}

let rapier: Promise<Rapier> | null = null;
/** Rapier, loaded and initialised once for the page. */
export function loadRapier(): Promise<Rapier> {
  rapier ??= RAPIER.init().then(() => RAPIER);
  return rapier;
}

// Cesium keeps these private or untyped.
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

export class Ground {
  readonly stats: GroundStats = { tiles: 0, triangles: 0, pending: 0 };
  debug = false;
  private readonly built = new Map<TileLike, Built>();
  /** Tiles drawn in the frame being drawn, and in the last one drawn. */
  private drawing = new Set<TileLike>();
  private drawn = new Set<TileLike>();
  private readonly foci = new Map<string, Vec3>();
  private readonly steppers = new Set<(h: number) => void>();
  private readonly abort = new AbortController();
  private readonly unlisten: (() => void)[] = [];
  private readonly ray = { origin: { x: 0, y: 0, z: 0 }, dir: { x: 0, y: 0, z: -1 } };
  private readonly rapierRay: RAPIER.Ray;

  private constructor(
    private readonly world: World,
    readonly frame: LocalFrame,
    readonly R: Rapier,
    readonly physics: RAPIER.World,
    tileset: C.Cesium3DTileset | null,
  ) {
    this.rapierRay = new R.Ray(this.ray.origin, this.ray.dir);
    physics.timestep = 1 / PHYSICS_HZ;
    const signal = this.abort.signal;
    if (tileset) {
      this.unlisten.push(
        tileset.tileVisible.addEventListener((tile: TileLike) => {
          this.drawing.add(tile);
        }),
        world.scene.postRender.addEventListener(() => {
          [this.drawn, this.drawing] = [this.drawing, this.drawn];
          this.drawing.clear();
        }),
      );
      loop.each("ground build", "sim", () => this.build(), signal);
    } else {
      physics.createCollider(
        R.ColliderDesc.cuboid(FLAT_HALF_M, FLAT_HALF_M, 1)
          .setTranslation(0, 0, -1)
          .setFriction(0.9),
      );
      this.stats.triangles = 2;
    }
    loop.fixed(
      "physics",
      PHYSICS_HZ,
      (h) => {
        for (const f of this.steppers) f(h);
        physics.step();
      },
      signal,
    );
  }

  /** The ground around `frame`, the 3D tiles' when they are the map's ground. */
  static async create(world: World, frame: LocalFrame): Promise<Ground> {
    const R = await loadRapier();
    await world.ready;
    const physics = new R.World({ x: 0, y: 0, z: -9.81 });
    return new Ground(world, frame, R, physics, world.tileset);
  }

  dispose(): void {
    this.abort.abort();
    for (const off of this.unlisten) off();
    this.setDebug(false);
    this.built.clear();
    this.steppers.clear();
    this.physics.free();
  }

  /** Build the ground around `p` (local), as `name` (the vehicle, the shooter); null: no more. */
  focus(name: string, p: Vec3 | null): void {
    if (p) this.foci.set(name, p);
    else this.foci.delete(name);
  }

  /** Run `f(h)` before each physics step, h s; the returned function stops it. */
  onStep(f: (h: number) => void): () => void {
    this.steppers.add(f);
    return () => this.steppers.delete(f);
  }

  /** Whether the ground's colliders are still being built round the foci. */
  get settling(): boolean {
    return this.stats.pending > 0;
  }

  // -- questions ---------------------------------------------------------------------------

  /** The first ground along `dir` (unit) from `from`, within `max` m. */
  cast(from: Vec3, dir: Vec3, max: number): Hit | null {
    const o = this.ray.origin;
    const d = this.ray.dir;
    [o.x, o.y, o.z] = from;
    [d.x, d.y, d.z] = dir;
    const hit = this.physics.castRayAndGetNormal(
      this.rapierRay,
      max,
      true,
      this.R.QueryFilterFlags.ONLY_FIXED,
    );
    if (!hit) return null;
    const t = hit.timeOfImpact;
    return {
      toi: t,
      point: [from[0] + dir[0] * t, from[1] + dir[1] * t, from[2] + dir[2] * t],
      normal: [hit.normal.x, hit.normal.y, hit.normal.z],
    };
  }

  /** The first ground between `a` and `b`. */
  segment(a: Vec3, b: Vec3): Hit | null {
    const d: Vec3 = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
    const len = Math.hypot(d[0], d[1], d[2]);
    if (len < 1e-9) return null;
    return this.cast(a, [d[0] / len, d[1] / len, d[2] / len], len);
  }

  /** The ground's height (local up) at (`e`, `n`): the first met looking down from `from` (a
   * little over the feet, so a tree's crown over them is not the ground; default: from high).
   * Undefined where none is built. */
  heightAt(e: number, n: number, from = TOP_M): number | undefined {
    const hit = this.cast([e, n, from], [0, 0, -1], from + TOP_M);
    return hit ? hit.point[2] : undefined;
  }

  /** Whether `p` is where ground is built (near a focus): a question elsewhere finds nothing. */
  covers(p: Vec3): boolean {
    for (const f of this.foci.values()) {
      if (Math.hypot(p[0] - f[0], p[1] - f[1]) < BUILD_RADIUS_M) return true;
    }
    return !this.world.tileset;
  }

  // -- building ----------------------------------------------------------------------------

  private build(): void {
    const wanted = this.wanted();
    const start = performance.now();
    let i = 0;
    for (; i < wanted.length && performance.now() - start < BUILD_BUDGET_MS; i++) {
      this.buildTile(wanted[i]!);
    }
    const pending = wanted.slice(i);
    this.prune(pending);
    this.stats.pending = pending.length;
  }

  /** The tiles drawn last frame that a focus needs and that are not built, nearest first. */
  private wanted(): TileLike[] {
    if (this.foci.size === 0) return [];
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

  /** Distance from the nearest focus to a tile's bounding sphere, metres (0 inside it). */
  private distance(tile: TileLike): number {
    const sphere = tile.boundingSphere;
    const c = C.Matrix4.multiplyByPoint(this.frame.toLocalM, sphere.center, scratchCentre);
    let best = Number.POSITIVE_INFINITY;
    for (const f of this.foci.values()) {
      const d = Math.hypot(c.x - f[0], c.y - f[1], c.z - f[2]) - sphere.radius;
      best = Math.min(best, d);
    }
    return Math.max(0, best);
  }

  /** Drop colliders far from every focus, and colliders whose tile is no longer drawn because a
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
      if (this.foci.size > 0 && this.distance(tile) > DROP_RADIUS_M) {
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
  private buildTile(tile: TileLike): void {
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
      C.Matrix4.multiply(this.frame.toLocalM, p.transform, m);
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
        ? this.physics.createCollider(
            this.R.ColliderDesc.trimesh(vertices, indices).setFriction(0.9),
          )
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
    if (entry.collider) this.physics.removeCollider(entry.collider, false);
    if (entry.debug) this.world.remove(entry.debug);
    this.built.delete(tile);
    this.stats.tiles = this.built.size;
    this.stats.triangles -= entry.triangles;
  }

  /** Draw every collider's triangle edges over the map, or stop. */
  setDebug(on: boolean): void {
    this.debug = on;
    for (const entry of this.built.values()) {
      if (on) this.showDebug(entry);
      else if (entry.debug) {
        this.world.remove(entry.debug);
        entry.debug = undefined;
      }
    }
  }

  private showDebug(entry: Built): void {
    if (entry.debug || entry.triangles === 0) return;
    entry.debug = this.world.add(
      wireframe(entry.vertices, entry.indices, this.frame.toFixedM),
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
