// The ground (DESIGN.md §3.2): what everything in the page stands on, runs into and hits, asked
// in microseconds. The map Cesium draws near each focus (the vehicle, the shooter, the truck) is
// read back into Rapier trimesh colliders, so the ground is what the map shows, and a question of
// it (a ray, a segment, a height) is a walk of Rapier's tree in memory: no render, no wait. Two
// builders, one for each kind of map:
// - Google's 3D tiles: each drawn tile's own triangles, level of detail and all (tiles.ts);
// - the free map, the globe's terrain: patches sampled from the drawn tiles' heights
//   (terrain.ts).
// (Asking Cesium instead, sampleHeight and its kin, is a render of its own each time: about a
// millisecond on Google's tiles. Those stay in world.ts, for where nothing is built yet.)
// Nothing is built past BUILD_RADIUS_M of a focus, and what is built goes DROP_RADIUS_M from
// every focus. Building is a few milliseconds a frame at most, timed as "ground build".
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
import { PatchBuilder } from "./terrain.ts";
import { googleTiles, TileBuilder, type Triangles } from "./tiles.ts";

type Rapier = typeof RAPIER;

/** Build tiles drawn within this distance of a focus, metres from the tile's bounds. */
const BUILD_RADIUS_M = 150;
/** Drop colliders this far from every focus. */
const DROP_RADIUS_M = 400;
/** Milliseconds of building a frame. A Google tile takes about 2, a terrain patch under 1. */
const BUILD_BUDGET_MS = 6;
/** The physics' fixed rate, Hz. */
export const PHYSICS_HZ = 60;
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

/** A collider made of some of the map's triangles, local. */
export interface Piece {
  collider: RAPIER.Collider | null;
  vertices: Float32Array;
  indices: Uint32Array;
  debug?: C.Primitive;
}

/** Where a builder puts its pieces. */
export interface PieceSink {
  readonly frame: LocalFrame;
  add(t: Triangles): Piece;
  remove(p: Piece): void;
}

/** What builds the ground round the foci, `budgetMs` a frame; returns the pieces still wanted. */
export interface Builder {
  build(foci: Vec3[], budgetMs: number): number;
  dispose(): void;
}

export class Ground {
  readonly stats: GroundStats = { tiles: 0, triangles: 0, pending: 0 };
  debug = false;
  private readonly pieces = new Set<Piece>();
  private readonly builder: Builder;
  private readonly foci = new Map<string, Vec3>();
  private readonly steppers = new Set<(h: number) => void>();
  private readonly abort = new AbortController();
  private readonly ray = { origin: { x: 0, y: 0, z: 0 }, dir: { x: 0, y: 0, z: -1 } };
  private readonly rapierRay: RAPIER.Ray;

  private constructor(
    private readonly world: World,
    readonly frame: LocalFrame,
    readonly R: Rapier,
    readonly physics: RAPIER.World,
  ) {
    const sink: PieceSink = { frame, add: (t) => this.add(t), remove: (p) => this.remove(p) };
    this.builder = world.tileset
      ? new TileBuilder(
          googleTiles(world.scene, world.tileset),
          sink,
          BUILD_RADIUS_M,
          DROP_RADIUS_M,
        )
      : new PatchBuilder(world.scene, sink, BUILD_RADIUS_M, DROP_RADIUS_M);
    this.rapierRay = new R.Ray(this.ray.origin, this.ray.dir);
    physics.timestep = 1 / PHYSICS_HZ;
    const signal = this.abort.signal;
    loop.each("ground build", "sim", () => this.build(), signal);
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

  /** The ground around `frame`: the 3D tiles' when they are the map's ground, else the globe's. */
  static async create(world: World, frame: LocalFrame): Promise<Ground> {
    const R = await loadRapier();
    await world.ready;
    const physics = new R.World({ x: 0, y: 0, z: -9.81 });
    return new Ground(world, frame, R, physics);
  }

  dispose(): void {
    this.abort.abort();
    this.builder.dispose();
    this.setDebug(false);
    this.pieces.clear();
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
    return false;
  }

  // -- building ----------------------------------------------------------------------------

  private build(): void {
    this.stats.pending = this.builder.build([...this.foci.values()], BUILD_BUDGET_MS);
  }

  private add({ vertices, indices }: Triangles): Piece {
    const triangles = indices.length / 3;
    const collider =
      triangles > 0
        ? this.physics.createCollider(
            this.R.ColliderDesc.trimesh(vertices, indices).setFriction(0.9),
          )
        : null;
    const piece: Piece = { collider, vertices, indices };
    this.pieces.add(piece);
    this.stats.tiles = this.pieces.size;
    this.stats.triangles += triangles;
    if (this.debug) this.showDebug(piece);
    return piece;
  }

  private remove(p: Piece): void {
    if (!this.pieces.delete(p)) return;
    if (p.collider) this.physics.removeCollider(p.collider, false);
    if (p.debug) this.world.remove(p.debug);
    this.stats.tiles = this.pieces.size;
    this.stats.triangles -= p.indices.length / 3;
  }

  /** Draw every collider's triangle edges over the map, or stop. */
  setDebug(on: boolean): void {
    this.debug = on;
    for (const entry of this.pieces) {
      if (on) this.showDebug(entry);
      else if (entry.debug) {
        this.world.remove(entry.debug);
        entry.debug = undefined;
      }
    }
  }

  private showDebug(entry: Piece): void {
    if (entry.debug || entry.indices.length === 0) return;
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
