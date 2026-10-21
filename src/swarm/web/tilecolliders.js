// Physics ground from the 3D tiles on the map: each tile drawn near the car becomes a Rapier
// trimesh collider holding that tile's own triangles, so the car drives on, and runs into,
// exactly what the map shows. The physics world is a local east-north-up frame in metres.
//
// The colliders follow what Cesium draws, level of detail and all: a tile is built when it is
// drawn near the car, and a collider whose tile has since been replaced on screen by a finer
// or coarser one goes once everything replacing it is built, so the ground never has a hole
// and settles with no tile twice. A collider out of view stays until the car is far from it:
// the chase camera does not see beside or behind the car.

const C = Cesium;

/** Build tiles drawn within this distance of the car, metres from the tile's bounds. */
const BUILD_RADIUS_M = 150;
/** Drop colliders this far from the car. */
const DROP_RADIUS_M = 400;
/** Coarser tiles are never built. Google's finest are about 2 m, 33 m across; the coarse ones
 * above them reach whole continents. */
const MAX_GEOMETRIC_ERROR_M = 8;
/** Milliseconds of tile building per frame. A Google tile takes about 2. */
const BUILD_BUDGET_MS = 6;

/**
 * @param {Cesium.Cesium3DTile} tile
 * @returns {Cesium.Cesium3DTile[]} its ancestors, nearest first
 */
function ancestors(tile) {
  const out = [];
  for (let t = tile.parent; t; t = t.parent) out.push(t);
  return out;
}

/** The glTF model of a tile's content, when it has one loaded. Cesium keeps it private. */
function tileModel(tile) {
  const content = tile.contentReady ? tile.content : undefined;
  return content?._model?.ready ? content._model : undefined;
}

export class TileColliders {
  /**
   * @param {Cesium.Viewer} viewer
   * @param {Cesium.Cesium3DTileset} tileset
   * @param {*} rapier the RAPIER module
   * @param {*} world a RAPIER.World in the east-north-up frame at `origin`
   * @param {Cesium.Cartesian3} origin ECEF origin of that frame
   */
  constructor(viewer, tileset, rapier, world, origin) {
    this.viewer = viewer;
    this.R = rapier;
    this.world = world;
    this.fixedToLocal = C.Matrix4.inverseTransformation(
      C.Transforms.eastNorthUpToFixedFrame(origin),
      new C.Matrix4(),
    );
    /** Per built tile: its collider, and its triangles in the local frame for the debug view.
     * @type {Map<Cesium.Cesium3DTile, {collider: any, triangles: number, vertices: Float32Array,
     *   indices: Uint32Array, debug?: Cesium.Primitive}>} */
    this.built = new Map();
    /** Tiles drawn in the current frame. */
    this._drawn = new Set();
    /** Tiles drawn in the last finished frame. */
    this.drawn = new Set();
    /** The car's position, local frame. */
    this.focus = new C.Cartesian3();
    this.debug = false;
    this.stats = { tiles: 0, triangles: 0, buildMs: 0, pending: 0 };
    this._unlisten = [
      tileset.tileVisible.addEventListener((tile) => this._drawn.add(tile)),
      viewer.scene.postRender.addEventListener(() => this._frame()),
    ];
  }

  /** Stop following the map and take the debug lines off it. The colliders go with the world. */
  destroy() {
    for (const off of this._unlisten) off();
    this.setDebug(false);
    this.built.clear();
  }

  /** The tiles drawn this frame that the car needs and that are not built yet, nearest first. */
  _wanted() {
    const wanted = [];
    for (const tile of this.drawn) {
      if (this.built.has(tile) || tile.geometricError > MAX_GEOMETRIC_ERROR_M) continue;
      if (!tileModel(tile)) continue;
      const d = this._distance(tile);
      // A tile drawn in place of a built one is wanted further out, so the built one can go.
      const replacing = d < DROP_RADIUS_M && ancestors(tile).some((a) => this.built.has(a));
      if (d < BUILD_RADIUS_M || replacing) wanted.push([d, tile]);
    }
    return wanted.sort((a, b) => a[0] - b[0]).map(([, tile]) => tile);
  }

  /** Distance from the car to a tile's bounding sphere, metres (0 inside it). */
  _distance(tile) {
    const sphere = tile.boundingSphere;
    const centre = C.Matrix4.multiplyByPoint(this.fixedToLocal, sphere.center, scratchCentre);
    return Math.max(0, C.Cartesian3.distance(centre, this.focus) - sphere.radius);
  }

  _frame() {
    [this.drawn, this._drawn] = [this._drawn, this.drawn];
    this._drawn.clear();
    const wanted = this._wanted();
    const start = performance.now();
    let i = 0;
    for (; i < wanted.length && performance.now() - start < BUILD_BUDGET_MS; i++) {
      this._build(wanted[i]);
    }
    const pending = wanted.slice(i);
    this._prune(pending);
    this.stats.pending = pending.length;
    this.stats.buildMs = performance.now() - start;
  }

  /**
   * Drop colliders far from the car, and colliders whose tile is no longer drawn because a
   * built tile related to it (an ancestor or a descendant) is, once no tile that is still to
   * be built is related to it either.
   */
  _prune(pending) {
    const drawnBuilt = new Set();
    const aboveDrawnBuilt = new Set();
    for (const tile of this.built.keys()) {
      if (!this.drawn.has(tile)) continue;
      drawnBuilt.add(tile);
      for (const a of ancestors(tile)) aboveDrawnBuilt.add(a);
    }
    const pendingSet = new Set(pending);
    const abovePending = new Set();
    for (const tile of pending) for (const a of ancestors(tile)) abovePending.add(a);
    for (const tile of [...this.built.keys()]) {
      if (this._distance(tile) > DROP_RADIUS_M) {
        this._drop(tile);
        continue;
      }
      if (this.drawn.has(tile) || abovePending.has(tile)) continue;
      const up = ancestors(tile);
      if (up.some((a) => pendingSet.has(a))) continue;
      const replaced = aboveDrawnBuilt.has(tile) || up.some((a) => drawnBuilt.has(a));
      if (replaced) this._drop(tile);
    }
  }

  /** Read a tile's triangles back from the GPU into a trimesh collider. */
  _build(tile) {
    const model = tileModel(tile);
    const parts = [];
    let vertexCount = 0;
    C.ModelReader.forEachPrimitive(model, {}, (runtimePrimitive, primitive, instances) => {
      if (primitive.primitiveType !== C.PrimitiveType.TRIANGLES || !primitive.indices) return;
      const position = primitive.attributes.find((a) => a.semantic === "POSITION");
      if (!position) return;
      const vertices = C.ModelReader.readAttributeAsTypedArray(position);
      const indices = C.ModelReader.readIndicesAsTypedArray(primitive.indices);
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
    const toLocal = new C.Matrix4();
    for (const p of parts) {
      // Vertex -> ECEF -> local, in doubles; float32 is ample once local.
      C.Matrix4.multiply(this.fixedToLocal, p.transform, toLocal);
      const m = toLocal;
      const v = p.vertices;
      for (let k = 0; k < v.length; k += 3) {
        const x = v[k];
        const y = v[k + 1];
        const z = v[k + 2];
        vertices[iv++] = m[0] * x + m[4] * y + m[8] * z + m[12];
        vertices[iv++] = m[1] * x + m[5] * y + m[9] * z + m[13];
        vertices[iv++] = m[2] * x + m[6] * y + m[10] * z + m[14];
      }
      for (let k = 0; k < p.indices.length; k++) indices[ii++] = p.indices[k] + p.base;
    }
    const triangles = indices.length / 3;
    let collider = null;
    if (triangles > 0) {
      collider = this.world.createCollider(this.R.ColliderDesc.trimesh(vertices, indices).setFriction(0.9));
    }
    const entry = { collider, triangles, vertices, indices };
    this.built.set(tile, entry);
    this.stats.tiles = this.built.size;
    this.stats.triangles += triangles;
    if (this.debug) this._showDebug(entry);
  }

  _drop(tile) {
    const entry = this.built.get(tile);
    if (entry.collider) this.world.removeCollider(entry.collider, false);
    if (entry.debug) this.viewer.scene.primitives.remove(entry.debug);
    this.built.delete(tile);
    this.stats.tiles = this.built.size;
    this.stats.triangles -= entry.triangles;
  }

  /** Draw every collider's triangle edges over the map, or stop. */
  setDebug(on) {
    this.debug = on;
    for (const entry of this.built.values()) {
      if (on) this._showDebug(entry);
      else if (entry.debug) {
        this.viewer.scene.primitives.remove(entry.debug);
        entry.debug = undefined;
      }
    }
  }

  _showDebug(entry) {
    if (entry.debug || entry.triangles === 0) return;
    const { vertices, indices } = entry;
    const lines = new Uint32Array(indices.length * 2);
    for (let t = 0, j = 0; t < indices.length; t += 3) {
      const a = indices[t];
      const b = indices[t + 1];
      const c = indices[t + 2];
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
      },
      indices: lines,
      primitiveType: C.PrimitiveType.LINES,
      boundingSphere: C.BoundingSphere.fromVertices(vertices),
    });
    entry.debug = this.viewer.scene.primitives.add(
      new C.Primitive({
        geometryInstances: new C.GeometryInstance({
          geometry,
          attributes: { color: C.ColorGeometryInstanceAttribute.fromColor(DEBUG_COLOR) },
        }),
        appearance: new C.PerInstanceColorAppearance({ flat: true, translucent: true }),
        modelMatrix: C.Matrix4.inverseTransformation(this.fixedToLocal, new C.Matrix4()),
        asynchronous: false,
      }),
    );
  }

  /**
   * The first collider hit by a ray in the local frame, as the distance along it, or null.
   * @param {{x:number,y:number,z:number}} from
   * @param {{x:number,y:number,z:number}} dir unit vector
   */
  castRay(from, dir, maxToi) {
    const ray = new this.R.Ray(from, dir);
    const hit = this.world.castRay(ray, maxToi, true, this.R.QueryFilterFlags.ONLY_FIXED);
    return hit ? hit.timeOfImpact : null;
  }
}

const scratchCentre = new C.Cartesian3();
const DEBUG_COLOR = C.Color.fromCssColorString("#5ee0ff").withAlpha(0.55);
