// The local frame (DESIGN.md §3.2): metres east, north and up from a point on the map, the
// tangent plane there (Cesium's east-north-up). Everything that moves or collides in the page
// works in one of these (the ground's colliders, the truck, the shooter, the rounds) and turns
// to the earth-fixed frame only to draw.
//
// Heights: up is from the origin's map height (above the WGS84 ellipsoid). The sim's heights are
// above sea level: sim = map - geo.mapOffset (DESIGN.md §3). `toGeo` gives both.

import * as C from "cesium";
import { mapOffset, type Vec3 } from "../world/geo.ts";

export interface Geo {
  lat: number;
  lon: number;
  /** Map height, m above the ellipsoid. */
  h: number;
  /** Sim height, m above sea level. */
  alt: number;
}

const scratchFixed = new C.Cartesian3();
const scratchLocal = new C.Cartesian3();
const scratchCarto = new C.Cartographic();

export class LocalFrame {
  /** Local to earth-fixed, and back. */
  readonly toFixedM: C.Matrix4;
  readonly toLocalM: C.Matrix4;

  /** At (`lat`, `lon`), map height `h`. */
  constructor(
    readonly lat: number,
    readonly lon: number,
    readonly h: number,
  ) {
    this.toFixedM = C.Transforms.eastNorthUpToFixedFrame(C.Cartesian3.fromDegrees(lon, lat, h));
    this.toLocalM = C.Matrix4.inverseTransformation(this.toFixedM, new C.Matrix4());
  }

  /** A local point, earth-fixed. */
  fixed(p: Vec3, result = new C.Cartesian3()): C.Cartesian3 {
    scratchLocal.x = p[0];
    scratchLocal.y = p[1];
    scratchLocal.z = p[2];
    return C.Matrix4.multiplyByPoint(this.toFixedM, scratchLocal, result);
  }

  /** An earth-fixed point, local. */
  local(c: C.Cartesian3): Vec3 {
    const p = C.Matrix4.multiplyByPoint(this.toLocalM, c, scratchLocal);
    return [p.x, p.y, p.z];
  }

  /** A point at map height `h`, local. */
  fromMap(lat: number, lon: number, h: number): Vec3 {
    return this.local(C.Cartesian3.fromDegrees(lon, lat, h, C.Ellipsoid.WGS84, scratchFixed));
  }

  /** A point at sim height `alt` (a pose's), local. */
  fromSim(lat: number, lon: number, alt: number): Vec3 {
    return this.fromMap(lat, lon, alt + mapOffset());
  }

  /** A local point on the map, and its sim height. */
  toGeo(p: Vec3): Geo {
    const c = C.Cartographic.fromCartesian(
      this.fixed(p, scratchFixed),
      C.Ellipsoid.WGS84,
      scratchCarto,
    );
    return {
      lat: C.Math.toDegrees(c.latitude),
      lon: C.Math.toDegrees(c.longitude),
      h: c.height,
      alt: c.height - mapOffset(),
    };
  }

  /** A local direction, earth-fixed. */
  fixedDir(v: Vec3, result = new C.Cartesian3()): C.Cartesian3 {
    scratchLocal.x = v[0];
    scratchLocal.y = v[1];
    scratchLocal.z = v[2];
    return C.Matrix4.multiplyByPointAsVector(this.toFixedM, scratchLocal, result);
  }
}
