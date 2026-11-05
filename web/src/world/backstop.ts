// The ground under Google's 3D tiles while flying. Google's tiles have no skirts: where
// neighbouring tiles differ in detail (flat water most of all) a slit opens between them. The
// globe stays drawn under them, flattened to BACKSTOP_M, so a slit shows the imagery far below
// rather than space.

import * as C from "cesium";

/** Where the globe is flattened to, m above the ellipsoid: below any sea on Earth (the geoid is at
 * most 106 m under the ellipsoid), so it never rises through the tiles. */
export const BACKSTOP_M = -120;

/** Heights along each side of a tile: as many as an ordinary heightmap has. Cesium judges a
 * heightmap's error by this (TerrainProvider.getEstimatedLevelZeroGeometricErrorForAHeightmap),
 * whatever the heights are; with 2, the globe split itself some five levels deeper than usual,
 * 20 to 100 thousand tiles visited each frame, the page at 15 fps (measured 2026-11-04). */
const SAMPLES = 65;

/** The globe's screen-space error while it is the backstop (Cesium's default, 2, in the menus):
 * it shows only through the slits. */
export const BACKSTOP_SSE = 4;

export function backstopTerrain(): C.CustomHeightmapTerrainProvider {
  return new C.CustomHeightmapTerrainProvider({
    width: SAMPLES,
    height: SAMPLES,
    callback: () => new Float32Array(SAMPLES * SAMPLES).fill(BACKSTOP_M),
  });
}
