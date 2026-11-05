import { expect, test } from "bun:test";
import * as C from "cesium";
import { BACKSTOP_M, backstopTerrain } from "../src/world/backstop.ts";

test("the backstop is judged no coarser than an ordinary heightmap, so the globe under the tiles is not split deeper", () => {
  // Cesium's own estimate for a 65-sample heightmap on the geographic tiling (2 tiles at level 0).
  const ordinary = C.TerrainProvider.getEstimatedLevelZeroGeometricErrorForAHeightmap(
    C.Ellipsoid.WGS84,
    65,
    2,
  );
  expect(backstopTerrain().getLevelMaximumGeometricError(0)).toBeLessThanOrEqual(ordinary);
});

test("the backstop is flat at BACKSTOP_M", async () => {
  const data = await backstopTerrain().requestTileGeometry(0, 0, 0);
  expect(data).toBeInstanceOf(C.HeightmapTerrainData);
  expect(data!.interpolateHeight(C.Rectangle.MAX_VALUE, 0, 0)).toBeCloseTo(BACKSTOP_M);
});
