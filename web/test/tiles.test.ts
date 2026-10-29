import { describe, expect, test } from "bun:test";
import {
  clampToEdge,
  metresPerPx,
  niceLength,
  projector,
  TILE,
  tileUrl,
  visibleTiles,
  worldPx,
  zoomFor,
} from "../src/hud/tiles.ts";

describe("Web Mercator", () => {
  test("the world's corners and centre", () => {
    expect(worldPx(0, 0, 0)).toEqual([128, 128]);
    const [x, y] = worldPx(85.05112878, -180, 1);
    expect(x).toBe(0);
    expect(y).toBeCloseTo(0, 6);
  });

  test("a known tile: Toronto's at zoom 16", () => {
    // Billy Bishop airport (43.63093, -79.38945): tile x 18315, y 23923 at z16 (OSM's
    // slippy-map formula, asinh(tan φ)).
    const [x, y] = worldPx(43.63093, -79.38945, 16);
    expect(Math.floor(x / TILE)).toBe(18315);
    expect(Math.floor(y / TILE)).toBe(23923);
  });

  test("metres per pixel halve with each zoom", () => {
    expect(metresPerPx(0, 0)).toBeCloseTo(156543.034, 2);
    expect(metresPerPx(45, 16) / metresPerPx(45, 17)).toBeCloseTo(2);
  });
});

describe("the minimap's zoom", () => {
  test("400 m across 200 px: tile pixels no coarser than the screen's", () => {
    for (const dpr of [1, 2]) {
      const z = zoomFor(43.63, 400, 200, dpr);
      expect(metresPerPx(43.63, z)).toBeLessThanOrEqual(400 / (200 * dpr));
      expect(metresPerPx(43.63, z - 1)).toBeGreaterThan(400 / (200 * dpr));
    }
    expect(zoomFor(43.63, 400, 200, 1)).toBe(16);
    expect(zoomFor(43.63, 400, 200, 2)).toBe(17);
  });

  test("capped at the imagery's deepest zoom", () => {
    expect(zoomFor(0, 1, 1000, 3)).toBe(19);
  });
});

describe("tiles in view", () => {
  test("a 200 px view round a tile corner takes the four tiles there", () => {
    const tiles = visibleTiles(512, 512, 100, 100, 3);
    expect(tiles.map((t) => [t.x, t.y])).toEqual([
      [1, 1],
      [2, 1],
      [1, 2],
      [2, 2],
    ]);
    expect(tiles[0]).toMatchObject({ dx: -256, dy: -256 });
  });

  test("x wraps round the antimeridian; y stops at the poles", () => {
    const tiles = visibleTiles(10, 10, 100, 100, 2);
    expect(tiles.map((t) => t.x)).toEqual([3, 0]);
    expect(tiles.every((t) => t.y === 0)).toBe(true);
  });

  test("Esri's URL is z/y/x", () => {
    expect(tileUrl(16, 18315, 23923)).toEndWith("/tile/16/23923/18315");
  });
});

describe("projection", () => {
  test("north is up, east is right, scaled from the centre", () => {
    const p = projector(43.63, -79.39, 16, 1);
    expect(p(43.63, -79.39)).toEqual([0, 0]);
    const [ex, ey] = p(43.63, -79.389);
    expect(ex).toBeGreaterThan(0);
    expect(Math.abs(ey)).toBeLessThan(1e-6);
    const [nx, ny] = p(43.631, -79.39);
    expect(ny).toBeLessThan(0);
    expect(Math.abs(nx)).toBeLessThan(1e-6);
    // 100 m east is 100 m / metresPerPx map px.
    const east = (100 / (6378137 * Math.cos((43.63 * Math.PI) / 180))) * (180 / Math.PI);
    expect(p(43.63, -79.39 + east)[0]).toBeCloseTo(100 / metresPerPx(43.63, 16), 0);
  });

  test("off the map, pinned to the rim", () => {
    expect(clampToEdge(10, 0, 100, true)).toEqual({ x: 10, y: 0, clamped: false });
    const r = clampToEdge(300, 400, 100, true, 10);
    expect(Math.hypot(r.x, r.y)).toBeCloseTo(90);
    expect(r.clamped).toBe(true);
    const s = clampToEdge(300, 150, 100, false);
    expect(s).toEqual({ x: 100, y: 50, clamped: true });
  });

  test("a scale bar's round length", () => {
    expect(niceLength(130)).toBe(100);
    expect(niceLength(260)).toBe(200);
    expect(niceLength(999)).toBe(500);
    expect(niceLength(0)).toBe(0);
  });
});
