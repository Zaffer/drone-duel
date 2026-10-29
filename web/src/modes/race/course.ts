// A race course drawn on the map: each gate a frame round its inside (the next one in ember and
// thicker, the ones passed in leaf, the rest pale), and a chevron hovering over the next.

import * as C from "cesium";
import type { Course } from "../../net/protocol.ts";
import { mapOffset, offsetEnu, toFixed } from "../../world/geo.ts";
import type { World } from "../../world/world.ts";
import { corners, type GateGeom, gateGeom } from "./gates.ts";

const NEXT = C.Color.fromCssColorString("#FF6B35");
const PASSED = C.Color.fromCssColorString("#8BE07A").withAlpha(0.7);
const LATER = C.Color.fromCssColorString("#EEF0F7").withAlpha(0.75);

export class CourseView {
  gates: GateGeom[] = [];
  private lines: C.PolylineCollection;
  private marker: C.BillboardCollection;

  constructor(
    private readonly world: World,
    readonly course: Course,
  ) {
    this.lines = world.scene.primitives.add(new C.PolylineCollection());
    // Hidden as a whole until placed (a hidden billboard's place still counts in the frustum).
    this.marker = world.scene.primitives.add(new C.BillboardCollection({ show: false }));
    const canvas = document.createElement("canvas");
    canvas.width = canvas.height = 48;
    const g = canvas.getContext("2d")!;
    g.fillStyle = "#FF6B35";
    g.strokeStyle = "#0D0E15";
    g.lineWidth = 3;
    g.beginPath();
    g.moveTo(8, 10);
    g.lineTo(24, 30);
    g.lineTo(40, 10);
    g.lineTo(40, 20);
    g.lineTo(24, 40);
    g.lineTo(8, 20);
    g.closePath();
    g.stroke();
    g.fill();
    this.marker.add({
      image: canvas,
      scale: 0.6,
      position: C.Cartesian3.ZERO,
      disableDepthTestDistance: Number.POSITIVE_INFINITY,
      show: false,
    });
  }

  /** Place the gates on the map's ground (AGL courses), then draw them. */
  async place(): Promise<void> {
    const out: GateGeom[] = [];
    for (const g of this.course.gates) {
      let alt = g.alt_m;
      if (this.course.alt_ref === "agl") {
        const ground = await this.world.groundHeightSettled(g.at[0], g.at[1], 8);
        alt += ground - mapOffset();
      }
      out.push(gateGeom(g, alt));
    }
    this.gates = out;
  }

  /** Draw with gate `next` (0..n-1) next; `passed` gates of this lap are done. */
  draw(next: number, passedThisLap: number): void {
    this.lines.removeAll();
    this.gates.forEach((g, i) => {
      const centre = toFixed(g.lat, g.lon, g.alt);
      const ring = corners(g).map((c) => offsetEnu(centre, c));
      ring.push(ring[0]!);
      const isNext = i === next;
      const colour = isNext ? NEXT : i < passedThisLap ? PASSED : LATER;
      this.lines.add({
        positions: ring,
        width: isNext ? 7 : 4,
        material: C.Material.fromType("Color", { color: colour }),
      });
      // The posts down to the ground, so a gate reads as standing.
      const drop = this.course.alt_ref === "agl" ? this.course.gates[i]!.alt_m - g.height / 2 : 0;
      for (const k of drop > 0.1 ? [0, 1] : []) {
        const foot = ring[k]!;
        this.lines.add({
          positions: [foot, offsetEnu(foot, [0, 0, -drop])],
          width: 3,
          material: C.Material.fromType("Color", { color: colour.withAlpha(0.6) }),
        });
      }
    });
    const g = this.gates[next];
    const b = this.marker.get(0);
    b.show = g !== undefined;
    this.marker.show = g !== undefined;
    if (g) b.position = offsetEnu(toFixed(g.lat, g.lon, g.alt), [0, 0, g.height / 2 + 1.5]);
  }

  dispose(): void {
    this.world.scene.primitives.remove(this.lines);
    this.world.scene.primitives.remove(this.marker);
  }
}
