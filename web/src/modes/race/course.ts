// A race course drawn on the map: each gate a solid square frame (the course's GateFrame) on posts
// to the ground. The next gate is ember with a chevron on each side pointing in, the one after it
// half ember with a dot on each side, the one after that a quarter ember; the rest pale, those
// passed this lap leaf.

import * as C from "cesium";
import type { Course } from "../../net/protocol.ts";
import { enuToFixed, mapOffset, offsetEnu, toFixed } from "../../world/geo.ts";
import type { World } from "../../world/world.ts";
import { type GateGeom, gateGeom, inPlane } from "./gates.ts";

const NEXT = C.Color.fromCssColorString("#FF6B35");
const PASSED = C.Color.fromCssColorString("#8BE07A");
const LATER = C.Color.fromCssColorString("#EEF0F7");
/** The next gate's colour, then the two after it: a half and a quarter of the way from the rest's
 * to it, so the order reads at a glance. */
const SOON = [
  NEXT,
  C.Color.lerp(LATER, NEXT, 0.5, new C.Color()),
  C.Color.lerp(LATER, NEXT, 0.25, new C.Color()),
];
/** The markers' gap outside the frame's edges, m. */
const MARK_GAP_M = 0.5;
/** A marker's side (right, up, as a unit) and its turn so a chevron points in at the gate. */
const SIDES: [number, number, number][] = [
  [0, 1, 0],
  [0, -1, Math.PI],
  [-1, 0, Math.PI / 2],
  [1, 0, -Math.PI / 2],
];
/** Markers keep their pixel size near and shrink to this far off, so they stay round their gate. */
const MARK_SCALE = new C.NearFarScalar(10, 1, 150, 0.4);

export class CourseView {
  gates: GateGeom[] = [];
  private lines: C.PolylineCollection;
  private frames: C.Primitive | null = null;
  /** Four chevrons round the next gate, then four dots round the one after it. */
  private marker: C.BillboardCollection;

  constructor(
    private readonly world: World,
    readonly course: Course,
  ) {
    this.lines = world.scene.primitives.add(new C.PolylineCollection());
    // Hidden as a whole until placed (a hidden billboard's place still counts in the frustum).
    this.marker = world.scene.primitives.add(new C.BillboardCollection({ show: false }));
    const chevron = canvas(48, (g) => {
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
    });
    const dot = canvas(24, (g) => {
      g.fillStyle = SOON[1]!.toCssColorString();
      g.strokeStyle = "#0D0E15";
      g.lineWidth = 3;
      g.beginPath();
      g.arc(12, 12, 8, 0, 2 * Math.PI);
      g.stroke();
      g.fill();
    });
    for (const [image, scale] of [
      [chevron, 0.6],
      [dot, 0.6],
    ] as const) {
      for (const _ of SIDES) {
        this.marker.add({
          image,
          scale,
          scaleByDistance: MARK_SCALE,
          position: C.Cartesian3.ZERO,
          disableDepthTestDistance: Number.POSITIVE_INFINITY,
          show: false,
        });
      }
    }
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
      out.push(gateGeom(g, this.course.gate, alt));
    }
    this.gates = out;
  }

  /** Draw with `upcoming` (gate indices, soonest first) the gates to fly next; the first
   * `passedThisLap` gates are done this lap. */
  draw(upcoming: readonly number[], passedThisLap: number): void {
    this.lines.removeAll();
    const boxes: C.GeometryInstance[] = [];
    this.gates.forEach((g, i) => {
      const soon = upcoming.indexOf(i);
      const colour = soon >= 0 ? SOON[soon]! : i < passedThisLap ? PASSED : LATER;
      boxes.push(...frame(g, colour));
      // The posts down to the ground, under the frame's sides, so a gate reads as standing.
      const drop =
        this.course.alt_ref === "agl" ? this.course.gates[i]!.alt_m - g.outerHeight / 2 : 0;
      if (drop <= 0.1) return;
      const centre = toFixed(g.lat, g.lon, g.alt);
      const x = (g.outerWidth + g.width) / 4;
      for (const side of [-x, x]) {
        const foot = offsetEnu(centre, inPlane(g, side, -g.outerHeight / 2));
        this.lines.add({
          positions: [foot, offsetEnu(foot, [0, 0, -drop])],
          width: 3,
          material: C.Material.fromType("Color", { color: colour.withAlpha(0.8) }),
        });
      }
    });
    if (this.frames) this.world.scene.primitives.remove(this.frames);
    this.frames = boxes.length
      ? this.world.scene.primitives.add(
          new C.Primitive({
            geometryInstances: boxes,
            appearance: new C.PerInstanceColorAppearance({ translucent: false }),
            // Drawn in this frame, so a gate passed changes colour at once.
            asynchronous: false,
          }),
        )
      : null;
    this.mark(this.gates[upcoming[0] ?? -1], 0);
    this.mark(this.gates[upcoming[1] ?? -1], SIDES.length);
    this.marker.show = upcoming.length > 0;
  }

  /** The markers from `first` round gate `g`, or hidden without one. */
  private mark(g: GateGeom | undefined, first: number): void {
    const centre = g ? toFixed(g.lat, g.lon, g.alt) : null;
    const axis =
      g && centre ? C.Cartesian3.normalize(enuToFixed(centre, g.up), new C.Cartesian3()) : null;
    SIDES.forEach(([across, up, rotation], k) => {
      const b = this.marker.get(first + k);
      b.show = g !== undefined;
      if (!g || !centre || !axis) return;
      b.position = offsetEnu(
        centre,
        inPlane(g, across * (g.outerWidth / 2 + MARK_GAP_M), up * (g.outerHeight / 2 + MARK_GAP_M)),
      );
      // Turned against the gate's up, so the chevrons point in however the view is rolled.
      b.alignedAxis = axis;
      b.rotation = rotation;
    });
  }

  dispose(): void {
    if (this.frames) this.world.scene.primitives.remove(this.frames);
    this.world.scene.primitives.remove(this.lines);
    this.world.scene.primitives.remove(this.marker);
  }
}

/** A gate's frame: four bars round its inside, `depth` thick, as boxes in the gate's own axes. */
function frame(g: GateGeom, colour: C.Color): C.GeometryInstance[] {
  const enu = C.Transforms.eastNorthUpToFixedFrame(toFixed(g.lat, g.lon, g.alt));
  const { right: r, fwd: f, up: u } = g;
  // biome-ignore format: a matrix
  const axes = C.Matrix4.fromRowMajorArray([
    r[0], f[0], u[0], 0,
    r[1], f[1], u[1], 0,
    r[2], f[2], u[2], 0,
    0, 0, 0, 1,
  ]);
  const gate = C.Matrix4.multiply(enu, axes, new C.Matrix4());
  const barW = (g.outerWidth - g.width) / 2;
  const barH = (g.outerHeight - g.height) / 2;
  // Each bar: its centre right and up of the gate's, its width and height.
  const bars: [number, number, number, number][] = [
    [0, (g.height + barH) / 2, g.outerWidth, barH],
    [0, -(g.height + barH) / 2, g.outerWidth, barH],
    [-(g.width + barW) / 2, 0, barW, g.height],
    [(g.width + barW) / 2, 0, barW, g.height],
  ];
  const color = C.ColorGeometryInstanceAttribute.fromColor(colour);
  return bars.map(
    ([x, z, w, h]) =>
      new C.GeometryInstance({
        geometry: C.BoxGeometry.fromDimensions({
          dimensions: new C.Cartesian3(w, g.depth, h),
          vertexFormat: C.PerInstanceColorAppearance.VERTEX_FORMAT,
        }),
        modelMatrix: C.Matrix4.multiplyByTranslation(
          gate,
          new C.Cartesian3(x, 0, z),
          new C.Matrix4(),
        ),
        attributes: { color },
      }),
  );
}

function canvas(size: number, paint: (g: CanvasRenderingContext2D) => void): HTMLCanvasElement {
  const c = document.createElement("canvas");
  c.width = c.height = size;
  paint(c.getContext("2d")!);
  return c;
}
