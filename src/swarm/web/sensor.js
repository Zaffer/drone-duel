// Camera field of view geometry: the view of the drone's FPV camera, fixed to its frame, as
// the COP draws it and as the FPV view looks through it. Angles in radians; heading clockwise
// from north, pitch nose up, roll right wing down (ArduPilot's ATTITUDE), tilt the camera's
// angle up from the nose.

const C = Cesium;

/** Corners in order top-left, top-right, bottom-right, bottom-left of the image. */
const CORNERS = [
  [-1, 1],
  [1, 1],
  [1, -1],
  [-1, -1],
];

/**
 * The camera's view direction `f`, image right `r` and image up `u` as east-north-up unit
 * vectors, for a camera on a frame at `heading`, `pitch` and `roll`, tilted `tilt` up from
 * the nose.
 * @returns {{f: number[], r: number[], u: number[]}}
 */
export function cameraAxes(heading, pitch, roll, tilt) {
  const sh = Math.sin(heading);
  const ch = Math.cos(heading);
  const nose = [sh * Math.cos(pitch), ch * Math.cos(pitch), Math.sin(pitch)];
  const level = [ch, -sh, 0];
  const top = cross(level, nose);
  // Roll about the nose: the right side goes down, the top leans right.
  const cr = Math.cos(roll);
  const sr = Math.sin(roll);
  const r = level.map((x, i) => x * cr - top[i] * sr);
  const up = top.map((x, i) => x * cr + level[i] * sr);
  // Tilt about the frame's right axis.
  const ct = Math.cos(tilt);
  const st = Math.sin(tilt);
  const f = nose.map((x, i) => x * ct + up[i] * st);
  const u = up.map((x, i) => x * ct - nose[i] * st);
  return { f, r, u };
}

/**
 * The image's corners on the plane `depth` metres ahead of the camera: straight out along the
 * view, whatever the ground does. A line from the apex through each is an edge of the view.
 * @param {Cesium.Cartesian3} apex camera position (ECEF)
 * @param {{f: number[], r: number[], u: number[]}} axes camera axes from cameraAxes
 * @param {number} hfov horizontal field of view
 * @param {number} vfov vertical field of view
 * @param {number} depth metres
 * @returns {Cesium.Cartesian3[]}
 */
export function viewCorners(apex, axes, hfov, vfov, depth) {
  const enu = C.Transforms.eastNorthUpToFixedFrame(apex);
  const { f, r, u } = axes;
  const th = Math.tan(hfov / 2);
  const tv = Math.tan(vfov / 2);
  return CORNERS.map(([sx, sy]) => {
    const d = f.map((x, i) => depth * (x + sx * th * r[i] + sy * tv * u[i]));
    return C.Matrix4.multiplyByPoint(enu, new C.Cartesian3(...d), new C.Cartesian3());
  });
}

/**
 * Cesium's Fade material for a polyline: `css()` at `alpha` where the line starts, fading
 * evenly to nothing where it ends.
 */
export class FadeMaterialProperty {
  /** @param {() => string} css */
  constructor(css, alpha) {
    this.css = css;
    this.alpha = alpha;
    this.definitionChanged = new C.Event();
    this._colors = new Map();
  }

  get isConstant() {
    return false;
  }

  getType() {
    return "Fade";
  }

  getValue(time, result = {}) {
    const css = this.css();
    let colors = this._colors.get(css);
    if (!colors) {
      const c = C.Color.fromCssColorString(css);
      colors = [c.withAlpha(this.alpha), c.withAlpha(0)];
      this._colors.set(css, colors);
    }
    [result.fadeInColor, result.fadeOutColor] = colors;
    result.maximumDistance = 1;
    result.repeat = false;
    result.fadeDirection = FADE_ALONG;
    result.time = FADE_FROM_START;
    return result;
  }

  equals(other) {
    return this === other;
  }
}

/** Fade along the polyline (st.s), from its start, not across its width. */
const FADE_ALONG = { x: true, y: false };
const FADE_FROM_START = new C.Cartesian2(0, 0);

function cross(a, b) {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}
