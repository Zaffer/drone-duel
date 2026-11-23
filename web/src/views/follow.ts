// Where the chase cameras sit (DESIGN.md §11): behind the vehicle's nose, wherever it is going.
// A quad sliding sideways or backing up is seen from behind, as its pilot would turn their own
// head, and a yaw swings the camera round on a spring, so the swing starts and ends gently,
// by the shorter way.

import { Spring } from "../engine/spring.ts";
import type { Quat } from "../world/geo.ts";

/** How fast the camera comes round, Hz: critically damped, half a second behind a steady yaw. */
const FOLLOW_HZ = 0.6;
/** The nose's length on the ground (of 1) under which it says no heading: within about 17° of
 * straight up or down, where a flip or a loop would spin the camera. */
const LEVEL_ENOUGH = 0.3;

const wrap = (a: number): number => Math.atan2(Math.sin(a), Math.cos(a));

export class NoseFollow {
  private readonly spring = new Spring(FOLLOW_HZ, 1);
  private started = false;

  /** Start again behind the next nose given. */
  reset(): void {
    this.started = false;
  }

  /** The heading the camera sits behind (radians clockwise from north, not wrapped), `dt` s on,
   * the vehicle's attitude now `q` (body to north-east-down). */
  step(q: Quat, dt: number): number {
    const [w, x, y, z] = q;
    const north = 1 - 2 * (y * y + z * z);
    const east = 2 * (w * z + x * y);
    const s = this.spring;
    if (!this.started) {
      s.x = s.target = Math.atan2(east, north);
      s.v = 0;
      this.started = true;
    } else if (Math.hypot(north, east) > LEVEL_ENOUGH) {
      // The short way round from where the camera is: a spin faster than it follows is not
      // chased all the way round, the camera meets the nose from the nearer side.
      s.target = s.x + wrap(Math.atan2(east, north) - s.x);
    }
    s.step(dt);
    return s.x;
  }
}
