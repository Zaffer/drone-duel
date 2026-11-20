// The page's quad model (src/sim/quad.ts, on the simulator's generated C plant) against the
// game's own (drone_duels.physics.quad.model, on SkyFlow-Dynamics' JAX backend): the reference
// flight Python flew (fixtures/sim-flight.json, drone_duels.webdata.reference_flight), flown
// again here, frame for frame. Needs the simulator built (scripts/setup-web.sh); skipped
// without it.

import { describe, expect, test } from "bun:test";
import { Sim } from "../src/sim/module.ts";
import { type Airframe, cellVoltage, QuadModel } from "../src/sim/quad.ts";
import data from "./fixtures/sim-data.json";
import flight from "./fixtures/sim-flight.json";
import { simModule } from "./sim-module.ts";

const module = await simModule();
const airframe = (data.sim.airframes as Record<string, Airframe>)[flight.airframe]!;

function near(got: ArrayLike<number>, want: number[], tol: number, what: string): void {
  for (let i = 0; i < want.length; i++) {
    if (Math.abs(got[i]! - want[i]!) > tol) {
      throw new Error(`${what}[${i}]: ${got[i]} is not ${want[i]} (within ${tol})`);
    }
  }
}

describe.skipIf(!module)("the page's quad model", () => {
  test("flies Python's reference flight: the pad, up, a push on each axis, down, stopped", () => {
    const sim = new Sim(module!);
    const origin = flight.origin as [number, number, number];
    const model = new QuadModel(sim, airframe, origin, flight.heading_deg, {}, 0);
    const pose = new Float64Array(18);
    let frame = 0;
    let at = 0;
    const phases = new Set<string>();
    for (const [frames, pwm] of flight.script as [number, number[]][]) {
      for (let i = 0; i < frames; i++) {
        if (frame === flight.ground.frame) model.setGround(flight.ground.alt);
        const fdm = model.step(0.001, pwm);
        frame++;
        const want = flight.samples[at];
        if (!want || want.frame !== frame) continue;
        at++;
        const ctx = `frame ${frame}`;
        expect(model.phase, ctx).toBe(want.phase as typeof model.phase);
        phases.add(model.phase);
        // The two backends differ in the last bits of each step; seven seconds of them, with a
        // landing between, stay within a micrometre.
        near(fdm.position, want.position, 1e-6, `${ctx} position`);
        near(fdm.velocity, want.velocity, 1e-6, `${ctx} velocity`);
        near(fdm.quat, want.quat, 1e-7, `${ctx} quat`);
        near(fdm.gyro, want.gyro, 1e-6, `${ctx} gyro`);
        near(fdm.accel, want.accel, 1e-5, `${ctx} accel`);
        near([fdm.batteryV, fdm.batteryA], [want.battery_v, want.battery_a], 1e-7, `${ctx} pack`);
        expect(model.pose(pose)).toBe(true);
        near(pose.subarray(0, 4), want.pose.slice(0, 4), 1e-9, `${ctx} pose t, lat, lon, alt`);
        near(pose.subarray(14), want.pose.slice(14), 1e-5, `${ctx} pose rotors`);
      }
    }
    expect(at).toBe(flight.samples.length);
    expect([...phases].sort()).toEqual(["flying", "ground"]);
  });

  test("a reset puts it back on the pad with a fresh pack", () => {
    const sim = new Sim(module!);
    const model = new QuadModel(sim, airframe, [43.63, -79.39, 77], 90, {}, 0);
    const rest = model.step(0.001, [1000, 1000, 1000, 1000]).position[2];
    for (let i = 0; i < 1500; i++) model.step(0.001, [1500, 1500, 1500, 1500]);
    expect(model.phase).toBe("flying");
    expect(model.last!.position[2]).toBeLessThan(-1);
    expect(model.battery.usedMah).toBeGreaterThan(0);
    model.reset();
    expect(model.phase).toBe("ground");
    expect(model.battery.usedMah).toBe(0);
    expect(model.step(0.001, [1000, 1000, 1000, 1000]).position[2]).toBeCloseTo(rest, 9);
  });

  test("into the ground fast, it has crashed, and its motors stop whatever is asked", () => {
    const sim = new Sim(module!);
    const model = new QuadModel(sim, airframe, [43.63, -79.39, 77], 0, {}, 0);
    for (let i = 0; i < 2500; i++) model.step(0.001, [1700, 1700, 1700, 1700]);
    expect(model.last!.position[2]).toBeLessThan(-15);
    for (let i = 0; i < 6000 && model.phase === "flying"; i++) {
      model.step(0.001, [1000, 1000, 1000, 1000]);
    }
    expect(model.phase).toBe("crashed");
    for (let i = 0; i < 500; i++) model.step(0.001, [2000, 2000, 2000, 2000]);
    expect(model.phase).toBe("crashed");
    const pose = new Float64Array(18);
    model.pose(pose);
    expect(Math.max(...pose.subarray(14))).toBeLessThan(1);
  });
});

test("the pack's cell: full at 4.2 V, empty at 3.27 V, between by the chart", () => {
  expect(cellVoltage(1)).toBeCloseTo(4.2, 12);
  expect(cellVoltage(0)).toBeCloseTo(3.27, 12);
  expect(cellVoltage(0.45)).toBeCloseTo(3.82, 12);
  expect(cellVoltage(2)).toBeCloseTo(4.2, 12);
});
