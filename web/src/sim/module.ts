// The simulator's WebAssembly module (DESIGN.md §18): Betaflight (cudaflight's SITL_LOCKSTEP
// target, web/wasm/bfweb.c) and the quad's plant (SkyFlow-Dynamics' generated C,
// web/wasm/plant.c), built by scripts/setup-web.sh. One instance is one quad: its flight
// controller on a virtual clock that runs only inside `step`, and its airframe's state.
//
// The module is standalone (no JavaScript glue): it imports a handful of WASI calls, answered
// here. What the firmware prints goes to `log`, a line at a time. The firmware ending itself
// (a reboot asked for, a failure mode: exit() on this target) throws FirmwareExit.

/** The firmware called exit(): `code` 0 for a reboot, 1 for a failure mode. */
export class FirmwareExit extends Error {
  constructor(readonly code: number) {
    super(`the firmware stopped (exit ${code})`);
  }
}

interface Exports {
  memory: WebAssembly.Memory;
  _initialize?: () => void;
  bfw_io: () => number;
  bfw_io_size: () => number;
  bfw_boot: () => void;
  bfw_eeprom: () => number;
  bfw_eeprom_size: () => number;
  bfw_cli: (len: number) => number;
  bfw_step: (us: number) => void;
  bfw_micros: () => number;
  bfw_set_imu: (p: number, q: number, r: number, fx: number, fy: number, fz: number) => void;
  bfw_set_air: (alt: number, qw: number, qx: number, qy: number, qz: number) => void;
  bfw_set_gps: (lat: number, lon: number, alt: number, vn: number, ve: number, vd: number) => void;
  bfw_rc: () => number;
  bfw_set_rc: (count: number) => void;
  bfw_motors: () => number;
  bfw_armed: () => number;
  bfw_arming_disabled: () => number;
  bfw_serial_write: (uart: number, len: number) => number;
  bfw_serial_read: (uart: number) => number;
  plant_state: () => number;
  plant_inputs: () => number;
  plant_params: () => number;
  plant_imu: () => number;
  plant_sizes: (which: number) => number;
  plant_advance: (dt: number, steps: number, rotorMax: number) => void;
  plant_throttle_to_speed: (u: number, wMin: number, wMax: number, k: number) => number;
}

const RC_CHANNELS = 16;
const ERRNO_BADF = 8;
const ERRNO_SPIPE = 70;

/** One instance of the module: a flight controller and a plant. */
export class Sim {
  private readonly x: Exports;
  private line = "";
  private readonly decoder = new TextDecoder();

  /** The plant's sizes: rotors, state, inputs, parameters (sfd.h). */
  readonly rotors: number;
  readonly stateSize: number;
  readonly inputSize: number;
  readonly paramSize: number;

  constructor(
    module: WebAssembly.Module,
    private readonly log: (line: string) => void = () => {},
  ) {
    const wasi: Record<string, (...args: number[]) => number> = {
      fd_write: (_fd, iov, count, written) => {
        const view = new DataView(this.x.memory.buffer);
        let total = 0;
        for (let i = 0; i < count; i++) {
          const at = view.getUint32(iov + 8 * i, true);
          const len = view.getUint32(iov + 8 * i + 4, true);
          this.print(new Uint8Array(this.x.memory.buffer, at, len));
          total += len;
        }
        view.setUint32(written, total, true);
        return 0;
      },
      fd_close: () => 0,
      fd_read: () => ERRNO_BADF,
      fd_seek: () => ERRNO_SPIPE,
      proc_exit: (code) => {
        throw new FirmwareExit(code);
      },
    };
    // Whatever else a build's libc asks for (it differs by Emscripten version): nothing here
    // has files, a clock or an environment, so each answers "no".
    const imports: WebAssembly.Imports = {};
    for (const want of WebAssembly.Module.imports(module)) {
      const known = want.module === "wasi_snapshot_preview1" ? wasi[want.name] : undefined;
      const answer = known ?? (want.name === "emscripten_notify_memory_growth" ? () => {} : no);
      (imports[want.module] ??= {})[want.name] = answer;
    }
    this.x = new WebAssembly.Instance(module, imports).exports as unknown as Exports;
    this.x._initialize?.();
    this.rotors = this.x.plant_sizes(0);
    this.stateSize = this.x.plant_sizes(1);
    this.inputSize = this.x.plant_sizes(2);
    this.paramSize = this.x.plant_sizes(3);
  }

  private print(bytes: Uint8Array): void {
    this.line += this.decoder.decode(bytes, { stream: true });
    const lines = this.line.split("\n");
    this.line = lines.pop() ?? "";
    for (const l of lines) this.log(l);
  }

  private bytes(at: number, length: number): Uint8Array {
    return new Uint8Array(this.x.memory.buffer, at, length);
  }

  private doubles(at: number, length: number): Float64Array {
    return new Float64Array(this.x.memory.buffer, at, length);
  }

  // The firmware.

  /** Boot Betaflight on `eeprom` (an image from `eeprom()`), or on its defaults. */
  boot(eeprom?: Uint8Array): void {
    if (eeprom) {
      if (eeprom.length !== this.x.bfw_eeprom_size()) {
        throw new Error("the eeprom image is another build's");
      }
      this.bytes(this.x.bfw_eeprom(), eeprom.length).set(eeprom);
    }
    this.x.bfw_boot();
  }

  /** The eeprom as it is now: a copy. */
  eeprom(): Uint8Array {
    return this.bytes(this.x.bfw_eeprom(), this.x.bfw_eeprom_size()).slice();
  }

  /** Type CLI `text` in and save it to the eeprom. Throws on a line Betaflight rejects. */
  cli(text: string): void {
    const bytes = new TextEncoder().encode(text);
    if (bytes.length > this.x.bfw_io_size()) throw new Error("the settings are too long");
    this.bytes(this.x.bfw_io(), bytes.length).set(bytes);
    const errors = this.x.bfw_cli(bytes.length);
    if (errors > 0) {
      const io = this.bytes(this.x.bfw_io(), 512);
      const first = new TextDecoder().decode(io.subarray(0, io.indexOf(0)));
      throw new Error(`Betaflight rejected its settings (${errors}): ${first.replace(/#/g, "")}`);
    }
  }

  /** The firmware on by `us` µs of its own clock. */
  step(us: number): void {
    this.x.bfw_step(us);
  }

  /** The firmware's clock, µs since it booted. */
  micros(): number {
    return this.x.bfw_micros();
  }

  /** The gyro (rad/s) and accelerometer (specific force, m/s²), body forward, right, down. */
  setImu(p: number, q: number, r: number, fx: number, fy: number, fz: number): void {
    this.x.bfw_set_imu(p, q, r, fx, fy, fz);
  }

  /** The barometer and compass: the height above sea level, m, and the true attitude (body
   * forward-right-down to north-east-down). */
  setAir(alt: number, q: readonly number[]): void {
    this.x.bfw_set_air(alt, q[0]!, q[1]!, q[2]!, q[3]!);
  }

  /** The GPS: degrees, m above sea level, velocity north, east, down. */
  setGps(lat: number, lon: number, alt: number, v: readonly number[]): void {
    this.x.bfw_set_gps(lat, lon, alt, v[0]!, v[1]!, v[2]!);
  }

  /** A frame from the receiver: pulse widths, µs, from channel 1. */
  setRc(us: readonly number[]): void {
    const n = Math.min(us.length, RC_CHANNELS);
    const rc = new Uint16Array(this.x.memory.buffer, this.x.bfw_rc(), RC_CHANNELS);
    for (let i = 0; i < n; i++) rc[i] = us[i]!;
    this.x.bfw_set_rc(n);
  }

  /** The motors' outputs, µs, in Betaflight's order: rear right, front right, rear left,
   * front left. A view: read it before the next call. */
  motors(): Float32Array {
    return new Float32Array(this.x.memory.buffer, this.x.bfw_motors(), 4);
  }

  get armed(): boolean {
    return this.x.bfw_armed() !== 0;
  }

  /** Betaflight's arming disable flags, bit by bit. */
  get armingDisabled(): number {
    return this.x.bfw_arming_disabled() >>> 0;
  }

  /** Send `data` to UART `uart` (1 is UART1). */
  serialWrite(uart: number, data: Uint8Array): void {
    this.bytes(this.x.bfw_io(), data.length).set(data);
    this.x.bfw_serial_write(uart, data.length);
  }

  /** What UART `uart` has sent since the last read: a view, good until the next call. */
  serialRead(uart: number): Uint8Array {
    const n = this.x.bfw_serial_read(uart);
    return this.bytes(this.x.bfw_io(), n);
  }

  // The plant: SkyFlow-Dynamics' flat layouts, views good until the memory grows (take them
  // afresh each frame).

  get state(): Float64Array {
    return this.doubles(this.x.plant_state(), this.stateSize);
  }

  get inputs(): Float64Array {
    return this.doubles(this.x.plant_inputs(), this.inputSize);
  }

  get params(): Float64Array {
    return this.doubles(this.x.plant_params(), this.paramSize);
  }

  /** The IMU after the last `advance`: specific force, then rates, body forward, left, up. */
  get imu(): Float64Array {
    return this.doubles(this.x.plant_imu(), 6);
  }

  /** `steps` steps of `dt` s with the inputs held, in `state`; then the IMU there. */
  advance(dt: number, steps: number, rotorMax: number): void {
    this.x.plant_advance(dt, steps, rotorMax);
  }

  throttleToSpeed(u: number, wMin: number, wMax: number, k: number): number {
    return this.x.plant_throttle_to_speed(u, wMin, wMax, k);
  }
}

/** A WASI or libc call this module has no answer for: "not supported". */
function no(): number {
  return 52; // ENOSYS
}

/** The eeprom image of a fresh Betaflight with `settings` (CLI text) typed in and saved. The
 * instance that flies boots on it: a setting takes effect at boot, as on a board. */
export function configure(
  module: WebAssembly.Module,
  settings: string,
  log?: (line: string) => void,
): Uint8Array {
  const sim = new Sim(module, log);
  sim.boot();
  sim.cli(settings);
  return sim.eeprom();
}
