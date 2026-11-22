// analog-vision's browser half (analog.js, plain JS): the subset the camera feed uses.

declare module "analog-vision" {
  /** The OSD chip's pixels for the feed: a canvas of them, the field line its rows start on, and
   * whether it changed since the feed last took it (the feed clears it). */
  export interface ChipOsd {
    canvas: HTMLCanvasElement | OffscreenCanvas;
    top: number;
    changed: boolean;
  }

  export interface AnalogSettings {
    link?: Record<string, unknown>;
    decoder?: Record<string, unknown>;
    deinterlace?: "weave" | "bob";
    dvr?: { quality?: number; one_field?: boolean; size?: [number, number] } | null;
    seed?: number;
  }

  /** NTSC composite over a 5.8 GHz FM link, decoded again, on the GPU (WebGL2). */
  export class AnalogFeed {
    constructor(
      canvas: HTMLCanvasElement | OffscreenCanvas,
      settings?: AnalogSettings,
      options?: { fit?: boolean },
    );
    /** False where WebGL2 with float render targets and six draw buffers is missing. */
    readonly ok: boolean;
    /** Carrier to noise, dB. */
    cnrDb: number;
    set(settings: AnalogSettings): void;
    /** Start the signal again: the decoder has to find it. */
    reset(): void;
    /** The lines due since the last frame, from `source`, with `osd` put in as the chip does;
     * then the picture to the canvas. */
    frame(source: TexImageSource, osd: ChipOsd | null, now: number): void;
    /** How the signal was for the picture made last: the share of lines synced and in colour. */
    stats(): { synced: number; colour: number };
    draw(): void;
  }

  /** The same picture for a small GPU (a phone): two passes a field, no radio and no search
   * for the sync. It makes its canvas 754 × 480, for the page to scale. */
  export class AnalogLight {
    constructor(canvas: HTMLCanvasElement | OffscreenCanvas, settings?: AnalogSettings);
    /** False where WebGL2 with half-float render targets is missing. */
    readonly ok: boolean;
    /** Carrier to noise, dB. */
    cnrDb: number;
    set(settings: AnalogSettings): void;
    /** Start the signal again. */
    reset(): void;
    /** The fields due since the last frame, from `source` (best 754 × 480), with `osd` put in
     * as the chip does; then the picture to the canvas. */
    frame(source: TexImageSource, osd: ChipOsd | null, now: number): void;
    /** How the signal was for the picture made last: the share of lines synced and in colour. */
    stats(): { synced: number; colour: number };
    draw(): void;
  }

  /** CNR at the receiver over free space, dB. */
  export function linkCnrDb(
    distanceM: number,
    txMw?: number,
    txGainDbi?: number,
    rxGainDbi?: number,
    more?: {
      frequency_hz?: number;
      noise_figure_db?: number;
      if_bandwidth_hz?: number;
      extra_loss_db?: number;
    },
  ): number;
}
